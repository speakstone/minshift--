#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <cv.h>    
#include <highgui.h>      
#include <string>    
#include <iostream>    
#include <algorithm>    
#include <iterator>   
#include <stdio.h>    
#include <ctype.h>   
#include <time.h> 
#include <math.h> 
#include<windows.h>
#include <mmsystem.h>
#include <direct.h>//for mk_dir
#include <io.h>//for _acess()
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include <vector>
#include<opencv2/imgproc/types_c.h>
#include "imgproc/imgproc.hpp"
#include "video/tracking.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2\objdetect\objdetect_c.h>

#include <Windows.h>

using namespace std;
using namespace cv;

HANDLE hComm;
OVERLAPPED OverLapped;
COMSTAT Comstat;
DWORD dwCommEvents;

bool bResult;

//打开串口
bool OpenPort()
{
	//HANDLE hComm;
	hComm = CreateFile(L"COM3",   //串口编号  
		GENERIC_READ | GENERIC_WRITE,  //允许读写  
		0,   //通讯设备必须以独占方式打开  
		NULL,
		OPEN_EXISTING,   //通讯设备已存在  
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,   //重叠方式  
		NULL);   //通讯设备不能用模板打开  
	if (hComm == INVALID_HANDLE_VALUE)
	{
		CloseHandle(hComm);
		return FALSE;
	}
	else
		return TRUE;
}

//配置串口
bool SetupDCB(int rate_arg)
{
	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	if (!GetCommState(hComm, &dcb))//获取当前DCB配置  
	{
		return FALSE;
	}
	dcb.DCBlength = sizeof(dcb);
	/* ---------- Serial Port Config ------- */
	dcb.BaudRate = rate_arg;
	dcb.Parity = NOPARITY;
	dcb.fParity = 0;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;
	dcb.fOutxCtsFlow = 0;
	dcb.fOutxDsrFlow = 0;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = 0;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = 0;
	dcb.fInX = 0;
	dcb.fErrorChar = 0;
	dcb.fBinary = 1;
	dcb.fNull = 0;
	dcb.fAbortOnError = 0;
	dcb.wReserved = 0;
	dcb.XonLim = 2;
	dcb.XoffLim = 4;
	dcb.XonChar = 0x13;
	dcb.XoffChar = 0x19;
	dcb.EvtChar = 0;
	if (!SetCommState(hComm, &dcb))
	{
		return false;
	}
	else
		return true;
}

//配置超时
bool SetupTimeout(DWORD ReadInterval, DWORD ReadTotalMultiplier, DWORD
	ReadTotalConstant, DWORD WriteTotalMultiplier, DWORD WriteTotalConstant)
{
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = ReadInterval;
	timeouts.ReadTotalTimeoutConstant = ReadTotalConstant;
	timeouts.ReadTotalTimeoutMultiplier = ReadTotalMultiplier;
	timeouts.WriteTotalTimeoutConstant = WriteTotalConstant;
	timeouts.WriteTotalTimeoutMultiplier = WriteTotalMultiplier;
	if (!SetCommTimeouts(hComm, &timeouts))
	{
		return false;
	}
	else
		return true;
}

void sendCMGS(){
	char SendData[512] = { 0 };
	sprintf_s(SendData, "AT+CMGS=\"%s\"\r", "15005284283");
	bResult = WriteFile(hComm, SendData, sizeof(SendData), 0, &OverLapped);
	FlushFileBuffers(hComm);
	Sleep(1000);
	printf("已发送报警命令+CMGS\n");
}

void sendMSG(){
	bResult = WriteFile(hComm, "warning", sizeof("warning"), 0, &OverLapped);
	FlushFileBuffers(hComm);
	Sleep(1000);
	printf("已发送报警命令+Warning\n");
}

void sendEnd(){
	BYTE over[1];
	over[0] = 0x1A;
	bResult = WriteFile(hComm, over, sizeof(over), 0, &OverLapped);
	FlushFileBuffers(hComm);
	Sleep(1000);
	printf("已发送报警命令+End\n");
}

DWORD WINAPI ATsend(LPVOID lpParamter)
{
	sendCMGS();
	sendMSG();
	sendEnd();
	return 0;
}


//对轮廓按面积降序排列
bool biggerSort(vector<Point> v1, vector<Point> v2)
{
	return contourArea(v1)>contourArea(v2);
}

int main()
{
	//视频不存在，就返回
	//VideoCapture cap("E:\\论文视频库\\swa.mp4");
	//VideoCapture cap(0);
	VideoCapture cap("D:\Opemncn\a.avi");
	if (cap.isOpened() == false)
		return 0;

	//定义变量
	int i;

	Mat frame;			//当前帧
	Mat foreground;		//前景
	Mat bw;				//中间二值变量
	Mat se;				//形态学结构元素

	//serial com
	if (OpenPort())
		printf("Open port success");
	else
		printf("Open port fail");
	if (SetupDCB(9600))
		printf("Set DCB success");
	if (SetupTimeout(0, 0, 0, 0, 0))
		printf("Set timeout success");
	PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	//用混合高斯模型训练背景图像
	Ptr<BackgroundSubtractorMOG2>  mog = createBackgroundSubtractorMOG2();
	mog->setVarThreshold(20);
	for (i = 0; i<10; ++i)
	{
		cout << "正在训练背景:" << i << endl;
		cap >> frame;

		if (frame.empty() == true)
		{
			cout << "视频帧太少，无法训练背景" << endl;
			getchar();
			return 0;
		}
		mog->apply(frame, foreground, 0.01);
		//	mog(frame, foreground, 0.01);
	}

	//目标外接框、生成结构元素（用于连接断开的小目标）
	Rect rt;
	se = getStructuringElement(MORPH_RECT, Size(5, 5));

	//统计目标直方图时使用到的变量
	vector<Mat> vecImg;
	vector<int> vecChannel;
	vector<int> vecHistSize;
	vector<float> vecRange;
	Mat mask(frame.rows, frame.cols, DataType<uchar>::type);
	//变量初始化
	vecChannel.push_back(0);
	vecHistSize.push_back(32);
	vecRange.push_back(0);
	vecRange.push_back(180);

	Mat hsv;		//HSV颜色空间，在色调H上跟踪目标（camshift是基于颜色直方图的算法）
	MatND hist;		//直方图数组
	double maxVal;		//直方图最大值，为了便于投影图显示，需要将直方图规一化到[0 255]区间上
	Mat backP;		//反射投影图
	Mat result;		//跟踪结果

	int countframe = 0;
	//视频处理流程
	while (1)
	{
		//读视频
		cap >> frame;
		if (frame.empty() == true)
			break;

		countframe++;
		//生成结果图
		frame.copyTo(result);

		//检测目标(其实是边训练边检测)
		mog->apply(frame, foreground, 0.01);
		//mog(frame, foreground, 0.01);
		imshow("混合高斯检测前景", foreground);
		moveWindow("混合高斯检测前景", 400, 0);
		//对前景进行中值滤波、形态学膨胀操作，以去除伪目标和接连断开的小目标		
		medianBlur(foreground, foreground, 5);
		imshow("中值滤波", foreground);
		moveWindow("中值滤波", 800, 0);
		morphologyEx(foreground, foreground, MORPH_DILATE, se);

		//检索前景中各个连通分量的轮廓
		foreground.copyTo(bw);
		vector<vector<Point>> contours;
		findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		if (contours.size()<1)
			continue;
		//对连通分量进行排序
		std::sort(contours.begin(), contours.end(), biggerSort);

		//结合camshift更新跟踪位置（由于camshift算法在单一背景下，跟踪效果非常好；
		//但是在监控视频中，由于分辨率太低、视频质量太差、目标太大、目标颜色不够显著
		//等各种因素，导致跟踪效果非常差。  因此，需要边跟踪、边检测，如果跟踪不够好，
		//就用检测位置修改
		cvtColor(frame, hsv, COLOR_BGR2HSV);
		vecImg.clear();
		vecImg.push_back(hsv);
		for (int k = 0; k<contours.size(); ++k)
		{
			//第k个连通分量的外接矩形框
			if (contourArea(contours[k])<contourArea(contours[0]) / 10)
				break;
			rt = boundingRect(contours[k]);
			mask = 0;
			mask(rt) = 255;

			//统计直方图
			calcHist(vecImg, vecChannel, mask, hist, vecHistSize, vecRange);
			minMaxLoc(hist, 0, &maxVal);
			hist = hist * 255 / maxVal;
			//计算反向投影图
			calcBackProject(vecImg, vecChannel, hist, backP, vecRange, 1);
			//camshift跟踪位置
			Rect search = rt;
			RotatedRect rrt = CamShift(backP, search, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1));
			Rect rt2 = rrt.boundingRect();
			rt &= rt2;

			//跟踪框画到视频上
			rectangle(result, rt, Scalar(0, 255, 0), 8);
		}

		if (countframe == 25)
		{
			HANDLE hsendAT = CreateThread(NULL, 0, ATsend, NULL, 0, NULL);
			CloseHandle(hsendAT);

			countframe = 0;
		}

		//结果显示
		imshow("原图", frame);
		moveWindow("原图", 0, 0);

		imshow("膨胀运算", foreground);
		moveWindow("膨胀运算", 0, 350);

		imshow("反向投影", backP);
		moveWindow("反向投影", 400, 350);

		imshow("跟踪效果", result);
		moveWindow("跟踪效果", 800, 350);
		waitKey(30);
	}

	getchar();
	return 0;
}
