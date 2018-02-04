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

//�򿪴���
bool OpenPort()
{
	//HANDLE hComm;
	hComm = CreateFile(L"COM3",   //���ڱ��  
		GENERIC_READ | GENERIC_WRITE,  //�����д  
		0,   //ͨѶ�豸�����Զ�ռ��ʽ��  
		NULL,
		OPEN_EXISTING,   //ͨѶ�豸�Ѵ���  
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,   //�ص���ʽ  
		NULL);   //ͨѶ�豸������ģ���  
	if (hComm == INVALID_HANDLE_VALUE)
	{
		CloseHandle(hComm);
		return FALSE;
	}
	else
		return TRUE;
}

//���ô���
bool SetupDCB(int rate_arg)
{
	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	if (!GetCommState(hComm, &dcb))//��ȡ��ǰDCB����  
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

//���ó�ʱ
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
	printf("�ѷ��ͱ�������+CMGS\n");
}

void sendMSG(){
	bResult = WriteFile(hComm, "warning", sizeof("warning"), 0, &OverLapped);
	FlushFileBuffers(hComm);
	Sleep(1000);
	printf("�ѷ��ͱ�������+Warning\n");
}

void sendEnd(){
	BYTE over[1];
	over[0] = 0x1A;
	bResult = WriteFile(hComm, over, sizeof(over), 0, &OverLapped);
	FlushFileBuffers(hComm);
	Sleep(1000);
	printf("�ѷ��ͱ�������+End\n");
}

DWORD WINAPI ATsend(LPVOID lpParamter)
{
	sendCMGS();
	sendMSG();
	sendEnd();
	return 0;
}


//�������������������
bool biggerSort(vector<Point> v1, vector<Point> v2)
{
	return contourArea(v1)>contourArea(v2);
}

int main()
{
	//��Ƶ�����ڣ��ͷ���
	//VideoCapture cap("E:\\������Ƶ��\\swa.mp4");
	//VideoCapture cap(0);
	VideoCapture cap("D:\Opemncn\a.avi");
	if (cap.isOpened() == false)
		return 0;

	//�������
	int i;

	Mat frame;			//��ǰ֡
	Mat foreground;		//ǰ��
	Mat bw;				//�м��ֵ����
	Mat se;				//��̬ѧ�ṹԪ��

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

	//�û�ϸ�˹ģ��ѵ������ͼ��
	Ptr<BackgroundSubtractorMOG2>  mog = createBackgroundSubtractorMOG2();
	mog->setVarThreshold(20);
	for (i = 0; i<10; ++i)
	{
		cout << "����ѵ������:" << i << endl;
		cap >> frame;

		if (frame.empty() == true)
		{
			cout << "��Ƶ̫֡�٣��޷�ѵ������" << endl;
			getchar();
			return 0;
		}
		mog->apply(frame, foreground, 0.01);
		//	mog(frame, foreground, 0.01);
	}

	//Ŀ����ӿ����ɽṹԪ�أ��������ӶϿ���СĿ�꣩
	Rect rt;
	se = getStructuringElement(MORPH_RECT, Size(5, 5));

	//ͳ��Ŀ��ֱ��ͼʱʹ�õ��ı���
	vector<Mat> vecImg;
	vector<int> vecChannel;
	vector<int> vecHistSize;
	vector<float> vecRange;
	Mat mask(frame.rows, frame.cols, DataType<uchar>::type);
	//������ʼ��
	vecChannel.push_back(0);
	vecHistSize.push_back(32);
	vecRange.push_back(0);
	vecRange.push_back(180);

	Mat hsv;		//HSV��ɫ�ռ䣬��ɫ��H�ϸ���Ŀ�꣨camshift�ǻ�����ɫֱ��ͼ���㷨��
	MatND hist;		//ֱ��ͼ����
	double maxVal;		//ֱ��ͼ���ֵ��Ϊ�˱���ͶӰͼ��ʾ����Ҫ��ֱ��ͼ��һ����[0 255]������
	Mat backP;		//����ͶӰͼ
	Mat result;		//���ٽ��

	int countframe = 0;
	//��Ƶ��������
	while (1)
	{
		//����Ƶ
		cap >> frame;
		if (frame.empty() == true)
			break;

		countframe++;
		//���ɽ��ͼ
		frame.copyTo(result);

		//���Ŀ��(��ʵ�Ǳ�ѵ���߼��)
		mog->apply(frame, foreground, 0.01);
		//mog(frame, foreground, 0.01);
		imshow("��ϸ�˹���ǰ��", foreground);
		moveWindow("��ϸ�˹���ǰ��", 400, 0);
		//��ǰ��������ֵ�˲�����̬ѧ���Ͳ�������ȥ��αĿ��ͽ����Ͽ���СĿ��		
		medianBlur(foreground, foreground, 5);
		imshow("��ֵ�˲�", foreground);
		moveWindow("��ֵ�˲�", 800, 0);
		morphologyEx(foreground, foreground, MORPH_DILATE, se);

		//����ǰ���и�����ͨ����������
		foreground.copyTo(bw);
		vector<vector<Point>> contours;
		findContours(bw, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		if (contours.size()<1)
			continue;
		//����ͨ������������
		std::sort(contours.begin(), contours.end(), biggerSort);

		//���camshift���¸���λ�ã�����camshift�㷨�ڵ�һ�����£�����Ч���ǳ��ã�
		//�����ڼ����Ƶ�У����ڷֱ���̫�͡���Ƶ����̫�Ŀ��̫��Ŀ����ɫ��������
		//�ȸ������أ����¸���Ч���ǳ��  ��ˣ���Ҫ�߸��١��߼�⣬������ٲ����ã�
		//���ü��λ���޸�
		cvtColor(frame, hsv, COLOR_BGR2HSV);
		vecImg.clear();
		vecImg.push_back(hsv);
		for (int k = 0; k<contours.size(); ++k)
		{
			//��k����ͨ��������Ӿ��ο�
			if (contourArea(contours[k])<contourArea(contours[0]) / 10)
				break;
			rt = boundingRect(contours[k]);
			mask = 0;
			mask(rt) = 255;

			//ͳ��ֱ��ͼ
			calcHist(vecImg, vecChannel, mask, hist, vecHistSize, vecRange);
			minMaxLoc(hist, 0, &maxVal);
			hist = hist * 255 / maxVal;
			//���㷴��ͶӰͼ
			calcBackProject(vecImg, vecChannel, hist, backP, vecRange, 1);
			//camshift����λ��
			Rect search = rt;
			RotatedRect rrt = CamShift(backP, search, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1));
			Rect rt2 = rrt.boundingRect();
			rt &= rt2;

			//���ٿ򻭵���Ƶ��
			rectangle(result, rt, Scalar(0, 255, 0), 8);
		}

		if (countframe == 25)
		{
			HANDLE hsendAT = CreateThread(NULL, 0, ATsend, NULL, 0, NULL);
			CloseHandle(hsendAT);

			countframe = 0;
		}

		//�����ʾ
		imshow("ԭͼ", frame);
		moveWindow("ԭͼ", 0, 0);

		imshow("��������", foreground);
		moveWindow("��������", 0, 350);

		imshow("����ͶӰ", backP);
		moveWindow("����ͶӰ", 400, 350);

		imshow("����Ч��", result);
		moveWindow("����Ч��", 800, 350);
		waitKey(30);
	}

	getchar();
	return 0;
}
