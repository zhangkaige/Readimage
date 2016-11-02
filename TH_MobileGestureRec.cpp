// TH_.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cv.h>
#include "watershedSegmenter.h"


using namespace std;
using namespace cv;
void TH_SiIm();


int TH_WaterToCube();
int _tmain(int argc, _TCHAR* argv[])
{
	//TH_WaterToCube();

	//TH_SiIm();
	//����Ƶ�ļ�����ʵ���ǽ���һ��VideoCapture�ṹ
	VideoCapture capture("F:\\VR\\Ӣ÷������\\����ָ.mp4");
	//VideoCapture capture("F:\\VR\\Ӣ÷������\\test.mp4");
	//����Ƿ�������:�ɹ���ʱ��isOpened����ture
	if (!capture.isOpened())
		cout << "fail to open!" << endl;
	//��ȡ����֡��
	long totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
	cout << "������Ƶ��" << totalFrameNumber << "֡" << endl;

	//���ÿ�ʼ֡()
	long frameToStart = 000;
	capture.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
	cout << "�ӵ�" << frameToStart << "֡��ʼ��" << endl;

	//���ý���֡
	int frameToStop = totalFrameNumber;

	if (frameToStop < frameToStart)
	{
		cout << "����֡С�ڿ�ʼ֡��������󣬼����˳���" << endl;
		return -1;
	}
	else
	{
		cout << "����֡Ϊ����" << frameToStop << "֡" << endl;
	}

	//��ȡ֡��
	double rate = capture.get(CV_CAP_PROP_FPS);
	cout << "֡��Ϊ:" << rate << endl;

	//����һ���������ƶ�ȡ��Ƶѭ�������ı���
	bool stop = false;
	//����ÿһ֡��ͼ��
	Mat frame;
	//��ʾÿһ֡�Ĵ���
	//namedWindow("Extracted frame");
	//��֡��ļ��ʱ��:
	int delay = 1000 / rate;

	//����whileѭ����ȡ֡
	//currentFrame����ѭ�����п��ƶ�ȡ��ָ����֡��ѭ�������ı���
	long currentFrame = frameToStart;

	//�˲����ĺ�
	int kernel_size = 3;
	Mat kernel = Mat::ones(kernel_size, kernel_size, CV_32F) / (float)(kernel_size*kernel_size);

	while (!stop)
	{
		//��ȡ��һ֡
		if (!capture.read(frame))
		{
			cout << "��ȡ��Ƶʧ��" << endl;
			return -1;
		}

		//������˲�����
		//imshow("Extracted frame", frame);
		filter2D(frame, frame, -1, kernel);
		Mat grayIm, edgeIm, hsvIm;
		
		cvtColor(frame, grayIm,CV_BGR2GRAY);
		imshow("after filter", frame);

		Canny(grayIm, edgeIm,3,9,3);
		cvtColor(frame, hsvIm, CV_BGR2YCrCb);
		//1.��ָ��ɫ��ȡ
		Mat dst= Mat::zeros(hsvIm.size(), hsvIm.type());

		for (int i = 0; i < dst.rows;i++)
		{
			for (int j = 0; j < dst.cols; j++)
			{
				Vec3b sr = hsvIm.at<Vec3b>(i, j);
				if (sr[1] >= 133 && sr[1] <= 173 && sr[2] >= 77 && sr[2] <= 127)
				{
					dst.at<Vec3b>(i, j) = frame.at<Vec3b>(i, j);
				}
			}
		}


		//2.��ָ��Ե���
		Canny(grayIm,edgeIm,50,150,3);

		imshow("��Ե",edgeIm);
		//3.��ָ��λ



		//4.��ָ����
		imshow("�����",dst);

		cout << "���ڶ�ȡ��" << currentFrame << "֡" << endl;
		int c = waitKey(delay);
		//����ESC���ߵ���ָ���Ľ���֡���˳���ȡ��Ƶ
		if ((char)c == 27 || currentFrame > frameToStop)
		{
			stop = true;
		}
		//���°������ͣ���ڵ�ǰ֡���ȴ���һ�ΰ���
		if (c >= 0)
		{
			waitKey(0);
		}
		currentFrame++;

	}
	//�ر���Ƶ�ļ�
	capture.release();
	waitKey(0);
	return 0;

}

//


//////////////////////////////////////////////////////////////////////////
//����ͼ��
void TH_SiIm()
{
	Mat hsvIm;
	Mat img = imread("F:/VR/Ӣ÷������/hands.jpg");
	cvtColor(img, hsvIm, CV_BGR2YCrCb);
	//1.��ָ��ɫ��ȡ
	Mat dst = Mat::zeros(hsvIm.size(), hsvIm.type());

	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			Vec3b sr = hsvIm.at<Vec3b>(i, j);
			if (sr[1] >= 133 && sr[1] <= 173 && sr[2] >= 77 && sr[2] <= 127)
			{
				dst.at<Vec3b>(i, j) = img.at<Vec3b>(i, j);
			}
		}
	}
	//4.��ָ����
	imshow("�����", dst);
	waitKey(0);
}

//void cvSkinYUV(IplImage* src, IplImage* dst)
//{
//	IplImage* ycrcb = cvCreateImage(cvGetSize(src), 8, 3);
//	//IplImage* cr=cvCreateImage(cvGetSize(src),8,1);    
//	//IplImage* cb=cvCreateImage(cvGetSize(src),8,1);    
//	cvCvtColor(src, ycrcb, CV_BGR2YCrCb);
//	//cvSplit(ycrcb,0,cr,cb,0);    
//
//	static const int Cb = 2;
//	static const int Cr = 1;
//	static const int Y = 0;
//
//	//IplImage* dst=cvCreateImage(cvGetSize(_dst),8,3);    
//	cvZero(dst);
//
//	for (int h = 0; h < src->height; h++)
//	{
//		unsigned char* pycrcb = (unsigned char*)ycrcb->imageData + h*ycrcb->widthStep;
//		unsigned char* psrc = (unsigned char*)src->imageData + h*src->widthStep;
//		unsigned char* pdst = (unsigned char*)dst->imageData + h*dst->widthStep;
//		for (int w = 0; w < src->width; w++) {
//			if (pycrcb[Cr] >= 133 && pycrcb[Cr] <= 173 && pycrcb[Cb] >= 77 && pycrcb[Cb] <= 127)
//			{
//				memcpy(pdst, psrc, 3);
//			}
//			pycrcb += 3;
//			psrc += 3;
//			pdst += 3;
//		}
//	}
//	//cvCopyImage(dst,_dst);    
//	//cvReleaseImage(&dst);    
//}


//�Ҷ�ͼ������Ч��ͼ

int TH_WaterToCube()
{
	//1.��ȡͼ��
	// Read input image ԭͼ
	cv::Mat image = cv::imread("F:/VR/Ӣ÷������/hands.jpg");
	if (!image.data)
		return 0;

	// Display the image
	cv::namedWindow("Original Image");
	cv::imshow("Original Image", image);


	// ��ֵͼ ������������ط�ת����Ϊһ��������255��ɫ��ʾǰ�������壩����0��ɫ��ʾ����
	cv::Mat binary;
	binary = cv::imread("F:/VR/Ӣ÷������/hands.jpg", 0);

	// Display the binary image
	cv::namedWindow("Binary Image");
	cv::imshow("Binary Image", binary);
	//2.�Ҷ�ת��

	// �ɶ�ֵͼ����ǰ������ʴ���Ƴ������΢С����
	cv::Mat fg;
	cv::erode(binary, fg, cv::Mat(), cv::Point(-1, -1), 6);

	// Display the foreground image
	cv::namedWindow("Foreground Image");
	cv::imshow("Foreground Image", fg);
	//3.��ˮ������ͼ��

	//

	//���Ͷ�ֵͼ����ȡ������ֻ�вݵأ�û�����֣�
	cv::Mat bg;
	cv::dilate(binary, bg, cv::Mat(), cv::Point(-1, -1), 6);
	cv::threshold(bg, bg, 1, 128, cv::THRESH_BINARY_INV);
	//���һ�������ı�ʾ ifsrc>1,dst=0,else dst=128��������ʹ����ȫΪ��ɫ��128��
	// Display the background image
	cv::namedWindow("Background Image");
	cv::imshow("Background Image", bg);

	// Show markers image
	cv::Mat markers(binary.size(), CV_8U, cv::Scalar(0));
	markers = fg + bg;//ʹ�����ز�����+
	cv::namedWindow("Markers");
	cv::imshow("Markers", markers);

	// Create watershed segmentation object
	WatershedSegmenter segmenter;

	// Set markers and process
	segmenter.setMarkers(markers);
	segmenter.process(image);

	// Display segmentation result
	cv::namedWindow("Segmentation");
	cv::imshow("Segmentation", segmenter.getSegmentation());
	waitKey(0);
	return 0;

}

//ͼ��ƴ�Ӽ���
void TH_ImgPinjie()
{

}