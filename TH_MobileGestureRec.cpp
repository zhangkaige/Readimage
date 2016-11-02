// TH_.cpp : 定义控制台应用程序的入口点。
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
	//打开视频文件：其实就是建立一个VideoCapture结构
	VideoCapture capture("F:\\VR\\英梅吉资料\\单手指.mp4");
	//VideoCapture capture("F:\\VR\\英梅吉资料\\test.mp4");
	//检测是否正常打开:成功打开时，isOpened返回ture
	if (!capture.isOpened())
		cout << "fail to open!" << endl;
	//获取整个帧数
	long totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
	cout << "整个视频共" << totalFrameNumber << "帧" << endl;

	//设置开始帧()
	long frameToStart = 000;
	capture.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
	cout << "从第" << frameToStart << "帧开始读" << endl;

	//设置结束帧
	int frameToStop = totalFrameNumber;

	if (frameToStop < frameToStart)
	{
		cout << "结束帧小于开始帧，程序错误，即将退出！" << endl;
		return -1;
	}
	else
	{
		cout << "结束帧为：第" << frameToStop << "帧" << endl;
	}

	//获取帧率
	double rate = capture.get(CV_CAP_PROP_FPS);
	cout << "帧率为:" << rate << endl;

	//定义一个用来控制读取视频循环结束的变量
	bool stop = false;
	//承载每一帧的图像
	Mat frame;
	//显示每一帧的窗口
	//namedWindow("Extracted frame");
	//两帧间的间隔时间:
	int delay = 1000 / rate;

	//利用while循环读取帧
	//currentFrame是在循环体中控制读取到指定的帧后循环结束的变量
	long currentFrame = frameToStart;

	//滤波器的核
	int kernel_size = 3;
	Mat kernel = Mat::ones(kernel_size, kernel_size, CV_32F) / (float)(kernel_size*kernel_size);

	while (!stop)
	{
		//读取下一帧
		if (!capture.read(frame))
		{
			cout << "读取视频失败" << endl;
			return -1;
		}

		//这里加滤波程序
		//imshow("Extracted frame", frame);
		filter2D(frame, frame, -1, kernel);
		Mat grayIm, edgeIm, hsvIm;
		
		cvtColor(frame, grayIm,CV_BGR2GRAY);
		imshow("after filter", frame);

		Canny(grayIm, edgeIm,3,9,3);
		cvtColor(frame, hsvIm, CV_BGR2YCrCb);
		//1.手指肤色提取
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


		//2.手指边缘检测
		Canny(grayIm,edgeIm,50,150,3);

		imshow("边缘",edgeIm);
		//3.手指定位



		//4.手指跟踪
		imshow("检测结果",dst);

		cout << "正在读取第" << currentFrame << "帧" << endl;
		int c = waitKey(delay);
		//按下ESC或者到达指定的结束帧后退出读取视频
		if ((char)c == 27 || currentFrame > frameToStop)
		{
			stop = true;
		}
		//按下按键后会停留在当前帧，等待下一次按键
		if (c >= 0)
		{
			waitKey(0);
		}
		currentFrame++;

	}
	//关闭视频文件
	capture.release();
	waitKey(0);
	return 0;

}

//


//////////////////////////////////////////////////////////////////////////
//理单张图像
void TH_SiIm()
{
	Mat hsvIm;
	Mat img = imread("F:/VR/英梅吉资料/hands.jpg");
	cvtColor(img, hsvIm, CV_BGR2YCrCb);
	//1.手指肤色提取
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
	//4.手指跟踪
	imshow("检测结果", dst);
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


//灰度图像立体效果图

int TH_WaterToCube()
{
	//1.读取图像
	// Read input image 原图
	cv::Mat image = cv::imread("F:/VR/英梅吉资料/hands.jpg");
	if (!image.data)
		return 0;

	// Display the image
	cv::namedWindow("Original Image");
	cv::imshow("Original Image", image);


	// 二值图 这里进行了像素反转，因为一般我们用255白色表示前景（物体），用0黑色表示背景
	cv::Mat binary;
	binary = cv::imread("F:/VR/英梅吉资料/hands.jpg", 0);

	// Display the binary image
	cv::namedWindow("Binary Image");
	cv::imshow("Binary Image", binary);
	//2.灰度转换

	// 由二值图像获得前景。腐蚀。移除噪点与微小物体
	cv::Mat fg;
	cv::erode(binary, fg, cv::Mat(), cv::Point(-1, -1), 6);

	// Display the foreground image
	cv::namedWindow("Foreground Image");
	cv::imshow("Foreground Image", fg);
	//3.分水岭立体图像

	//

	//膨胀二值图来获取背景（只有草地，没有树林）
	cv::Mat bg;
	cv::dilate(binary, bg, cv::Mat(), cv::Point(-1, -1), 6);
	cv::threshold(bg, bg, 1, 128, cv::THRESH_BINARY_INV);
	//最后一个参数的表示 ifsrc>1,dst=0,else dst=128。这样就使背景全为灰色（128）
	// Display the background image
	cv::namedWindow("Background Image");
	cv::imshow("Background Image", bg);

	// Show markers image
	cv::Mat markers(binary.size(), CV_8U, cv::Scalar(0));
	markers = fg + bg;//使用重载操作符+
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

//图像拼接技术
void TH_ImgPinjie()
{

}