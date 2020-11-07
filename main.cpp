#include <iostream>
#include <vector>                      //向量
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>   
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>

using namespace cv;
using namespace std;

double Distance(Point2f a, Point2f b) ;

int main()
{
	// 读取相机参数
	cv::Mat camera_matrix;
	cv::Mat distortion_coefficients;
	//打开校准文件calib_params.yml，进行读取操作，存储到 fs_calib 中
	cv::FileStorage fs_calib("/home/shouyiyang/桌面/视觉实习生任务/任务5/相机内参及畸变参数.yml", cv::FileStorage::READ);  
	if (fs_calib.isOpened())	//  从YAML中读取相机校准参数          
	{
		fs_calib["camera_matrix"] >> camera_matrix;                        //输出相机内参（包括焦距、畸变、光心等）到camera_matrix
		fs_calib["distortion_coefficients"] >> distortion_coefficients;    //输入变形参数 到distortion_coefficients中
		distortion_coefficients.convertTo( distortion_coefficients, CV_64F);
		camera_matrix.convertTo(camera_matrix, CV_64F);
		cout << "camera _matrix: \n" << camera_matrix<< endl;
		cout << "distortion_coefficients: \n" << distortion_coefficients << endl;
	}
	else cout <<"相机内参及畸变参数.yml 读取异常!" << endl;

   	//截取视频的每一帧图像
	VideoCapture video(0);            // 打开视频文件
	video.open("/home/shouyiyang/桌面/视觉实习生任务/任务5/water01.avi");
	if (!video.isOpened())            // 判断是否打开成功
	{
	    cout << "open video file failed. " << endl;
		return -1;
	}
	else    cout << "成功打开视频文件" << endl;

    //创建接受图像的窗口
	Mat hsvimg;
	Mat src;

	while (video.read(src))
	{
		if (src.empty())	 return -1;                                      // frame不为空时，显示原始图像
		//imshow("src1",src);    
		//矫正图像
		//undistort(src, src, camera_matrix, distortion_coefficients);
                                
                //提取蓝色的图像，并二值化处理
		cvtColor(src, hsvimg, CV_BGR2HSV);               
		vector<Mat> channels;                            
		split(src,channels);
		Mat hue = channels.at(0);
		Mat saturate = channels.at(1);
		Mat value = channels.at(2);
		Mat mask(hue.rows, hue.cols, CV_8UC1, Scalar(0, 0, 0));

		for (int i = 0; i < hue.rows; i++)
		{
			for (int j = 0; j < hue.cols; j++)
			{
				int h = hue.at<uchar>(i, j); 
				int s = saturate.at<uchar>(i, j);
				int v = value.at<uchar>(i, j);
				if (h > 5 && h < 50  &&  s <150  && v > 100 )
					mask.at<uchar>(i, j) = 255;
			}
		}
	        Mat kernel1 = getStructuringElement(MORPH_RECT, Size(10, 10));     //设置内核1
		Mat kernel2 = getStructuringElement(MORPH_RECT, Size(8, 8));   //设置内核2
		Mat kernel3 = getStructuringElement(MORPH_RECT, Size(15, 15)); //设置内核3
		dilate(mask, mask, kernel1); 
		morphologyEx(mask,mask,MORPH_OPEN,kernel2);     //进行开运算(去除小的噪点)
		floodFill(mask, Point(0, 0), Scalar(0));        //漫水法
		morphologyEx(mask, mask, MORPH_CLOSE, kernel3); //闭运算(连接白色区域，减少图形数量)
		//imshow("黑白图像", mask);

		//找到外接矩形
		Mat frame =src.clone();
	        vector<vector<Point>> contours;
	        vector<Vec4i> hireachy;
		vector<RotatedRect> minRects;
		Point2f vertex[4];                              //外接矩形的四个顶点
		findContours(mask, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		
		float getArea;

		for (int i = 0; i < contours.size(); ++i) 
		{

			RotatedRect minRect = minAreaRect(Mat(contours[i]));  //最小外接矩形
			minRect.points(vertex);                               //外接矩形的四个顶点
			getArea = minRect.size.width * minRect.size.height;
			if (getArea > 1000)
			{
				for (int l = 0; l < 4; l++)
				{
					line(frame, vertex[l], vertex[(l + 1) % 4], Scalar(255, 255, 255), 2);   
				}
			}
			
			minRects.push_back(minRect);

		}
		imshow("frame", frame);

		//画出两矩形的边点和中心点
		float distance;
		float height;
		RotatedRect leftRect, rightRect;  //左右两个矩形
		Point2f leftRect_up;
		Point2f rightRect_up;
		Point2f leftRect_down;
		Point2f rightRect_down;
		Point2f Center; 
		double area[2];
	
		for (int i = 0; i < minRects.size(); ++i) 
		{
			for (int j = i + 1; j < minRects.size(); ++j) 
			{
				int num = 1;    //同一张图上的不同中心点
				int temp[3];
				leftRect = minRects[i];        //遍历所有矩形，两两组合
				rightRect = minRects[j];
				//判断两个矩形的图形大小
				area[0] = leftRect.size.width * leftRect.size.height;
				area[1] = rightRect.size.width * rightRect.size.height;

				if (area[0] > 1000 && area[1] > 1000)
				{
					//得到装甲板中心点的坐标
					Center.x = (minRects[i].center.x + minRects[j].center.x) / 2;
					Center.y = (minRects[i].center.y + minRects[j].center.y) / 2;
					//得到矩形上下边的中点
					leftRect_up.x = minRects[i].center.x;      //左侧矩形上方中点  
					leftRect_up.y = minRects[i].center.y - (minRects[i].size.height + 30) / 2;
					leftRect_down.x = minRects[i].center.x;    //左侧矩形下方中点  
					leftRect_down.y = minRects[i].center.y + (minRects[i].size.height + 30) / 2;
					
					rightRect_up.x = minRects[j].center.x;     //右侧矩形上方中点  
					rightRect_up.y = minRects[j].center.y - (minRects[j].size.height + 30) / 2;
					rightRect_down.x = minRects[j].center.x;   //右侧矩形下方中点  
					rightRect_down.y = minRects[j].center.y + (minRects[j].size.height + 30) / 2;
					
					//画出装甲板的中心
					circle(frame, Center, 2, Scalar(0, 0, 255), 3);
					//画出两侧灯光的中心和连线
					circle(frame, leftRect_up, 2, Scalar(0, 255, 0), 2);
					circle(frame, leftRect_down, 2, Scalar(0, 255, 0), 2);
					circle(frame, rightRect_up, 2, Scalar(0, 255, 0), 2);
					circle(frame, rightRect_down, 2, Scalar(0, 255, 0), 2);
					line(frame, leftRect_up, Center, Scalar(255, 255, 255), 1);
					line(frame, leftRect_down, Center, Scalar(255, 255, 255), 1);
					line(frame, rightRect_up, Center, Scalar(255, 255, 255), 1);
					line(frame, rightRect_down, Center, Scalar(255, 255, 255), 1);
				    // 添加对四个点排序的代码

				}
			}
		}
		imshow("frame（识别装甲板）", frame);
		minRects.clear();                      

		//将控制点在相机坐标系的坐标压入容器points
		vector<Point3f> objP;             //自己定义世界坐标系三维坐标
		vector<Point2f> points;           //相机坐标系的二维坐标
		Mat objM;                         //自己定义世界坐标系矩阵，包括四个点
		Mat rotM, rotT,rvec,tvec;
		
		objP.clear();
		objP.push_back(Point3f(0,0,0));   //自己定义世界坐标系坐标    三维坐标的单位是毫米 注意点的顺序
		objP.push_back(Point3f(67.5,0, 0));
		objP.push_back(Point3f(67.5,26.5, 0));
		objP.push_back(Point3f(0,26.5, 0));
		Mat(objP).convertTo(objM, CV_32F);
		
		points.clear();
		points.push_back(leftRect_up);
		points.push_back(leftRect_down);
		points.push_back(rightRect_down);
		points.push_back(rightRect_up);

		//使用pnp解算求出相机到世界坐标系的旋转矩阵和平移矩阵       
		solvePnP(objM, Mat(points), camera_matrix, distortion_coefficients, rvec, tvec);
		Rodrigues(rvec, rotM);  //将旋转向量变换成旋转矩阵
		Rodrigues(tvec, rotT);
		//cout<<"1 rotation vector: "<<endl<<rvec<<endl;  
		cout<<"2 translation vector: "<<endl<<tvec<<endl;  
		cout<<"3 rotation matrix: "<<endl<<rotM<<endl;  
		//cout<<"4 translation matrix: "<<endl<<rotT<<endl;  

		if(waitKey(50) == 27)                                       //esc退出
		{
			break;
		}
    }
    cout << "finish!!" << endl;
    return 0;

}


double Distance(Point2f a, Point2f b)  
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
