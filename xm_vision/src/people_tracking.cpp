#include <ros/ros.h>
#include "std_msgs/String.h"

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Camera.h>
#include <time.h>
#include <unistd.h>
#include "yolo_v2_class.hpp" 

#include <xm_msgs/xm_People.h>
#include <xm_msgs/xm_Object.h>
#include <xm_msgs/xm_ObjectDetect.h>
#include <xm_msgs/xm_FollowPerson.h>

using namespace Vision;

cv::Rect Filter(std::vector<bbox_t> result_vec)
{
	cv::Rect rect = cv::Rect(0,0,0,0);
	double max_area = 0;
	for(auto &i : result_vec){
		if(i.obj_id==0){
			double area = i.w * i.h;
			if(area<100000) continue;
			if(area>max_area){
				max_area = area;
				rect = cv::Rect(i.x, i.y, i.w, i.h);
			}
		}
	}
	return rect;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "people_tracking");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<xm_msgs::xm_FollowPerson>("follow", 1);
    ros::Rate loop_rate(5);
    xm_msgs::xm_FollowPerson pos_xyz;

    std::string  cfg_file = "/home/domistic/darknet/cfg/yolov3.cfg";
    std::string  weights_file = "/home/domistic/darknet/yolov3.weights";
	float const thresh = 0.2;
    Detector detector(cfg_file, weights_file);

	cv::Mat frame;
	Camera cam;
	if(!cam.openCamera())
	{
		std::cout<<"cam failure"<<std::endl;
		return 0;
	}
	cv::namedWindow("Tracking",0);
	while(1){
		char ch;
		cam.transColorImg().copyTo(frame);
		// cv::Mat frame = cv::imread("/home/domistic/darknet/data/frame_pscene.jpg");
		// cv::imwrite("/home/domistic/Vision/Frame/frame_tracking.jpg",frame);
		std::vector<bbox_t> result_vec = detector.detect(frame);
		cv::Rect rect = Filter(result_vec);
		cv::rectangle(frame, rect, cv::Scalar(255,255,0), 4);
		cv::imwrite("/home/domistic/Vision/Result/result_tracking.jpg",frame);
        std::vector<cv::Rect> detCordinate;

		detCordinate.push_back(rect);
		std::vector<Vision::Cordinate3D> targetCor;
		cam.getCordinate3D(detCordinate,targetCor);
		// if(targetCor[0].z<0.7){
		// 	targetCor[0].x = 10;
		// 	targetCor[0].y = 10;
		// 	targetCor[0].z = 10;
		// }


		cv::imshow("Tracking",frame);
		ch = cv::waitKey(1);
			if (ch == 'q')
				break;
		pos_xyz.position.point.x = targetCor[0].x;
		pos_xyz.position.point.y = targetCor[0].y;
		pos_xyz.position.point.z = targetCor[0].z;
		pos_xyz.position.header.frame_id = "camera_link";
		pos_xyz.position.header.stamp = ros::Time::now();
		printf("people_position:%lf, %lf, %lf\n", pos_xyz.position.point.x, 
												pos_xyz.position.point.y, 
												pos_xyz.position.point.z);
		chatter_pub.publish(pos_xyz);
		ros::spinOnce();  
		loop_rate.sleep();
	}
	cam.closeCamera();
	std::cout<<"finishfinish"<<std::endl;

	return 0;
}
