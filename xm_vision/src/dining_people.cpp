#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <Processor.h>
#include <Camera.h>
#include <time.h>
#include <fstream>  //ifstream
#include <string>    
#include <xm_msgs/xm_Object.h>
#include <xm_msgs/xm_ObjectDetect.h>

int if_detected = 0;


std::string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp,sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep));
    return tmp;
}


std::vector<std::string> name_init();
bool callback(xm_msgs::xm_ObjectDetect::Request &req, 
                      xm_msgs::xm_ObjectDetect::Response &res);
int main(int argc, char **argv)
{ 

    int count = 0;
    ros::init(argc, argv, "dining_people");  
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("get_position", &callback);

    while(1)
    {
        count++;
        if(if_detected)
        {
            break;
        }
        else
        {
            std::cout<<"time out!"<<std::endl;
            std::cout<<count<<std::endl;
            
            ros::spinOnce();
        }

    }
  return 0;
}

bool callback(xm_msgs::xm_ObjectDetect::Request &req, 
                      xm_msgs::xm_ObjectDetect::Response &res)
{
    Vision::Camera cam;
    Vision::Processor processor("person");
    std::vector<std::string>vec;//清单名称初始化
    vec.push_back("person");
    processor.setObjectName(vec);

    xm_msgs::xm_Object people_tmp;
    int count = 1;

    processor.setDetectName("person")  ; //检测目标

    cv::Mat frame;
    std::vector<cv::Rect> detCordinate;
    std::vector<Vision::Cordinate3D> targetCor;
    // cv::namedWindow("windows",0);
    if(!cam.openCamera())
    {
    	std::cout<<"camera open failed"<<std::endl;
        cam.closeCamera();
	    return false;
    }
    std::cout<<"***hello***"<<std::endl;

    while(/*!if_detected &&*/count <= 10)  //检测多少次停止
    {
        if(count<=5){       //前五帧准备，最多十帧
            count++;
            continue;
        }
        count++;
        targetCor.clear();
        detCordinate.clear();
        cam.transColorImg().copyTo(frame);

        processor.passMat(frame);
        detCordinate = processor.dining_people();
        std::cout<<detCordinate.size()<<std::endl;
        if(detCordinate.size() == 0)
        {
            std::cout<<"there is no people need food."<<std::endl;
            continue;

        }
        cam.getCordinate3D(detCordinate,targetCor);
        for(int i = 0; i<targetCor.size() ; i++)
        {
            std::cout<<"x:"<<targetCor[i].y<<"  y:"<<targetCor[i].x<<"  z:"<<targetCor[i].z<<std::endl;
        }
        processor.draw_dining_people();
        char ch;
        ch = cv::waitKey(1);
        if (ch == 'q')
            break;
        cv::Mat result;
        processor.getImage().copyTo(result);
        cv::imshow("windows", result);
        if(detCordinate.size() != 0)
        {
            break;
        }

    }

    cam.closeCamera();

    cv::Mat result = processor.getImage();
    std::string time = getTime();
    cv::imwrite("/home/domistic/Vision/Body/dining_people"+time+".jpg",result);
    cv::resize(result,result,cv::Size(result.cols/2,result.rows/2),0,0);    
    cv::imshow("windows", result);
    cv::moveWindow("windows",0,0);
    cv::waitKey(1000);
    // obj_tmp.pos.point.y = 1;//x   //此处默认返回检测到的第一个任务物品的位置，后续优化
    // obj_tmp.pos.point.z = 1;//y
    // obj_tmp.pos.point.x = 1;//z
    people_tmp.pos.point.x = targetCor[0].x;//x   //此处默认返回检测到的第一个任务物品的位置，后续优化
    people_tmp.pos.point.y = targetCor[0].y;//y
    people_tmp.pos.point.z = targetCor[0].z;//z
    people_tmp.pos.header.frame_id = "camera_link";
    people_tmp.pos.header.stamp = ros::Time::now();

    if_detected = 1;
    res.object.push_back(people_tmp);
    std::cout<<"finish!"<<std::endl;
    return true;

}
