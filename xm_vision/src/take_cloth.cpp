//object_detect的ROS服务器节点
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <Processor.h>
#include <Camera.h>
#include <time.h>
#include <fstream>  //ifstream
#include <string>    
#include <xm_msgs/xm_Cloth.h>
#include <xm_msgs/xm_Object.h>

int if_detected = 0;

std::vector<std::string> name_init();
bool callback(xm_msgs::xm_Cloth::Request &req, 
                      xm_msgs::xm_Cloth::Response &res);
int main(int argc, char **argv)
{ 
    int count = 0;
    ros::init(argc, argv, "cloth_detect");  
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

bool callback(xm_msgs::xm_Cloth::Request &req, 
                      xm_msgs::xm_Cloth::Response &res)
{
    Vision::Camera cam;
    Vision::Processor processor("cloth");
    std::vector<std::string>vec = name_init();//清单名称初始化
    processor.setObjectName(vec);

    std::string obj_name_tmp;
    obj_name_tmp = req.object_name;
    xm_msgs::xm_Object obj_tmp;
    int count = 1;
    
    std::cout<<obj_name_tmp<<std::endl;
    processor.setDetectName(obj_name_tmp)  ; //检测目标

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
    std::cout<<"hello"<<std::endl;

    while(count <= 30)  //检测多少次停止
    {
        count++;
        targetCor.clear();
        detCordinate.clear();
        cam.transColorImg().copyTo(frame);

        processor.passMat(frame);
        detCordinate = processor.getBoundingBox();
        std::cout<<detCordinate.size()<<std::endl;
        if(detCordinate.size() == 0)
        {
            std::cout<<"no object"<<std::endl;
            continue;

        }
        cam.getCordinate3D(detCordinate,targetCor);
        for(int i = 0; i<targetCor.size() ; i++)
        {
            std::cout<<"x:"<<targetCor[i].y<<"  y:"<<targetCor[i].x<<"  z:"<<targetCor[i].z<<std::endl;
        }
        processor.drawRect();
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
	std::cout<<count<<std::endl;
    }

    cam.closeCamera();

    cv::Mat result = processor.getImage();
    cv::imwrite("/home/domistic/Vision/data/result/cloth.jpg",result);
    cv::resize(result,result,cv::Size(result.cols/2,result.rows/2),0,0);    
    cv::imshow("windows", result);
    cv::moveWindow("windows",0,0);
    cv::waitKey(1000);

    obj_tmp.pos.point.x = targetCor[0].x;//x   //此处默认返回检测到的第一个任务物品的位置，后续优化
    obj_tmp.pos.point.y = targetCor[0].y;//y
    obj_tmp.pos.point.z = targetCor[0].z;//z
    obj_tmp.pos.header.frame_id = "camera_link";
    obj_tmp.pos.header.stamp = ros::Time::now();

    if_detected = 1;
    res.object.push_back(obj_tmp);
    std::cout<<"Finish"<<std::endl;
    return true;
}

std::vector<std::string> name_init()
{
    std::vector<std::string>vec;
    std::string s;
    std::ifstream inf;
    inf.open("/home/domistic/Vision/data/cloth_name.txt");
    while(std::getline(inf,s))
    {
    vec.push_back(s);
    }
    inf.close();    
    std::cout<<"the number of objects is :"<<vec.size();
    std::cout<<std::endl;
    return vec;
}
