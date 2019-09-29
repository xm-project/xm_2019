#include <iostream>
#include <Processor.h>
#include <Camera.h>
#include <time.h>

std::vector<std::string> name_init();
int main()
{
    
    Vision::Camera cam;
    if(!cam.openCamera())
    {
    	std::cout<<"camera open failed"<<std::endl;
        std::cout<<"Task:detetct_all failed! "<<std::endl;
	    return 0;
    }
    
    Vision::Processor processor("object");      //处理类初始化
    std::vector<std::string>vec = name_init();//清单名称初始化
    processor.setObjectName(vec);

    int count = 1;
    int obj_num=0;
    cv::Mat frame;
    cv::Mat result_img;
    std::vector<std::string> detect_name;
    std::vector<std::string> result_name;

    while (count<=30)       //检测30次,记录下检测结果最好，即检测出数目最多的一次，
    {
	    count++;
	    cam.transColorImg().copyTo(frame);
        processor.passMat(frame);
        detect_name = processor.getAllObject();

        //改动
        obj_num = detect_name.size();
        result_name = detect_name;
        std::cout<<"the number of objects detected is "<<obj_num<<std::endl;
    }

    std::ofstream outfile("/home/domistic/Vision/data/result/out.txt", ios::trunc ); //将检测到物品名称保存txt
    for(int i=0;i<result_name.size();i++)
    {
        outfile << result_name[i]; 
        outfile << std::endl;
    }
    outfile.close();
    processor.drawRect_known();
    result_img =  processor.getDetectedImage(); 
    std::cout<<"已知物体已检测完毕"<<std::endl;

    cv::imwrite("/home/domistic/Vision/data/result/detect_all_result.jpg",result_img);
    cv::resize(result_img,result_img,cv::Size(result_img.cols/4,result_img.rows/4),0,0);   
    cv::imshow("detect",result_img);
    cv::moveWindow("detect",0,0);

    cv::Mat depth_img;
    cv::Mat result_img_2;
    cam.transDepthImg().copyTo(depth_img);
    processor.passDepthMat(depth_img);
    std::vector<cv::Rect> detCordinate;             //深度物体坐标xy
    detCordinate = processor.depth_handle(); 
    processor.unknown_color();
    processor.drawRect_unknown();
    result_img_2 =  processor.getDetectedImage(); 

    cv::imwrite("/home/domistic/Vision/data/result/detect_all_result_with_unknown.jpg",result_img_2);
    cv::resize(result_img_2,result_img_2,cv::Size(result_img_2.cols/4,result_img_2.rows/4),0,0);    
    cv::imshow("unkonwn",result_img_2);
    cv::moveWindow("unkonwn",0,0);
    std::cout<<"检测结束"<<std::endl;

    cv::waitKey(3000);
    cam.closeCamera();
    std::cout<<"Task:detetct_all finish! "<<std::endl;
    std::cout<<"the number of objects detected is "<<obj_num<<std::endl;
    return 0;

}


std::vector<std::string> name_init()
{
    std::vector<std::string>vec;
    std::string s;
    std::ifstream inf;
    inf.open("/home/domistic/Vision/data/object_name.txt");
    while(std::getline(inf,s))
    {
    vec.push_back(s);
    }
    inf.close();    
    std::cout<<"the number of objects is :"<<vec.size();
    std::cout<<std::endl;
    return vec;
}
