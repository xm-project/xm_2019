#include <iostream>
#include <Camera.h>
#include "inc/samplecode.h"

int main()
{
    Vision::Camera cam;
    cv::Mat rgbmat;
    // cv::namedWindow("windows",0);
    // cv::namedWindow("test",0);
    if(!cam.openCamera())
    {
    	std::cout<<"camera open failed"<<std::endl;
	return 0;
    }

    int count = 0;
    while(count<25){
	count++;
        cam.transColorImg().copyTo(rgbmat);
        resize(rgbmat,rgbmat,cv::Size(0.5*rgbmat.cols,0.5*rgbmat.rows));
        //imshow("rgb", rgbmat);
    }
    imwrite("/home/domistic/Vision/data/gender_and_pose/output.bmp", rgbmat);
    imwrite("/home/domistic/Vision/data/gender_and_pose/pose/pose_img/image_for_pose.jpg", rgbmat);
    cam.closeCamera();
    gender_draw(rgbmat);
    cv::imshow("rgb", rgbmat);
    cv::waitKey(500);
    return 0;
}
