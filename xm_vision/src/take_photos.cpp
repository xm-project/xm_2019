#include <iostream>
#include <Camera.h>
#include <time.h>

std::string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp,sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep));
    return tmp;
}

int main()

{
    
    Vision::Camera cam;
    cv::Mat frame;
    // cv::namedWindow("windows",0);
    // cv::namedWindow("test",0);
    if(!cam.openCamera())
    {
    	std::cout<<"camera open failed"<<std::endl;
	return 0;
    }

    // frame=cv::imread("/home/domistic/pic/1.jpg");
    // cv:: resize(frame,frame,cv::Size(800,600));  
    // cv::imshow("test",frame);
    int pic_num =400;  
   // system("sudo rm     /home/domistic/Vision/data/dataset/");
   // system("sudo mkdir  /home/domistic/Vision/data/dataset");
    cv::waitKey(1000);
    while (1)
    {
	    
	    cam.transColorImg().copyTo(frame);       
        // cv::imshow("windows", frame);
        cv:: resize(frame,frame,cv::Size(0.5*frame.cols,0.5*frame.rows));
        cv::imshow("test",frame);
        char ch =cv::waitKey(1);
        if(ch == 'c')
        {
            char buf[7];
            sprintf(buf,"%06d",pic_num);
            std::string name;
            name = buf;
	    std::string time = getTime();
            std::string path = "/home/domistic/Vision/data/picture2019/"+name+"_"+time+".jpg";
            cv::imwrite(path,frame);
            pic_num++;
            std::cout<<"picture have "<<pic_num<<"@"<<time<<std::endl;
        }
        if(ch == 'q')
	    break;    
    }

    cam.closeCamera();
    return 0;

         

}
