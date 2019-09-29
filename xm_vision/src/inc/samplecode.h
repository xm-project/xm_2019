#include "arcsoft_face_sdk.h"
#include "amcomdef.h"
#include "asvloffscreen.h"
#include "merror.h"
#include <iostream>  
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#define APPID "AQwC9ptTpce5rnPgWU6pM8voRPTD7GRpiUQEhPiridLG"
#define SDKKey "BpbT3xdiY19Y1aLLcRSY5EeVzerCaT53Dt4ZroCR1iqX"

#define NSCALE 32				//最小人脸相对于图片宽度的比例，推荐16，图片范围2-32
#define FACENUM	10			//最多的人脸数

#define SafeFree(p) { if ((p)) free(p); (p) = NULL; }
#define SafeArrayDelete(p) { if ((p)) delete [] (p); (p) = NULL; } 
#define SafeDelete(p) { if ((p)) delete (p); (p) = NULL; } 

void gender_draw(cv::Mat src_jpg)
{
	
	//激活SDK

	MRESULT res = ASFActivation(APPID, SDKKey);

	if (MOK != res && MERR_ASF_ALREADY_ACTIVATED != res)
		std::cout<<"ALActivation fail:"<<res<<std::endl;
	else
		std::cout<<"ALActivation success:"<<res<<std::endl;


	//初始化引擎
	MHandle handle = NULL;
	MInt32 mask = ASF_FACE_DETECT | ASF_GENDER;			//功能选择
	res = ASFInitEngine(ASF_DETECT_MODE_IMAGE, ASF_OP_0_ONLY, NSCALE, FACENUM, mask, &handle);
	if (res != MOK)
		std::cout<<"ALInitEngine fail:"<<res<<std::endl;
	else
		std::cout<<"ALInitEngine sucess:"<<res<<std::endl;
	
	cv::Mat src_bmp = cv::imread("/home/domistic/Vision/data/gender_and_pose/output.bmp");		//用于读取
	if(src_bmp.empty()){
        	std::cout<<"can not load image!"<<std::endl;
    	}
	else
	{	
		std::cout<<"load image successfully!"<<std::endl;	
		int Width = src_jpg.cols;
		int Height = src_jpg.rows;
		IplImage* src__bmp = cvLoadImage("/home/domistic/Vision/data/gender_and_pose/output.bmp");
		MUInt8* imageData = (MUInt8*)src__bmp->imageData;

		// 人脸检测
		ASF_MultiFaceInfo detectedFaces = { 0 };
		res = ASFDetectFaces(handle, Width, Height, ASVL_PAF_RGB24_B8G8R8, imageData, &detectedFaces);
		if (res != MOK)
			std::cout<<" ASFDetectFaces fail: "<<res<<std::endl;
		else
		{
			std::cout<<" ASFDetectFaces sucess: "<<res<<std::endl;
		}
		int face_num = detectedFaces.faceNum;

		// 人脸信息检测
		MInt32 lastMask = ASF_GENDER;
		res = ASFProcess(handle, Width, Height, ASVL_PAF_RGB24_B8G8R8, imageData, &detectedFaces, lastMask);
		if (res != MOK)
			std::cout<<"ASFProcess fail:"<<res<<std::endl;
		else
			std::cout<<"ASFProcess sucess:"<<res<<std::endl;

		// 获取性别
		ASF_GenderInfo genderInfo = { 0 };
		res = ASFGetGender(handle, &genderInfo);
		if (res != MOK)
			std::cout<<" ASFGetGender fail: "<<res<<std::endl;
		else
			std::cout<<" ASFGetGender sucess: "<<res<<std::endl;
		int male_num = 0;
		int female_num = 0;
		for(int i=0;i<face_num;i++){
			cv::Rect rect(detectedFaces.faceRect[i].left, 
					  detectedFaces.faceRect[i].top, 
					  detectedFaces.faceRect[i].right - detectedFaces.faceRect[i].left, 
					  detectedFaces.faceRect[i].bottom - detectedFaces.faceRect[i].top);
			if(genderInfo.genderArray[i] == 0){
				male_num++;
				cv::rectangle(src_jpg, rect, cv::Scalar(255, 255, 0), 2, 8);
				cv::putText(src_jpg, "male", rect.tl(), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0), 1);
			}
			else if(genderInfo.genderArray[i] == 1){
				female_num++;
				cv::rectangle(src_jpg, rect, cv::Scalar(0, 0, 255), 2, 8);
				cv::putText(src_jpg, "female", rect.tl(), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
			}
		}
		
		//结果输出
		if(face_num != male_num + female_num){
			std::cout<<"there are faces that too small"<<std::endl;
			face_num = male_num + female_num;
		}
		std::cout<<std::endl;
		std::cout<<"**********result**********"<<std::endl;
		std::cout<<"there number of people is "<<face_num<<std::endl;
		std::cout<<"there number of males is "<<male_num<<std::endl;
		std::cout<<"there number of females is "<<female_num<<std::endl;
		std::cout<<std::endl;

		//结果保存txt里从上至下依次为总和、男性数量、女性数量
		std::ofstream fout("/home/domistic/Vision/data/gender_and_pose/result.txt");
		fout<<face_num<<std::endl;
		fout<<male_num<<std::endl;
		fout<<female_num<<std::endl;

		SafeArrayDelete(imageData);
		
		//获取版本信息
		const ASF_VERSION* pVersionInfo = ASFGetVersion(handle);

		//销毁引擎
		res = ASFUninitEngine(handle);
		if (res != MOK)
			std::cout<<"ALUninitEngine fail:"<<res<<std::endl;
		else
			std::cout<<"ALUninitEngine success:"<<res<<std::endl;
	}
	cv::imwrite("/home/domistic/Vision/data/gender_and_pose/gender_result.jpg", src_jpg);

}
