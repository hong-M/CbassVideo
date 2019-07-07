
#include "CameraApi.h"
#include <opencv2/opencv.hpp>

using namespace cv;

unsigned char * g_pRgbBuffer; //处理后数据缓存区

int
main()
{

	int iCameraCounts = 1;
	int iStatus = -1;
	tSdkCameraDevInfo tCameraEnumList;
	int hCamera;
	tSdkCameraCapbility tCapability; //设备描述信息
	tSdkFrameHead sFrameInfo;
	BYTE* pbyBuffer;
	int iDisplayFrames = 100; //10000
	IplImage *iplImage = NULL;
	int channel = 3;

	CameraSdkInit(1);

	//枚举设备，并建立设备列表
	iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
	printf("state = %d\n", iStatus);

	printf("count = %d\n", iCameraCounts);
	//没有连接设备
	if (iCameraCounts == 0) {
		return -1;
	}

	//相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
	iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

	//初始化失败
	printf("state = %d\n", iStatus);
	if (iStatus != CAMERA_STATUS_SUCCESS) {
		return -1;
	}

	//获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
	CameraGetCapability(hCamera, &tCapability);

	//
	g_pRgbBuffer = (unsigned char*) malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
	//g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

	/*让SDK进入工作模式，开始接收来自相机发送的图像
	数据。如果当前相机是触发模式，则需要接收到
	触发帧以后才会更新图像。    */
	CameraPlay(hCamera);

	/*其他的相机参数设置
	例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
	     CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
	     CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
	     更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
	 */



	//cv::VideoWriter vout("./ttt.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 25e0, cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight));

	//int codec = cv::VideoWriter::fourcc('D','I','V','X');
	//int codec = CV_FOURCC('D','I','V','X');
	//cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
	int codec = CV_FOURCC('M','J','P','G');       //这个是旧版本，后续opencv版本升级之后，弃用这个接口，这个接口是之前的C接口。C与C++不要混着用。 
	//int codec = cv::VideoWriter::fourcc('M','J','P','G');
	cv::VideoWriter vout;

	//循环显示1000帧图像

	while (iDisplayFrames--) {

		if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {

			CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);



			cv::Mat matImage(
					cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer
					);

			std::cout<<matImage.size.operator()()<<std::endl;
                        //cv::resize(matImage,matImage, cv::Size(800,600));
			//cv::resize(matImage,matImage, cv::Size(2592,1944));
			//vout << matImage;

			if (!vout.isOpened()) {
				vout.open("./test.avi", codec, 25, matImage.size.operator ()());	
				printf("isOpened----iDisplayFrames=%d\r\n",iDisplayFrames);			
			}
			vout << matImage;
			printf("while----iDisplayFrames=%d\r\n",iDisplayFrames);		


			cv::imshow("Opencv Demo", matImage);

			cv::waitKey(5);

			//在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera, pbyBuffer);

		}
	}

	vout.release();










#if 0
	//cv::VideoWriter vout("./ttt.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 25e0, cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight));
	//cv::VideoWriter vout("./ttt.mp4", CV_FOURCC('M', 'P', '4', 'V'), 25e0, cv::Size(800, 600));  //无视频输出
	//cv::VideoWriter vout("./ttt.avi", CV_FOURCC('M', 'P', '4', 'V'), 25e0, cv::Size(800, 600));    //视频输出无发打开视频
	//cv::VideoWriter vout("./ttt.avi", CV_FOURCC('P','I','M','1'), 25e0, cv::Size(800, 600));     //视频输出无发打开视频
	cv::VideoWriter vout("./ttt.avi", CV_FOURCC('M', 'P', '4', '2'), 8, cv::Size(800, 600));


	//循环显示1000帧图像
	while (iDisplayFrames--) {
		if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
			CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

			cv::Mat matImage(
					cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					g_pRgbBuffer
					);

			vout << matImage;

			cv::imshow("Opencv Demo", matImage);

			cv::waitKey(5);

			//在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera, pbyBuffer);

		}
	}

	vout.release();
#endif


	CameraUnInit(hCamera);
	//注意，现反初始化后再free
	free(g_pRgbBuffer);


	return 0;
}

