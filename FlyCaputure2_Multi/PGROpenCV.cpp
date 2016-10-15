#include "PGROpenCV.h"
#include <Windows.h>

// Constructor
TPGROpenCV::TPGROpenCV(int _useCameraIndex)
{
	windowNameCamera = "camera";
	cv::namedWindow(windowNameCamera.c_str(), cv::WINDOW_NORMAL);

	useCamIndex = _useCameraIndex;
}

// Destructor
TPGROpenCV::~TPGROpenCV()
{

}

// initialize PGR
int TPGROpenCV::init(FlyCapture2::PixelFormat _format, int ColorProcessingAlgorithm)
{
	// get number of cameras on the bus
	fc2Error = fc2BusMgr.GetNumOfCameras(&numCameras);
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}
	else {
		std::cout << "Number of cameras detected: " << numCameras << std::endl;
	}

	// get guid from index
	fc2Error = fc2BusMgr.GetCameraFromIndex(useCamIndex, &fc2Guid);
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}

	// connect to the camera using guid
	fc2Error = fc2Cam.Connect(&fc2Guid);
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}

	// get the camera information
	fc2Error = fc2Cam.GetCameraInfo(&fc2CamInfo);
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}
	else {
		//-カメラの製造情報を表示
		//PrintCameraInfo( &fc2CamInfo );
	}

	// set the pixel format
	fc2PixelFormat = _format;

	//set the color processing algolizm
	fc2CPA = (FlyCapture2::ColorProcessingAlgorithm)ColorProcessingAlgorithm;

	//initialize camera parameter
	InitCameraParameter();

	fc2Cam.GetProperty(&fc2Prop);

	return 0;
}

//initialize camera parameter
void TPGROpenCV::InitCameraParameter()
{
	loadParameters();
	setShutterSpeed(ShutterSpeed);
	setGain(Gain);
	setWhiteBalance(Wb_Red, Wb_Blue);
	setFrameRate(Framerate);
}


void TPGROpenCV::PrintBuildInfo()
{
	FlyCapture2::FC2Version fc2Version;
	FlyCapture2::Utilities::GetLibraryVersion(&fc2Version);

	std::cout << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build << std::endl;
	std::cout << "Application build date: " << __DATE__ << " " << __TIME__ << std::endl << std::endl;
}

void TPGROpenCV::PrintError(FlyCapture2::Error error)
{
	error.PrintErrorTrace();
}

void TPGROpenCV::PrintCameraInfo(FlyCapture2::CameraInfo* pCamInfo)
{
	std::cout << std::endl << "\n*** CAMERA INFORMATION ***" << std::endl
		<< "Serial number - " << pCamInfo->serialNumber << std::endl
		<< "Camera model - " << pCamInfo->modelName << std::endl
		<< "Camera vendor - " << pCamInfo->vendorName << std::endl
		<< "Sensor - " << pCamInfo->sensorInfo << std::endl
		<< "Resolution - " << pCamInfo->sensorResolution << std::endl
		<< "Firmware version - " << pCamInfo->firmwareVersion << std::endl
		<< "Firmware build time - " << pCamInfo->firmwareBuildTime << std::endl << std::endl;
}

// reply present pixel format in OpenCV style
int TPGROpenCV::PixelFormatInOpenCV()
{
	switch (fc2PixelFormat) {
	case FlyCapture2::PIXEL_FORMAT_BGR:
	case FlyCapture2::PIXEL_FORMAT_BGRU:
	case FlyCapture2::PIXEL_FORMAT_RGB:
	case FlyCapture2::PIXEL_FORMAT_RGBU:
		return CV_8UC3;
		break;
	case FlyCapture2::PIXEL_FORMAT_S_RGB16:
		return CV_16SC3;
		break;
	case FlyCapture2::PIXEL_FORMAT_BGR16:
	case FlyCapture2::PIXEL_FORMAT_BGRU16:
	case FlyCapture2::PIXEL_FORMAT_RGB16:
		return CV_16UC3;
		break;
	case FlyCapture2::PIXEL_FORMAT_MONO8:
	case FlyCapture2::PIXEL_FORMAT_RAW8:
		return CV_8UC1;
		break;
	case FlyCapture2::PIXEL_FORMAT_MONO16:
	case FlyCapture2::PIXEL_FORMAT_RAW16:
		return CV_16UC1;
		break;
	case FlyCapture2::PIXEL_FORMAT_S_MONO16:
		return CV_16SC1;
		break;
	default:
		return CV_8UC3;
		break;
	}
}

int TPGROpenCV::start()
{
	fc2Error = fc2Cam.StartCapture();
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}
	else {
		// retrieve an image to allocate fc2Mat
		FlyCapture2::Image	wk;
		fc2Error = fc2Cam.RetrieveBuffer(&wk);
		if (fc2Error != FlyCapture2::PGRERROR_OK) {
			PrintError(fc2Error);
			return -1;
		}
		else {
			fc2Mat.create(wk.GetRows(), wk.GetCols(), PixelFormatInOpenCV());
		}
		return 0;
	}
	return -1;
}

int TPGROpenCV::queryFrame()
{
	// retrieve a frame image (0~1ms)
	FlyCapture2::Image	rawImage;
	rawImage.SetDefaultColorProcessing(fc2CPA);
	fc2Error = fc2Cam.RetrieveBuffer(&rawImage);
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}
	// convert the raw image (12ms)
	FlyCapture2::Image	cvtImage;
	//tm.restart();
	fc2Error = rawImage.Convert(fc2PixelFormat, &cvtImage);
	//tm.elapsed();
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}
	// copy (0~1ms)
	memcpy(fc2Mat.data, cvtImage.GetData(), cvtImage.GetDataSize());

	return 0;
}

int TPGROpenCV::stop()
{
	fc2Error = fc2Cam.StopCapture();
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}
	else {
		return 0;
	}
}

int TPGROpenCV::release()
{
	fc2Mat.release();

	fc2Error = fc2Cam.Disconnect();
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
		return -1;
	}
	else {
		return 0;
	}
}

// SET camera parameter
void TPGROpenCV::setShutterSpeed(float shutterSpeed)
{
	fc2Prop.type = FlyCapture2::SHUTTER;
	fc2Prop.autoManualMode = false;
	fc2Prop.absControl = true;
	fc2Prop.absValue = shutterSpeed;
	fc2Cam.SetProperty(&fc2Prop);
}
void TPGROpenCV::setGain(float gain)
{
	fc2Prop.type = FlyCapture2::GAIN;
	fc2Prop.autoManualMode = false;
	fc2Prop.absControl = true;
	fc2Prop.absValue = gain;
	fc2Cam.SetProperty(&fc2Prop);
}
void TPGROpenCV::setWhiteBalance(int r, int b)
{
	fc2Cam.GetProperty(&fc2Prop);
	fc2Prop.type = FlyCapture2::WHITE_BALANCE;
	fc2Prop.autoManualMode = false;
	fc2Prop.absControl = false;
	//fc2Prop.onOff = true;
	fc2Prop.valueA = r;
	fc2Prop.valueB = b;
	fc2Cam.SetProperty(&fc2Prop);
}
void TPGROpenCV::setPixelFormat(FlyCapture2::PixelFormat format)
{
	// set the pixel format
	fc2PixelFormat = format;
}
void TPGROpenCV::setColorProcessingAlgorithm(FlyCapture2::ColorProcessingAlgorithm algorithm)
{
	//set the color processing algolizm
	fc2CPA = (FlyCapture2::ColorProcessingAlgorithm)algorithm;

}
/* .ini からパラメータの取得 */
void TPGROpenCV::loadParameters()
{
	char buf[128];

	//-シャッタースピード
	GetPrivateProfileStringA("shutterspeed", "shutter_speed", NULL, buf, sizeof(buf), PGR_PARAMETER_FILE);
	ShutterSpeed = atof(buf);

	//-ゲイン
	GetPrivateProfileStringA("gain", "gain", NULL, buf, sizeof(buf), PGR_PARAMETER_FILE);
	Gain = atof(buf);

	//-ホワイトバランス
	GetPrivateProfileStringA("whitebalance", "red", NULL, buf, sizeof(buf), PGR_PARAMETER_FILE);
	Wb_Red = (unsigned int)atof(buf);
	GetPrivateProfileStringA("whitebalance", "blue", NULL, buf, sizeof(buf), PGR_PARAMETER_FILE);
	Wb_Blue = (unsigned int)atof(buf);

	//-フレームレート
	GetPrivateProfileStringA("framerate", "framerate", NULL, buf, sizeof(buf), PGR_PARAMETER_FILE);
	Framerate = atof(buf);

	//-ガンマ値
	Gamma = 1.0;

	//-ブライトネス
	Brightness = 0.0;

}

// GET camera parameter

float TPGROpenCV::getShutterSpeed()
{
	fc2Prop.type = FlyCapture2::SHUTTER;
	fc2Prop.absControl = true;
	return fc2Prop.absValue;
}
float TPGROpenCV::getGain()
{
	fc2Prop.type = FlyCapture2::GAIN;
	fc2Prop.absControl = true;
	return fc2Prop.absValue;
}
void TPGROpenCV::getWhiteBalance(int &r, int &b)
{
	fc2Cam.GetProperty(&fc2Prop);
	fc2Prop.type = FlyCapture2::WHITE_BALANCE;
	fc2Prop.autoManualMode = false;
	fc2Prop.absControl = false;
	//fc2Prop.onOff = true;
	fc2Prop.valueA = r;
	fc2Prop.valueB = b;
	fc2Cam.SetProperty(&fc2Prop);
}
void TPGROpenCV::setGamma(float gamma)
{
	fc2Prop.type = FlyCapture2::GAMMA;
	fc2Prop.absControl = true;
	fc2Prop.absValue = gamma;
	fc2Cam.SetProperty(&fc2Prop);
}
void TPGROpenCV::setBrightness(float brightness)
{
	fc2Prop.type = FlyCapture2::BRIGHTNESS;
	fc2Prop.absControl = true;
	fc2Prop.absValue = brightness;
	fc2Cam.SetProperty(&fc2Prop);
}
void TPGROpenCV::setFrameRate(float framerate)
{
	fc2Prop.type = FlyCapture2::FRAME_RATE;
	fc2Prop.absControl = true;
	fc2Prop.absValue = framerate;
	fc2Cam.SetProperty(&fc2Prop);
}

//
void TPGROpenCV::showCapImg(cv::Mat cap)
{
	//cv::namedWindow(windowNameCamera.c_str(), cv::WINDOW_NORMAL);
	//引き数に何も指定しなかった場合はここで撮影画像を取得
	//if (cap.data == NULL)
	//	cap = getVideo();
	//cv::resize(cap, cap, cv::Size(), 0.8, 0.8);
	cv::imshow(windowNameCamera, cap);
	cv::waitKey(1);

}


// 渡したMat画像に直接書き込む関数
void TPGROpenCV::CameraCapture(cv::Mat &image)
{
	// retrieve a frame image
	FlyCapture2::Image	rawImage;
	rawImage.SetDefaultColorProcessing(fc2CPA);
	fc2Error = fc2Cam.RetrieveBuffer(&rawImage);
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
	}
	// convert the raw image
	FlyCapture2::Image	cvtImage;
	fc2Error = rawImage.Convert(fc2PixelFormat, &cvtImage);
	if (fc2Error != FlyCapture2::PGRERROR_OK) {
		PrintError(fc2Error);
	}

	// rgbImageをMat型に変換
	image = cv::Mat(cvtImage.GetRows(), cvtImage.GetCols(), CV_8U);
	memcpy(image.data, cvtImage.GetData(), cvtImage.GetDataSize());
}
