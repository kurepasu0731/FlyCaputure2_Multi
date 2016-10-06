#include <FlyCapture2.h>
#include <opencv2\opencv.hpp>
#include <iostream>
#include <cstdio>

#include "PGROpenCV.h"

//// OpenCV�̃o�[�W�������擾����pragma���Ń��C�u������ǂݍ���
//#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
//
//#ifdef _DEBUG
//#define CV_EXT_STR "d.lib"
//#else _REREASE
//#define CV_EXT_STR ".lib"
//#endif
//
//#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
//#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
//#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)
//#pragma comment(lib,"opencv_video" CV_VERSION_STR CV_EXT_STR)
//#pragma comment(lib,"opencv_features2d" CV_VERSION_STR CV_EXT_STR)
//
//#pragma comment(lib,"FlyCapture2.lib")
//#pragma comment(lib,"FlyCapture2GUI.lib")


std::vector<cv::Point2f> corners;

//�R�[�i�[�_�����o(src�̓��m�N��)
cv::Mat detectCorner(const cv::Mat &src)
{
	//�R�[�i�[���o
	cv::goodFeaturesToTrack(src, corners, 100, 0.01, 50);
	//�����x��
	cv::cornerSubPix(src, corners, cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));



	//�`�悵�����̂�Ԃ�
	cv::Mat drawimage = cv::Mat(src.rows, src.cols, CV_8UC3);
	//memcpy(drawimage.data, src.data, drawimage.rows * drawimage.cols * drawimage.channels());
	cv::cvtColor(src, drawimage, CV_GRAY2BGR);

	//�`��
	for(int i = 0; i < corners.size(); i++)
	{
		cv::circle(drawimage, corners[i], 1, cv::Scalar(0, 0, 255), 3);
	}
	return drawimage;

}

int main( int argc, char* argv[] )
{
	// �N���������J�����C���f�b�N�X���w��
	TPGROpenCV	pgrOpenCV(0);
	cv::Mat cap;

	// initialization
	pgrOpenCV.init(FlyCapture2::PIXEL_FORMAT_MONO8, FlyCapture2::NEAREST_NEIGHBOR);
	// start capturing
	pgrOpenCV.start();
	for( ;; ) {
		pgrOpenCV.tm.restart();

		pgrOpenCV.queryFrame();

		//�R�[�i�[�_�����o
		pgrOpenCV.showCapImg(detectCorner(pgrOpenCV.getVideo()));
		//pgrOpenCV.showCapImg(pgrOpenCV.getVideo());
		
		//pgrOpenCV.CameraCapture(cap);
		//pgrOpenCV.showCapImg(cap);

		if( cv::waitKey( 10 ) == ' ' ) {
			break;
		}

		pgrOpenCV.tm.elapsed();
	}

	// stop capturing
	pgrOpenCV.stop();

	// finalization
	pgrOpenCV.release();

	return 0;
}