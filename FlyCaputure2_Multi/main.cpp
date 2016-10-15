#include <FlyCapture2.h>
#include <opencv2\opencv.hpp>
#include <iostream>
#include <cstdio>

#include "PGROpenCV.h"

#define CAMERA_WIDTH 2448
#define CAMERA_HEIGHT 2048

#define DOT_SIZE 150
#define A_THRESH_VAL -5
#define DOT_THRESH_VAL_MIN 100  // �h�b�g�m�C�Y�e��
#define DOT_THRESH_VAL_MAX 500 // �G�b�W�m�C�Y�e��

void calCoG_dot_v0(cv::Mat &src, cv::Point& sum, int& cnt, cv::Point& min, cv::Point& max, cv::Point p) 
{
	if (src.at<uchar>(p)) {
		sum += p; cnt++;
		src.at<uchar>(p) = 0;
		if (p.x<min.x) min.x = p.x;
		if (p.x>max.x) max.x = p.x;
		if (p.y<min.y) min.y = p.y;
		if (p.y>max.y) max.y = p.y;

		if (p.x - 1 >= 0) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x-1, p.y));
		if (p.x + 1 < CAMERA_WIDTH) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x + 1, p.y));
		if (p.y - 1 >= 0) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x, p.y - 1));
		if (p.y + 1 < CAMERA_HEIGHT) calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(p.x, p.y + 1));
	}
}

bool init_v0(cv::Mat &src) 
{
	cv::Mat origSrc = src.clone();
	//�K���I臒l����
	cv::adaptiveThreshold(src, src, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 7, A_THRESH_VAL);
	//���ʂ̓�l��
	//cv::threshold(src, src, 150, 255, cv::THRESH_BINARY);
	cv::Mat ptsImg = cv::Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
	cv::cvtColor(src, ptsImg, CV_GRAY2BGR);

	cv::Point sum, min, max, p;
	int cnt;
	std::vector<cv::Point> dots;
	for (int i = 0; i < CAMERA_HEIGHT; i++) {
		for (int j = 0; j < CAMERA_WIDTH; j++) {
			if (src.at<uchar>(i, j)) {
				sum = cv::Point(0, 0); cnt = 0; min = cv::Point(j, i); max = cv::Point(j, i);
				calCoG_dot_v0(src, sum, cnt, min, max, cv::Point(j, i));
				if (cnt>DOT_THRESH_VAL_MIN && max.x - min.x < DOT_THRESH_VAL_MAX && max.y - min.y < DOT_THRESH_VAL_MAX) {
					dots.push_back(cv::Point(sum.x / cnt, sum.y / cnt));
				}
			}
		}
	}


	//cv::rectangle(ptsImg, cv::Point(CamWidth / 4, CamHeight / 4), cv::Point(CamWidth * 3 / 4, CamHeight * 3 / 4), cv::Scalar(255, 0, 0), 5, 4);
	// OpenGL�p�ɗ\��RGB�p�̃f�[�^�쐬
	//cv::rectangle(ptsImg, cv::Point(CamWidth / 4, CamHeight / 4), cv::Point(CamWidth * 3 / 4, CamHeight * 3 / 4), cv::Scalar(0, 0, 255), 5, 4);
	std::vector<cv::Point>::iterator it = dots.begin();
	
	bool k = (dots.size()==DOT_SIZE);
	//for (int i=0; it != dots.end(); i++,++it) {
	//	if (i && i%MarkersWidth == 0) {
	//		if ((*it).y <= (*(it - 1)).y) k = false;
	//	} else {
	//		if (i && (*it).x <= (*(it - 1)).x) k = false;
	//	}
	//}
	//cv::Scalar co = k ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
	// OpenGL�p�ɗ\��RGB�p�̃f�[�^�쐬
	cv::Scalar color = k ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
	for (it = dots.begin(); it != dots.end(); ++it) {
		cv::circle(ptsImg, *it, 3, color, 2);
	}

	//if (k) {
	//	it = dots.begin();
	//	for (int i = 0; it != dots.end(); ++it) {
	//		marker_u[i / MarkersWidth][i%MarkersWidth] = *it;
	//		marker_s[i / MarkersWidth][i%MarkersWidth] = true;
	//		i++;
	//	}
	//	
	//	flag = 1;
	//	std::cout << "init complete!" << std::endl;
	//	
	//}

	cv::Mat resize_src, resize_pts;
	cv::resize(origSrc, resize_src, cv::Size(), 0.5, 0.5);
	cv::resize(ptsImg, resize_pts, cv::Size(), 0.5, 0.5);

	cv::imshow("src", resize_src);
	cv::imshow("result", resize_pts);

	return k;
}


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
	cv::cornerSubPix(src, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));



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

//�K���I臒l�̃e�X�g(src�̓��m�N��)
//���ʂ̓�l���Ɣ�r
void adaptiveThresholdTest(const cv::Mat &src)
{
		//�K���I臒l�����̊m�F//
		cv::Mat adapBinImg, binImg;
		cv::adaptiveThreshold(src, adapBinImg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 7, A_THRESH_VAL);
		cv::threshold(src, binImg, 150, 255, cv::THRESH_BINARY);

		cv::Mat resizeAdap, resizeBin;
		cv::resize(adapBinImg, resizeAdap, cv::Size(adapBinImg.cols * 0.5, adapBinImg.rows * 0.5));
		cv::resize(binImg, resizeBin, cv::Size(binImg.cols * 0.5, binImg.rows * 0.5));

		cv::imshow("Threshold", resizeBin);
		cv::imshow("adaptiveThreshold", resizeAdap);
		//�K���I臒l�����̊m�F//
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

		//�K���I臒l�����ƕ��ʂ̓�l���̔�r
		//adaptiveThresholdTest(pgrOpenCV.getVideo());

		//�h�b�g���o
		//init_v0(pgrOpenCV.getVideo());

		//�m�[�}��
		//pgrOpenCV.showCapImg(pgrOpenCV.getVideo());
		
		//���܂������Ȃ��H
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