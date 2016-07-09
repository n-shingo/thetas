#ifndef __SSM_IMAGE_HD720P_HPP__
#define __SSM_IMAGE_HD720P_HPP__

#include <opencv2/opencv.hpp>

////////////////////////////
// Theta S Live View (USB)

// マクロ変数
#define SNAME_HD720P "imgC3_HD720P"
#define HD720P_HEIGHT 720
#define HD720P_WIDTH 1280

/* HD(720p) 用画像 */
typedef struct
{
	unsigned char image[ HD720P_HEIGHT * HD720P_WIDTH * 3];

}ImageC3_HD720p;

// プロトタイプ宣言
void Mat_to_ImageC3_HD720p( cv::Mat src, ImageC3_HD720p *dst );
void ImageC3_HD720p_to_Mat( ImageC3_HD720p &src, cv::Mat *dst );

// 関数定義
void Mat_to_ImageC3_HD720p( cv::Mat src, ImageC3_HD720p *dst )
{
	memcpy( dst->image, src.data, sizeof(char)*HD720P_WIDTH*HD720P_HEIGHT*3);
}

void ImageC3_HD720p_to_Mat( ImageC3_HD720p &src, cv::Mat *dst )
{
	cv::Mat t3( cv::Size(HD720P_WIDTH, HD720P_HEIGHT), CV_8UC3, src.image);
	*dst = t3.clone();
}


#endif // __SSM_IMAGE_HD720P_HPP__
