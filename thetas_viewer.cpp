//
//  thetaS_viewer.cpp
//  ThetaS_TEST
//
//  Created by shingo on 2016/07/07.
//  Copyright (c) 2016年 shingo. All rights reserved.
//

//  透視射影で Theta S を見る

#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "opencv2/opencv.hpp"


#define HEIGHT 720
#define WIDTH 1280

using namespace std;
using namespace cv;

void undistortLeftWithFixedParams( Mat img, Mat &dst, Matx33d prjMat, Size size );

int main(int argc, char ** argv) {

    // カメラ id
    int cam_id = 1;
    
	int c;
	while( (c = getopt( argc, argv, "i:" )) != -1  ){
		switch( c ){
			case 'i':
				cam_id = atoi(optarg);
				break;
		}
	}
	cout << "cam id:" << cam_id << endl;

    //////////////////////
    //  出力画像サイズ
    int image_w = 640;
    int image_h = image_w * 3/4;
    
    //////////////////////////////////////////////////////////////
    // ビデオのオープン
    cv::VideoCapture cap( cam_id );
    if( !cap.isOpened()){
        printf( "Failed to open video!\n" );
        return 1;
    }
	cap.set( CV_CAP_PROP_FRAME_WIDTH, WIDTH );
	cap.set( CV_CAP_PROP_FRAME_HEIGHT, HEIGHT );
    int org_w = cap.get( CV_CAP_PROP_FRAME_WIDTH  );
    int org_h = cap.get( CV_CAP_PROP_FRAME_HEIGHT );
    printf( "%d,%d\n", org_w, org_h);
    
    //////////////////////////////////////////////////////////////
    // 動画再生
    char winname_cal[] ="Calibrated Video";
    namedWindow(winname_cal);
    
    int a_deg = 0;
    int b_deg = 0;
    
    while(1)
    {
        int key = waitKey(1);
        
        // カメラ操作
        if( key == 65362 ) // Up
            b_deg = MIN( 90, b_deg+4 );

        else if( key == 65364 ) // Down
            b_deg = MAX( -90, b_deg-4);

        else if( key == 65361 ){ // Left
            a_deg += 4;
            if(a_deg>360) a_deg -= 360;
        }
        if( key == 65363 ){ // right
            a_deg -= 4;
            if(a_deg<0) a_deg += 360;
        }
        
        // key == Esc, 'q'
        if( key == 27 || key == 'q') break;
        
        // 原画像取得
        Mat frame;
        cap >> frame;
        
        // キャリブレーション適用
        Mat calibImg;
        
        double c_a = cos(a_deg*M_PI/180.0);
        double s_a = sin(a_deg*M_PI/180.0);
        double c_b = cos(b_deg*M_PI/180.0);
        double s_b = sin(b_deg*M_PI/180.0);
        double shift_u = image_w / 2;
        double shift_v = image_h / 2;
        double scale = 300;
        cv::Mat T = (Mat_<double>(3,3) << 1.0, 0.0, shift_u, 0.0, 1.0, shift_v, 0.0, 0.0, 1.0);
        cv::Mat S = (Mat_<double>(3,3) << scale, 0.0, 0.0, 0.0, scale, 0.0, 0.0, 0.0, 1.0);
        cv::Mat A = (Mat_<double>(3,3) << c_a, 0.0, s_a, 0.0, 1.0, 0.0, -s_a, 0.0, c_a);
        cv::Mat B = (Mat_<double>(3,3) << 1.0, 0.0, 0.0, 0.0, c_b, s_b, 0.0, -s_b, c_b);
        cv::Mat projMat = T*S*B*A;
        
        undistortLeftWithFixedParams( frame, calibImg, projMat, Size(image_w,image_h) );
        
        
        // 画像表示
        imshow(winname_cal, calibImg);
    }
    destroyWindow(winname_cal);
    
    return 0;
}


// 決め打ちのカメラ内部パラメータで変換する
void undistortLeftWithFixedParams( Mat img, Mat &dst, Matx33d prjMat, Size size )
{
    // カメラ内部パラメータ
    double s = 181.0;
    cv::Matx33d K1(s, 0., 320, 0., s, 320., 0., 0., 1.);
    cv::Matx33d K2(s, 0., 958, 0., -s, 320., 0., 0., 1.);
    cv::Vec4d   D(0., 0., 0., 0.);
    Vec2d f1(K1(0,0), K1(1,1));
    Vec2d c1(K1(0,2), K1(1,2));
    Vec2d f2(K2(0,0), K2(1,1));
    Vec2d c2(K2(0,2), K2(1,2));
    
    // 透視射影画像から世界座標に変換する行列を求める
    Matx33d iR = prjMat.inv(DECOMP_SVD);
    
    // rempa用のマップ準備
    Mat map1(size, CV_16SC2), map2(size, CV_16SC1);
    
    // (i,j)に対する(u,v)の座標の計算
    for( int i=0; i<size.height; i++ )
    {
        float* m1f = map1.ptr<float>(i);
        float* m2f = map2.ptr<float>(i);
        short* m1 = (short*)m1f;
        ushort *m2 = (ushort*)m2f;
        
        double _x = i*iR(0,1) + iR(0,2);
        double _y = i*iR(1,1) + iR(1,2);
        double _w = i*iR(2,1) + iR(2,2);
        
        for( int j=0; j<size.width; j++ )
        {
            double x = _x/_w, y = _y/_w;
            double r = sqrt(x*x + y*y);
            double theta = atan(r);
            
            // 歪み計算
            //double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
            //double theta_d = theta * (1 + D[0]*theta2 + D[1]*theta4 + D[2]*theta6 + D[3]*theta8);
            double theta_d = theta;
            
            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u, v;
            if(_w>0.0){
                u = f1[0]*x*scale + c1[0];
                v = f1[1]*y*scale + c1[1];
            }
            else{
                u = f2[0]*x*scale + c2[0];
                v = f2[1]*y*scale + c2[1];
            }
            
            // map 作成
            int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
            int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
            m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
            m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
            m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            
            
            // 次の j のための準備
            _x += iR(0,0);
            _y += iR(1,0);
            _w += iR(2,0);
        }
        
    }
    
    // 変換実行
    remap(img, dst, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
    
}
