//-------------------------------------------------
//camera-viewer.cpp
//YSD
//since: 2014-07-31
//-------------------------------------------------

#include<cstdio>
#include <iostream>
#include <signal.h>
#include "opencv2/opencv.hpp"
#include <ssm.hpp>
#include "SSM-Image-HD720p.hpp"

#define WIDTH 1024
#define HEIGHT 768
#define DELTA_DEG 4  // 矢印キーによる移動量[deg]

using namespace std;
using namespace cv;

bool gShutOff;

void ctrlC(int aStatus)
{
    signal(SIGINT, NULL);
    gShutOff = true;
}

// Ctrl-C による正常終了を設定
inline void setSigInt(){ signal(SIGINT, ctrlC); }

// 決め打ちのカメラ内部パラメータでTHETASの画像を透視射影に変換する
void makePerspectiveViewWithFixedParams( Mat img, Mat &dst, Matx33d prjMat, Size size);
void showHelp(void);

int main(int argc, char ** argv)
{
    //==========================================================
    // ---> DECLARATION
    //==========================================================
    int ssm_id = 0;
	int whole_angle = 0;
	bool show_srcimg = false;

	Mat frm;

    // <--- DECLARATION

    //==========================================================
    // ---> INITALIZE
    //==========================================================
    //--------------------------------------
    // オプション解析
    int c;
    while( (c = getopt(argc, argv, "hsi:r:")) != -1 )
    {
        switch ( c )
        {
        case 'i':
            fprintf( stderr, "input ssm id = %d\n", atoi(optarg) );
            ssm_id = atoi(optarg);
            break;
		case 'r':
			whole_angle = atoi(optarg) % 4;
			break;		
		case 's':
			show_srcimg = true;
			break;
        case 'h':
		default:
			showHelp();
			return 0;
        }
    }

	if(!initSSM()){
		cerr << "SSM Error : initSSM()" << endl;
		return 0;
	}

    SSMApi<ImageC3_HD720p> cam_image(SNAME_HD720P, ssm_id);

	if( !cam_image.open( ) ){
		cerr << "SSM Error : create()" << endl;
		return 1;
	}

	setSigInt();
    // <--- INITALIZE

    //==========================================================
    // ---> OPERATION
    //==========================================================
	//ループ処理の開始
	cerr << "Main Loop Started" << endl;

	int a_deg = 0, b_deg = 0;
	double angle_scale = 400.0;
	while(!gShutOff){
		
		int key = waitKey(1);
        
        // カメラ操作
        if( key == 65362 ) // Up
            b_deg = MIN( 90, b_deg+DELTA_DEG );

        else if( key == 65364 ) // Down
            b_deg = MAX( -90, b_deg-DELTA_DEG);

        else if( key == 65361 ){ // Left
            a_deg += DELTA_DEG;
            if(a_deg>360) a_deg -= 360;
        }
        else if( key == 65363 ){ // right
            a_deg -= DELTA_DEG;
            if(a_deg<0) a_deg += 360;
        }
       	
		// 画像操作
		else if( key == 'z' )
			angle_scale *= 1.125;
		else if( key == 'x' )
			angle_scale /= 1.125;

        else if( key == 'a' ){
			whole_angle = (whole_angle + 1 ) % 4;
		}
        
        // key == Esc, 'q'
        if( key == 27 || key == 'q'){
			gShutOff = true;
			break;
		}



		// 画像読み込み
		cam_image.readLast();
		ImageC3_HD720p_to_Mat(cam_image.data,&frm);
		
		
        // 投影変換行列
        double c_a = cos(a_deg*M_PI/180.0);
        double s_a = sin(a_deg*M_PI/180.0);
        double c_b = cos(b_deg*M_PI/180.0);
        double s_b = sin(b_deg*M_PI/180.0);
        double c_c = cos(whole_angle*M_PI/2.0);
        double s_c = sin(whole_angle*M_PI/2.0);
        double shift_u = WIDTH / 2;
        double shift_v = HEIGHT / 2;
        cv::Mat T = (Mat_<double>(3,3) << 1.0, 0.0, shift_u, 0.0, 1.0, shift_v, 0.0, 0.0, 1.0);
        cv::Mat S = (Mat_<double>(3,3) << angle_scale, 0.0, 0.0, 0.0, angle_scale, 0.0, 0.0, 0.0, 1.0);
        cv::Mat A = (Mat_<double>(3,3) << c_a, 0.0, s_a, 0.0, 1.0, 0.0, -s_a, 0.0, c_a);
        cv::Mat B = (Mat_<double>(3,3) << 1.0, 0.0, 0.0, 0.0, c_b, s_b, 0.0, -s_b, c_b);
        cv::Mat C = (Mat_<double>(3,3) << c_c, s_c, 0.0,-s_c, c_c, 0.0, 0.0, 0.0, 1.0 ); 
        cv::Mat projMat = T*S*B*A*C;

		// ソース画像 or 変換画像の表示
		if( show_srcimg ){
			imshow( "Theta S Original Image", frm );
		}
		else{
        	Mat calibImg;
        	makePerspectiveViewWithFixedParams( frm, calibImg, projMat, Size(WIDTH,HEIGHT) );
			imshow( "Perspective View", calibImg );
		}
        
        usleep(25000);

	}
    // <--- OPERATION

    //==========================================================
    // ---> FINALIZE
    //==========================================================
	cam_image.close();

    endSSM();
    cerr << "End SSM" << endl;
    // <--- FINALIZE

    cout << "End Successfully" << endl;
    return 0;
}

// 決め打ちのカメラ内部パラメータでTHETASの画像を透視射影に変換する
void makePerspectiveViewWithFixedParams( Mat img, Mat &dst, Matx33d prjMat, Size size)
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


void showHelp(void)
{
	// 書式
	fprintf( stdout, "\n\n" );
	fprintf( stdout, "\033[1m書式\033[0m\n" );
	fprintf( stdout, "\t\033[1mthetas-viewer\033[0m [-i ssmID] [-r rot]\n" );
	fprintf( stdout, "\t\033[1mthetas-viewer\033[0m [-s]\n" );
	fprintf( stdout, "\t\033[1mthetas-viewer\033[0m [-h]\n" );
	fprintf( stdout, "\n" );

	// 説明
	fprintf( stdout, "\n" );
	fprintf( stdout, "\033[1m説明\033[0m\n" );
	fprintf( stdout, "\t\033[1m-i\033[0m\tSSMのIDを指定する\n" );
	fprintf( stdout, "\t\033[1m-r\033[0m\t表示画像の初期回転角(90度毎)を指定する[0-3]\n" );
	fprintf( stdout, "\t\033[1m-s\033[0m\t透視図を表示せずにSSMのソース画像を表示する\n" );
	fprintf( stdout, "\t\033[1m-h\033[0m\tこのヘルプを表示する\n" );
	fprintf( stdout, "\n" );
	
	// 使用方法
	fprintf( stdout, "\n\033[1m使用方法\033[0m\n" );
	fprintf( stdout, "\t%-18s仰角と方位角の向きを%d度ずつ変更する\n", "矢印キー", DELTA_DEG );
	fprintf( stdout, "\t%-16s表示画像を90度回転させる\n", "A キー" );
	fprintf( stdout, "\t%-16s画角を狭くする(ズームイン)\n", "Z キー" );
	fprintf( stdout, "\t%-16s画角を広げる(ズームアウト)\n", "X キー" );
	fprintf( stdout, "\n" );

	// フッター
	fprintf( stdout, "\t\t\t\t\t\t\t2016年07月\n" );
	fprintf( stdout, "\n\n" );

}
