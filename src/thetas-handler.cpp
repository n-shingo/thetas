//-------------------------------------------------
// thetas-handler.cpp
// S. NAKAMURA
// since: 2016-07-09
//-------------------------------------------------

#include<cstdio>
#include<iostream>
#include<signal.h>
#include"opencv2/opencv.hpp"
#include<ssm.hpp>
#include"SSM-Image-HD720p.hpp"

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

// ヘルプ
void showHelp(void);

int main(int argc, char ** argv)
{
    //==========================================================
    // ---> DECLARATION
    //==========================================================
    int cam_id = 0;
    int ssm_id = 0;
	Mat frm;

    // <--- DECLARATION

    //==========================================================
    // ---> INITALIZE
    //==========================================================
    //--------------------------------------
    // オプション解析
    int c;
    while( (c = getopt(argc, argv, "hi:o:")) != -1 )
    {
        switch ( c )
        {
        case 'i':
            fprintf( stderr, "input camera id = %d\n", atoi(optarg) );
            cam_id = atoi(optarg);
            break;
        case 'o':
            fprintf( stderr, "output ssm id = %d\n", atoi(optarg) );
            ssm_id = atoi(optarg);
            break;
        case 'h':
        default:
			showHelp();
			return 0;
        }
    }

    if( !initSSM() )
    {
		cerr << "SSM Error : initSSM()" << endl;
		return 0;
	}

    SSMApi<ImageC3_HD720p> cam_image(SNAME_HD720P, ssm_id);

	//1秒保持、10fps <- 3秒保持、30fps
	//logicool webcam 9000が最大30fpsなので。
    if( !cam_image.create( 1.0, 1.0/10.0 ) )
    {
		cerr << "SSM Error : create()" << endl;
		return 1;
	}

	setSigInt();

	VideoCapture cap;
	
    cap.open( cam_id );
	
    if( !cap.isOpened() )
    {
		cerr << "failed open the capture device" << endl;
		return 1;
	}
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, HD720P_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HD720P_HEIGHT);

    // <--- INITALIZE

    //==========================================================
    // ---> OPERATION
    //==========================================================
	cerr << "Main Loop Started" << endl;

    int cnt = 0;
    while( !gShutOff )
    {
		int key;

		cap >> frm;

        Mat_to_ImageC3_HD720p(frm, &cam_image.data);

	
		if(cnt%3 == 0)
    	    cam_image.write();

		cnt++;	


		key = waitKey(1);
		switch(key){
		case 27: // ESC key
			gShutOff = true;
			break;
		default:
			break;
		}
	}

    // <--- OPERATION

    //==========================================================
    // ---> FINALIZE
    //==========================================================
	cam_image.release();

    endSSM();
    cerr << "End SSM" << endl;
    // <--- FINALIZE

    cout << "End Successfully" << endl;
    return 0;
}

void showHelp(void)
{
	fprintf( stdout, "\n" );

	// 書式
	fprintf( stdout, "\n" );
	fprintf( stdout, "\033[1m書式\033[0m\n" );
	fprintf( stdout, "\t\033[1mthetas-handler\033[0m [-i camID] [-o ssmID]\n" );
	fprintf( stdout, "\t\033[1mthetas-handler\033[0m [-h]\n" );
	fprintf( stdout, "\n" );

	// 説明
	fprintf( stdout, "\n" );
	fprintf( stdout, "\033[1m説明\033[0m\n" );
	fprintf( stdout, "\t\033[1m-i\033[0m\tカメラのIDを指定する\n" );
	fprintf( stdout, "\t\033[1m-o\033[0m\tSSMのIDを指定する\n" );
	fprintf( stdout, "\t\033[1m-h\033[0m\tこのヘルプを表示する\n" );
	fprintf( stdout, "\n" );

	// フッター
	fprintf( stdout, "\t\t\t\t\t\t\t2016年7月\n" );
	fprintf( stdout, "\n\n" );

}
