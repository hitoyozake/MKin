// 複数のKinectのカメラ画像を表示する
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>

#include <fstream>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/lexical_cast.hpp>

// NuiApi.hの前にWindows.hをインクルードする
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define NO_MINMAX

struct Runtime
{
	INuiSensor *        kinect;         // Kinectのインスタンス

	struct image_data
	{
		HANDLE event_;	//データ更新イベントハンドル
		HANDLE stream_handle_;  //画像データのハンドル
		std::string window_name_;
		cv::Ptr< IplImage > image_; //表示データ
	};

	image_data color_;
	image_data depth_;

};

struct graph
{
	std::string window_name_;
	cv::Ptr< IplImage > image_;
};


void set_mortor( long const angle, INuiSensor & kinect )
{
	long const max_angle = 30;
	long const min_angle = -30;		
	auto const value = std::max( min_angle, std::min( max_angle, angle ) );
	kinect.NuiCameraElevationSetAngle( value );
	Sleep( 100 );
}

void init( std::vector< graph > & graph )
{
	using namespace std;
	
	for( size_t i = 0; i < graph.size(); ++i )
	{
		//深度==============================================================

		// ウィンドウ名を作成
		graph[ i ].window_name_ = "MultiKinectPlayer[" + boost::lexical_cast< string >\
			( i + 1 ) + "] Depth";

		// OpenCVの初期設定
		graph[ i ].image_ = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );
		::cvNamedWindow( graph[ i ].window_name_.c_str() );
	}
}


void wait_input( bool & input_come, std::string & input )
{
	while( input != "end" )
	{
		if( ! input_come )
		{
			std::cin >> input;
			input_come = true;
			Sleep( 300 );
		}
	}
}


void draw()
{
	using namespace std;

	ifstream ifs( "test.txt", ios::binary );

	//size_t filesize = ( size_t )ifs.seekg( 0, std::ios::end).tellg();

	//ifs.seekg( 0, std::ios::beg );

	//std::cout << "size:" << filesize << std::endl;

	try {

		std::vector< graph > graph( 3 );
		
		bool continue_flag = true;
		int count = 0;

		long now_angle = 0;

		init( graph );

		while ( continue_flag )
		{
			for( size_t i = 0; i < graph.size(); ++i )
			{
				// 画像データの取得

				// データのコピーと表示
				
				ifs.read( graph[ i ].image_->imageData, 640 * 480 * 2 ); 
				
				Sleep( 7 );

				if( ifs.eof() )
					continue_flag = false;
				::cvShowImage( graph[ i ].window_name_.c_str(), graph[ i ].image_ );


				int key = ::cvWaitKey( 10 );
				if ( key == 'q' ) {
					continue_flag = false;
				}
			}
			cout << "hoge" << ++count << endl;

		}
		

		::cvDestroyAllWindows();

	}
	catch ( std::exception & ex ) {
		std::cout << ex.what() << std::endl;
	}
}

int main()
{
	draw();
	return 0;
}