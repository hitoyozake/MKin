// 複数のKinectのカメラ画像を表示する
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <thread>

#include <fstream>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

#include <boost/lexical_cast.hpp>
// NuiApi.hの前にWindows.hをインクルードする
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../KinectTest/video.h"
#include "../KinectTest/video.cpp"


#define NO_MINMAX

int const KINECT_NUM = 1;

struct mouse_info
{
	int x1_, x2_, y1_, y2_;
	int flag_;

	mouse_info() : x1_( 0 ), x2_( 10 ), y1_( 0 ), y2_( 10 )
	{

	}
};

void on_mouse( int event, int x, int y, int flags, void *param )
{
	auto mouse = static_cast< mouse_info * >( param );
	switch(event){
	case CV_EVENT_MOUSEMOVE:
		break;
	case CV_EVENT_LBUTTONDOWN:
		cout << "hello" << endl;
		mouse->flag_ = false;
		mouse->x1_ = x;
		mouse->y1_ = y;
		// When Left button is pressed, ...
		break;

	case CV_EVENT_LBUTTONUP:
		mouse->x2_ = x;
		mouse->y2_ = y;
		cout << "hellohellohello" << endl;

		mouse->flag_ = true;
		// When Left button is released, ...
		break;

	case CV_EVENT_RBUTTONDOWN:

		break;

	case CV_EVENT_RBUTTONUP:

		break;

	default:
		break;
	}
}

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
	struct data
	{
		std::string window_name_;
		cv::Ptr< IplImage > image_;
	};

	data depth_;
	data color_;

	std::string filename_;
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
		graph[ i ].depth_.window_name_ = "MultiKinectPlayer[" + boost::lexical_cast< string >\
			( i + 1 ) + "] Depth";

		graph[ i ].color_.window_name_ = "MultiKinectPlayer[" + boost::lexical_cast< string >\
			( i + 1 ) + "] Color";
		graph[ i ].filename_ = string( "depth_" ) + boost::lexical_cast< string >\
			( i ) + ".txt";


		// OpenCVの初期設定
		graph[ i ].color_.image_ = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 4 );
		::cvNamedWindow( graph[ i ].color_.window_name_.c_str(), CV_WINDOW_KEEPRATIO );
		graph[ i ].depth_.image_ = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1  );
		::cvNamedWindow( graph[ i ].depth_.window_name_.c_str(), CV_WINDOW_KEEPRATIO );
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
			Sleep( 600 );
		}
	}
}

cv::Ptr< IplImage > convert_color_from_depth( cv::Ptr< IplImage > depth )
{
	auto const width = depth->width;
	auto const height = depth->height;
	cv::Ptr< IplImage > color = cvCreateImage( cvSize( width, height ), \
		IPL_DEPTH_8U, 4 );

	for( int x = 0; x < width; ++x )
	{
		for( int y = 0; y < height; ++y )
		{
			auto * pixel_ptr = & color->imageData[ x * 4 + width * y * 4 ];
			auto const pixel = ( ( ( UINT16 * )( depth->imageData +\
				depth->widthStep * y ) )[ x ] ) >> 3;

			const int tmp = 0;

			pixel_ptr[ 0 ] = 50;
			pixel_ptr[ 1 ] = 50;
			pixel_ptr[ 2 ] = 50;


			if( pixel < 650 && pixel >= 10)
			{
				pixel_ptr[ 0 ] = 0;
				pixel_ptr[ 1 ]  = ( char )( ( pixel - 400 ) * ( 255.0 / 250.0 ) ); 
				pixel_ptr[ 2 ] = 255;
			}
			if( pixel < 1250 && pixel >= 650 )
			{
				pixel_ptr[ 0 ] = 0;
				pixel_ptr[ 1 ] = 255;
				pixel_ptr[ 2 ]  =  ( char )( 255 - ( pixel - 650 )* ( 255.0 / 600.0 ) ); 
			}
			if( pixel < 1750 && pixel >= 1250 )
			{
				pixel_ptr[ 0 ] = ( char )( ( pixel - 1250 ) * ( 255.0 / 500.0 ) );
				pixel_ptr[ 1 ] = 255;
				pixel_ptr[ 2 ]  =  0; 
			}
			if( pixel < 2250 && pixel >= 1750 )
			{
				pixel_ptr[ 0 ] = 255;
				pixel_ptr[ 1 ] = ( char )( 255 - ( pixel - 1750 )* ( 255.0 / 500.0 ) );
				pixel_ptr[ 2 ]  = ( char )( ( pixel - 1750 ) * ( 255.0 / 500.0 ) ); 
			}
			if( pixel < 2750 && pixel >= 2250 )
			{
				pixel_ptr[ 0 ] = 255;
				pixel_ptr[ 1 ] = 0;
				pixel_ptr[ 2 ]  = ( char )( 255 - ( pixel - 2250 ) * ( 255.0 / 500.0 ) ); 
			}
			if( pixel < 3250 && pixel >= 2750 )
			{
				pixel_ptr[ 0 ] = static_cast< char >( 255 - ( pixel - 2750 ) * ( 255.0 / 500.0 ) );
				pixel_ptr[ 1 ] = static_cast< char >( 255 - ( pixel - 2750 ) * ( 255.0 / 500.0 ) );
				pixel_ptr[ 2 ]  = static_cast< char >( 255 - ( pixel - 2750 ) * ( 255.0 / 500.0 ) ); 
			}

			if( pixel >= 3250 )
			{
				pixel_ptr[ 0 ] = 140;
				pixel_ptr[ 1 ] = 140;
				pixel_ptr[ 2 ] = 140; 
			}
		}
	}
	return color;
}

std::vector< std::string > get_filelist_from_current_dir()
{
	namespace fs = boost::filesystem;

	std::vector< std::string > result;

	const fs::path path( fs::current_path() );
	fs::directory_iterator end;
	for( fs::directory_iterator i( fs::current_path() ); i != end; ++i )
	{

		std::cout << i->path() << std::endl;

		if( ! fs::is_directory( i->path() ) )
		{
			result.push_back( i->path().string() );
		}
	}

	return result;
}

std::vector< std::string > get_recorded_filelist( std::vector< std::string > const & filelist )
{
	std::vector< std::string > result;

	using boost::regex;

	regex reg_ex( "([0-9]*[a-z]*[A-Z]*)*.txt$");

	boost::smatch match;

	for( auto & i : filelist )
	{
		cout << i << endl;
		//正規表現でデータファイルを探す
		//if( boost::regex_search( ( std::string )( i.c_str() ), match, reg_ex ) )
		if( i.find( "depth" ) != std::string::npos )
		{
			std::cout << i << std::endl;

			result.push_back( i );
		}
	}
	return result;
}



void draw()
{
	using namespace std;
	int const kinect_count = 1;
	bool const use_mouse = false;
	vector< ifstream >  ifs_depth;//( kinect_count );
	vector< ifstream >  ifs_color( 1 );//( kinect_count );

	//std::ifstream ifs_timestamp( "debug_log.txt");

	mouse_info mouse;
	//size_t filesize = ( size_t )ifs.seekg( 0, std::ios::end).tellg();
	//video::vfw_manager video_m( "output.avi", "output.avi", 640, 480, 1, 30, 30 * 60 * 60 );
	//ifs.seekg( 0, std::ios::beg );

	//std::cout << "size:" << filesize << std::endl;

	auto const filelist = get_recorded_filelist( get_filelist_from_current_dir() );

	for( int i = 0; i < 1 + 0 * filelist.size(); ++i )
	{
		auto const filename_d = filelist[ i ];
		//auto const filename_c = string( "color_" ) + boost::lexical_cast< string >\
		( i ) + ".txt";

		ifs_depth.push_back( ifstream() );

		ifs_depth[ i ].open( filename_d, ios::binary );
		//ifs_color[ i ].open( filename_c, ios::binary );
	}
	ifs_color[ 0 ].open( "color_0.txt", ios::binary  );

	try {

		std::vector< graph > graph( ifs_depth.size() );

		bool continue_flag = true;
		int count = 0;

		long now_angle = 0;

		init( graph );

		int x1 = 0, x2 = 10, y1 = 0, y2 = 10;

		bool pause = false;

		while ( ( graph.size() > 0 ) && continue_flag )
		{
			{
				if( mouse.flag_ )
				{
					mouse.flag_ = 0;
					x1 = max( 0, min( mouse.x1_ , mouse.x2_ ) );
					x2 = min( 160, max( mouse.x1_ , mouse.x2_ ) );
					y1 = max( 0, min( mouse.y1_ , mouse.y2_ ) );
					y2 = min( 120, max( mouse.y1_ , mouse.y2_ ) );
				}
			}

			cvSetMouseCallback( "MultiKinectPlayer[1] Depth", on_mouse, & mouse );

			if( ! pause )
			{

				for( size_t i = 0; i < graph.size(); ++i )
				{
					ifs_depth[ i ].read( graph[ i ].depth_.image_->imageData, 640 * 480 * 2 ); 

					ifs_color[ i ].read( graph[ i ].color_.image_->imageData, 640 * 480 * 4 ); 

					//if( use_mouse )
					{
						// 画像データの取得

						// データのコピーと表示

						//ifs_color[ i ].read( graph[ i ].color_.image_->imageData, 640 * 480 * 4 ); 

						Sleep( 1 );

						if( ifs_depth[ i ].eof() )
							continue_flag = false;

						unsigned short max_value = 3100, min_value = 655300;

						for( int h = y1; h < y2; ++h )
						{
							for( int w = x1; w < x2; ++w )
							{
								unsigned short pixel = ( ( UINT16 * )( graph[ i ].depth_.image_->imageData +\
									graph[ i ].depth_.image_->widthStep * h ) )[ w ];
								//cout << pixel << endl;


								if( ( y2 - y1 ) * ( x2 - x1 ) < 50 )
									std::cout << pixel / 8 << endl;

							}
						}

						for( int h = y1; h < y2; ++h )
						{
							for( int w = x1; w < x2; ++w )
							{
								( ( UINT16 * )( graph[ i ].depth_.image_->imageData +\
									graph[ i ].depth_.image_->widthStep * h ) )[ w ] -= min_value;

								//代入
								( ( UINT16 * )( graph[ i ].depth_.image_->imageData +\
									graph[ i ].depth_.image_->widthStep * h ) )[ w ] = 
									( ( UINT16 * )( graph[ i ].depth_.image_->imageData +\
									graph[ i ].depth_.image_->widthStep * h ) )[ w ] * 65530.0
									/ ( max_value - min_value );
							}
						}

					}

					
					//選択領域から最大のものと最小の画素を選んでその値で割る? 3000 - 3500 x / 3500
					auto c = convert_color_from_depth( graph[ i ].depth_.image_ );
					//video_m.write( true, c );
					::cvShowImage( graph[ i ].depth_.window_name_.c_str(), graph[ i ].depth_.image_ );
					
					//::cvShowImage( graph[ i ].depth_.window_name_.c_str(), c );
					::cvShowImage( graph[ i ].color_.window_name_.c_str(), graph[ i ].color_.image_ );
					//::cvShowImage( graph[ i ].depth_.window_name_.c_str(), graph[ i ].depth_.image_ );
					//::cvShowImage( graph[ i ].color_.window_name_.c_str(), graph[ i ].color_.image_ );

					cout << "frame : " << ++count << endl;
					c.release();

					//if( ! ( ifs_timestamp.fail() || ifs_timestamp.eof() ) )
					//{
					//	double time;
					//	ifs_timestamp >> time ;

					//	if( time > 0.0 && time < 10.0 )
					//	{
					//		//Sleep( time * 1000 );
					//	}
					//	else
					//	{
					//	}

					//}

				}
			}
			int key = ::cvWaitKey( 20 );
			if ( key == 'q' ) {
				continue_flag = false;
			}
			else if( key == 'p' )
			{
				pause = ! pause;
			}


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