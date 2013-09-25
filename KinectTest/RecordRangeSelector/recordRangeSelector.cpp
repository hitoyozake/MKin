// 複数のKinectのカメラ画像を表示する
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <array>
#include <fstream>
#include <queue>
#include <chrono>
#include <boost/format.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/timer/timer.hpp>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/lexical_cast.hpp>
//#include <boost/date_time/gregorian/gregorian.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>

// NuiApi.hの前にWindows.hをインクルードする
#include <Windows.h>
#include <NuiApi.h>
#include <audiopolicy.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#pragma comment( lib, "x86/Kinect10.lib" )
#pragma comment( lib, "libboost_timer-vc110-mt-gd-1_51.lib" )

#define NO_MINMAX

//*****************************************
//プログラム概要
//	録画エリアをマウス２クリックで選択．
//　Kinectの数だけ選択する
//　結果はarea+timestamp.txtに保存される．
//*****************************************


namespace recording
{
	struct Runtime
	{
		INuiSensor * kinect_;         // Kinectのインスタンス

		struct image_data
		{
			HANDLE event_;	//データ更新イベントハンドル
			HANDLE stream_handle_;  //画像データのハンドル
			std::string window_name_;
			cv::Ptr< IplImage > image_; //表示データ
			cv::Ptr< IplImage > buf_;
		};

		image_data color_;
		image_data depth_;

		boost::shared_ptr< std::ofstream > ofs_d_;
		boost::shared_ptr< std::ofstream > ofs_c_;

		int id_;
	};

	struct mouse_info
	{
		int x1_, x2_, y1_, y2_;
		bool flag_;

		bool event_;

		int count_;

		mouse_info() : x1_( 0 ), x2_( 10 ), y1_( 0 ), y2_( 10 ), flag_( false ), count_ ( 0 )
		{
		}
	};

	void on_mouse( int event, int x, int y, int flags, void * param )
	{
		auto mouse = static_cast< mouse_info * >( param );

		if( mouse->event_ )
		{
			switch( event )
			{
			case CV_EVENT_MOUSEMOVE:
				break;
			case CV_EVENT_LBUTTONDOWN:
				if( mouse->flag_ == false )
				{
					if( mouse->count_ == 0 )
					{
						cout << "mouse Left button holding......." << endl;
						mouse->flag_ = true;
						mouse->x1_ = x;
						mouse->y1_ = y;
						mouse->x2_ = x + 1;
						mouse->y2_ = y + 1;
						// When Left button is pressed, ...
						break;
					}
					if( mouse->count_ == 1 )
					{
						cout << "mouse Left button holding2......" << endl;

						mouse->x2_ = x;
						mouse->y2_ = y;
						if( mouse->x1_ > mouse->x2_ )
						{
							swap( mouse->x1_, mouse->x2_ );
						}
						if( mouse->y1_ > mouse->y2_ )
						{
							swap( mouse->y1_, mouse->y2_ );
						}
						mouse->flag_ = true;
						// When Left button is released, ...
						++mouse->count_;
					}
				}
				break;
			case CV_EVENT_LBUTTONUP:
				++mouse->count_;
				mouse->flag_ = false;
				break;
			case CV_EVENT_RBUTTONDOWN:
				break;

			case CV_EVENT_RBUTTONUP:
				break;

			default:
				break;
			}
		}
	}

	void set_mortor( long const angle, INuiSensor & kinect )
	{
		long const max_angle = 27;
		long const min_angle = -27;		
		auto const value = std::max( min_angle, std::min( max_angle, angle ) );
		kinect.NuiCameraElevationSetAngle( value );
		Sleep( 100 );
	}

	void init( std::vector< Runtime > & runtime, std::string const & current_time, bool const color_view = false )
	{
		using namespace std;
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

		for( size_t i = 0; i < runtime.size(); ++i )
		{
			NuiCreateSensorByIndex( i, & runtime[ i ].kinect_ );

			runtime[i].kinect_->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH
				| NUI_INITIALIZE_FLAG_USES_AUDIO );

			runtime[ i ].color_.event_ = ::CreateEvent( 0, TRUE, FALSE, 0 );
			runtime[ i ].depth_.event_ = ::CreateEvent( 0, TRUE, FALSE, 0 );

			//Color=============================================================
			runtime[i].kinect_->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
				0, 2, runtime[i].color_.image_, &runtime[i].color_.stream_handle_ );

			// ウィンドウ名を作成
			runtime[i].color_.window_name_= "MultiKinect[" + boost::lexical_cast< string >\
				( i + 1 ) + "] Color";

			// 画面サイズを取得
			DWORD x = 0, y = 0;
			::NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_640x480, x, y );			

			// OpenCVの初期設定
			runtime[i].color_.image_ = ::cvCreateImage( cvSize( x, y ), IPL_DEPTH_8U, 4 );

			//深度==============================================================
			::NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_640x480, x, y );	

			//解像度を指定してStreamを開く
			runtime[i].kinect_->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
				0, 2, runtime[ i ].depth_.image_, & runtime[ i ].depth_.stream_handle_ );
			// ウィンドウ名を作成
			runtime[ i ].depth_.window_name_ = "MultiKinect[" + boost::lexical_cast< string >\
				( i + 1 ) + "] Depth";

			// 画面サイズを取得

			// OpenCVの初期設定
			if( color_view )
				runtime[i].depth_.image_ = ::cvCreateImage( cvSize( x, y ), IPL_DEPTH_8U, 4 );
			else
				runtime[ i ].depth_.image_ = ::cvCreateImage( cvSize( x, y ), IPL_DEPTH_16U, 1 );

			runtime[ i ].kinect_->NuiImageStreamSetImageFrameFlags( \
				runtime[i].color_.stream_handle_, \
				NUI_IMAGE_STREAM_FLAG_SUPPRESS_NO_FRAME_DATA //これで 無効フレーム抑制
				| NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE //Nearモード
				);

			runtime[ i ].kinect_->NuiImageStreamSetImageFrameFlags( \
				runtime[i].depth_.stream_handle_, \
				NUI_IMAGE_STREAM_FLAG_SUPPRESS_NO_FRAME_DATA //これで 無効フレーム抑制
				| NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE 
				);
		}
	}

	boost::optional< NUI_LOCKED_RECT > get_image( NUI_IMAGE_FRAME const & image_frame, std::string const & kind )
		// 画像データの取得
	{
		NUI_LOCKED_RECT rect;
		auto hRes = image_frame.pFrameTexture->LockRect( 0, std::addressof( rect ), 0, 0 );

		if( hRes != S_OK )
		{
			printf(" ERR: %sフレームバッファロック失敗. res=%d.", kind.c_str(), hRes );
			return boost::none;
		}
		else
			return boost::optional< NUI_LOCKED_RECT >( std::move( rect ) );
	}

	//mouse_infoは別スレッドで書き換えられるので注意
	void kinect_thread( Runtime & runtime, int & go_sign, int & end_sign, int & ready_sign, mouse_info & mi )
	{
		using namespace std;
		//go_signをオフにするのはこっち. readyをオフにするのはメイン

		int count = 0;

		//ビデオライタのスレッド
		bool video_end = false, video_queue_writing = false;
		queue< cv::Ptr< IplImage > > image_queue;

		bool rect_init = false;

		cv::Ptr< IplImage > depth_image = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );
		cv::Ptr< IplImage > color_320x240 = cvCreateImage( cvSize( 320, 240 ), IPL_DEPTH_8U, 4 );
		cout << "Kinect Thread Launched．" << endl;

		// カメラデータの取得用フレームオブジェクト
		NUI_IMAGE_FRAME image_frame_depth_;
		NUI_IMAGE_FRAME image_frame_color_;

		NUI_IMAGE_FRAME * image_frame_depth = std::addressof( image_frame_depth_ );
		NUI_IMAGE_FRAME * image_frame_color = std::addressof( image_frame_color_ );

		while( end_sign == 0 )
		{
			while( go_sign == 1 )
			{
				++count;

				go_sign = 0; 
				// データの更新を待つ
				// INFINITEで無効データが来ないっぽいが、同期だいじょうぶ?
				// StreamFlagsでとりあえず抑制
				::WaitForSingleObject( runtime.color_.stream_handle_, INFINITE );
				::WaitForSingleObject( runtime.depth_.stream_handle_, INFINITE );

				bool image_get_succeeded = true;

				{
					auto hRes = runtime.kinect_->NuiImageStreamGetNextFrame( runtime.depth_.stream_handle_, 0, image_frame_depth );
					if( hRes != S_OK ){
						printf(" ERR: [%d]DEPTH%dフレーム取得失敗. NuiImageStreamGetNextFrame() returns %d.\n", runtime.id_, count, hRes);
						ready_sign = 1;
						image_get_succeeded = false;
					}
				}
				{
					auto hRes = runtime.kinect_->NuiImageStreamGetNextFrame( runtime.color_.stream_handle_, 0, image_frame_color );
					if( hRes != S_OK ){
						printf(" ERR: [%d]COLOR%dフレーム取得失敗. NuiImageStreamGetNextFrame() returns %d.\n", runtime.id_, count, hRes );
						ready_sign = 1;
						image_get_succeeded = false;
					}
				}
				// 画像データの取得
				if( ! image_get_succeeded )
				{
					Sleep( 600 );
				}
				else 
				{ 
					if( auto const rect = std::move( get_image( image_frame_color_, "COLOR" ) ) )
					{
						// データのコピーと表示
						memcpy( runtime.color_.image_->imageData, (BYTE*)rect->pBits, \
							runtime.color_.image_->widthStep * runtime.color_.image_->height );

						//左右反転
						cvFlip( runtime.color_.image_, runtime.color_.image_, 1 );

						cvRectangle( runtime.color_.image_, cvPoint( mi.x1_, mi.y1_ ), cvPoint( mi.x2_, mi.y2_ ), \
							cv::Scalar( 0, 0, 255 ), 3 );

						::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );
					}

					// 画像データの取得
					if( auto const rect = std::move( get_image( image_frame_depth_, "DEPTH" ) ) )
					{
						// データのコピーと表示
						memcpy( depth_image->imageData, static_cast< BYTE * >( rect->pBits ), \
							depth_image->widthStep * depth_image->height );
						cvFlip( depth_image, depth_image, 1 );

						//::cvShowImage( "depth", depth_image );
#pragma region 色変換
					}
					// カメラデータの解放
					runtime.kinect_->NuiImageStreamReleaseFrame( runtime.color_.stream_handle_, image_frame_color );
					runtime.kinect_->NuiImageStreamReleaseFrame( runtime.depth_.stream_handle_, image_frame_depth );
					ready_sign = 1;
					Sleep( 5 );

				}
				if( end_sign == 1 )
				{
					break;
				}
			}
			Sleep( 2 );
		}

		cvRectangle( runtime.color_.image_, cvPoint( mi.x1_, mi.y1_ ), cvPoint( mi.x2_, mi.y2_ ), \
			cv::Scalar( 0, 0, 255 ), 3 );

		::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );

		Sleep( 500 );
		depth_image.release();

		std::cout << " Kinect Thread End" << std::endl;
		//VIDEOスレッドの終了と同時にAVIもcloseされる...
	}

	std::string generate_current_day_and_time()
	{
		using boost::gregorian::date;
		auto const today = boost::gregorian::day_clock::local_day();

		boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

		auto const now_str = boost::posix_time::to_iso_string( now );

		//yyyyMMddThhmmss
		//std::cout << boost::posix_time::to_iso_string( now ) << std::endl;
		return now_str;
	}

	void kinect_view( Runtime & runtime, std::ofstream & area )
	{
		mouse_info mouse;

		::cvNamedWindow( runtime.color_.window_name_.c_str(),  CV_WINDOW_KEEPRATIO );
		cvSetMouseCallback( runtime.color_.window_name_.c_str(), on_mouse, std::addressof( mouse ) );

		Sleep( 10 );

		int go_sign = 1;
		int end_sign = 0;
		int ready_sign = 1;

		auto k_th = thread( kinect_thread, \
			ref( runtime ), ref( go_sign ),ref( end_sign ), \
			ref( ready_sign ), ref( mouse ) );

		while( 1 )
		{
			Sleep( 40 );
			if( mouse.count_ >= 3 )
			{
				end_sign = 1;
				break;
			}
			if( ready_sign != 0 )
			{
				go_sign = 1;
			}
			cvWaitKey( 10 );
		}

		Sleep( 1000 );

		//mouse_infoの書き出し
		area << "x1:" << mouse.x1_ << "\nx2:" << mouse.x2_ << \
			"\ny1:" << mouse.y1_ << "\ny2:" << mouse.y2_ << std::endl;

		cvDestroyAllWindows();
		k_th.join();
	}

	void draw()
	{
		using namespace std;

		auto const current_time = generate_current_day_and_time();

		ofstream area( "area"+ current_time + ".txt" );

		//解像度の設定
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

		// アクティブなKinectの数を取得する
		int kinect_count = 0;
		::NuiGetSensorCount( std::addressof( kinect_count ) );

		// Kinectのインスタンスを生成する
		typedef std::vector< Runtime > Runtimes;
		Runtimes runtime( kinect_count );

		init( runtime, current_time, true );

		bool continue_flag = true;
		int count = 0;

		long now_angle = 0;
		string input;
		bool input_come = false;

		vector< int > go_sign( kinect_count, 0 );	//スレッド管理用
		vector< int > ready_sign( kinect_count, 0 );
		vector< int > end_sign( kinect_count, 0 );

		vector< mouse_info > mouse( kinect_count );

		area << kinect_count << endl;

		for( int i = 0; i < kinect_count; ++i )
		{
			mouse[ i ].event_ = true;
			runtime[ i ].id_ = i;
		}

		for( std::size_t i = 0; i < runtime.size(); ++i )
		{
			kinect_view( runtime[ i ], area );
		}

		std::cout << "Waiting for Kinect Shutdown" << std::endl;
		// 終了処理
		for( auto const & i : runtime )
		{
			//set_mortor( 10, * runtime[ 0 ].kinect );
			i.kinect_->NuiShutdown();	
		}

		std::cout << "PROGRAM WAS CLOSED" << std::endl;

	}
}

int main()
{
	recording::draw();

	return 0;
}