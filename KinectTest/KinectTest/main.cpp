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

#include "video.h"
#include "filesystem.h"
#include "sound_recorder.h"


#pragma comment( lib, "x86/Kinect10.lib" )
#pragma comment( lib, "libboost_timer-vc110-mt-gd-1_51.lib" )


#define NO_MINMAX

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

		boost::shared_ptr< video::vfw_manager > vw_;
		std::string drive_;

		int id_;
		bool record_start_flag_;
	};

	struct mouse_info
	{
		int x1_, x2_, y1_, y2_;
		bool flag_;

		bool event_;

		mouse_info() : x1_( 0 ), x2_( 10 ), y1_( 0 ), y2_( 10 )
		{
		}
	};


	void set_mortor( long const angle, INuiSensor & kinect )
	{
		long const max_angle = 27;
		long const min_angle = -27;		
		auto const value = std::max( min_angle, std::min( max_angle, angle ) );
		kinect.NuiCameraElevationSetAngle( value );
		Sleep( 100 );
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

	int init( std::vector< Runtime > & runtime, std::string const & current_time, bool const color_view = false )
	{
		using namespace std;
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;


		for( size_t i = 0; i < runtime.size(); ++i )
		{
			runtime[ i ].record_start_flag_ = false;

			NuiCreateSensorByIndex( i, & runtime[ i ].kinect_ );

			runtime[i].kinect_->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH
				| NUI_INITIALIZE_FLAG_USES_AUDIO );


			runtime[ i ].vw_ = boost::shared_ptr< video::vfw_manager >
				( new video::vfw_manager( (std::string)"C:/recorded_data/" + to_string( i ) + "_" + current_time + "_output.avi", to_string( i ) + "_" + current_time + "_output.avi", \
				640, 480, 1, 30, 30 * 60 * 60 * 4 ) );

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
			::cvNamedWindow( runtime[ i ].color_.window_name_.c_str(),  CV_WINDOW_KEEPRATIO );

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

			::cvNamedWindow( runtime[ i ].depth_.window_name_.c_str(),  CV_WINDOW_KEEPRATIO );

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

		return 0;
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
	void wait_input( bool & input_come, std::string & input )
	{
		while( input != "end" )
		{
			if( ! input_come )
			{
				std::cin >> input;
				input_come = true;
				Sleep( 100 );
			}
		}
	}

	void video_thread( boost::shared_ptr< video::vfw_manager > vfw, std::queue< cv::Ptr< IplImage > > & image_queue, \
		bool & reading, bool & end_flag )
	{
		while( ! end_flag )
		{
			if( ! reading && ! image_queue.empty() )
			{
				reading = true;
				auto image = image_queue.front();
				image_queue.pop();
				reading = false;

				vfw->write( true, image );
				//std::cout << "queue SIZE : " << image_queue.size() << std::endl;
			}
			else
			{
				Sleep( 5 );
				continue;
			}
			Sleep( 1 );
		}

		//終了前に全て書き込む
		while( ! image_queue.empty() )
		{
			if( image_queue.size() % 50 == 0 )
			{
				//残り書き込み量を表示
				std::cout << "now ending : last queue SIZE : " << image_queue.size() << std::endl;
			}
			auto image = image_queue.front();
			image_queue.pop();
			vfw->write( true, image );
		}
		vfw->close();
	}



	//mouse_infoは別スレッドで書き換えられるので注意
	void kinect_thread( Runtime & runtime, int & go_sign, int & end_sign, int & ready_sign, mouse_info const & mi )
	{
		using namespace std;
		//go_signをオフにするのはこっち. readyをオフにするのはメイン

		int count = 0;
		char prev_frame[ 640 * 480 ];

		//ビデオライタのスレッド
		bool video_end = false, video_queue_writing = false;
		queue< cv::Ptr< IplImage > > image_queue;

		//thread vw_thread( video_thread, runtime.vw_, std::ref( image_queue ), \
			std::ref( video_queue_writing ), std::ref( video_end ) );

		bool rect_init = false;

		cv::Ptr< IplImage > depth_image = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );
		cv::Ptr< IplImage > resized = cvCreateImage( cvSize( runtime.vw_->width(), runtime.vw_->height() ), IPL_DEPTH_8U, 4 );
		cv::Ptr< IplImage > color_320x240 = cvCreateImage( cvSize( 320, 240 ), IPL_DEPTH_8U, 4 );
		cout << "test" << endl;

		int debug_counter = 0;

		// カメラデータの取得用フレームオブジェクト
		NUI_IMAGE_FRAME image_frame_depth_;
		NUI_IMAGE_FRAME image_frame_color_;

		NUI_IMAGE_FRAME * image_frame_depth = & image_frame_depth_;
		NUI_IMAGE_FRAME * image_frame_color = & image_frame_color_;

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
						image_get_succeeded = false;
					}
				}
				{
					auto hRes = runtime.kinect_->NuiImageStreamGetNextFrame( runtime.color_.stream_handle_, 0, image_frame_color );
					if( hRes != S_OK ){
						printf(" ERR: [%d]COLOR%dフレーム取得失敗. NuiImageStreamGetNextFrame() returns %d.\n", runtime.id_, count, hRes );
						image_get_succeeded = false;
					}
				}
				// 画像データの取得
				if( ! image_get_succeeded )
				{
					//if failed then write pre frame
					//cvResize( runtime.color_.image_, resized );

					//if( video_queue_writing )
					//{
					//	//スピン待機
					//	while( video_queue_writing )
					//	{
					//		Sleep( 1 );
					//	}
					//}
					////キュー追加
					//video_queue_writing = true;
					//image_queue.push( resized );
					//video_queue_writing = false;
					if( runtime.record_start_flag_ )
					{
						runtime.ofs_d_->write( runtime.depth_.buf_->imageData, runtime.depth_.buf_->widthStep * runtime.depth_.buf_->height );
						runtime.ofs_c_->write( runtime.color_.buf_->imageData, runtime.color_.buf_->widthStep * runtime.color_.buf_->height );
						cvResetImageROI( depth_image );
						cvSetImageROI( depth_image, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );

						cvCopy( depth_image, runtime.depth_.buf_ );

						runtime.ofs_d_->write( runtime.depth_.buf_->imageData, runtime.depth_.buf_->widthStep * runtime.depth_.buf_->height );

						cvSetImageROI( runtime.color_.image_, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );

						cvCopy( runtime.color_.image_, runtime.color_.buf_ );

						runtime.ofs_c_->write( \
							runtime.color_.buf_->imageData, runtime.color_.buf_->widthStep * runtime.color_.buf_->height );


						::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );

						cvResetImageROI( runtime.color_.image_ );

						cvResetImageROI( depth_image );//なくても良いかも

					}
					Sleep( 300 );
					ready_sign = 1;
				}
				else 
				{ 
					if( auto rect = std::move( get_image( image_frame_color_, "COLOR" ) ) )
					{
						// データのコピーと表示
						memcpy( runtime.color_.image_->imageData, (BYTE*)rect->pBits, \
							runtime.color_.image_->widthStep * runtime.color_.image_->height );

						//左右反転
						cvFlip( runtime.color_.image_, runtime.color_.image_, 1 );
						//cvResize( runtime.color_.image_, resized );

						//if( video_queue_writing )
						//{
						//	//スピン待機
						//	while( video_queue_writing )
						//	{
						//		Sleep( 1 );
						//	}
						//}
						////キュー追加
						//video_queue_writing = true;
						//image_queue.push( resized );
						//video_queue_writing = false;
						//cvResize( runtime.color_.image_, color_320x240 );
						//runtime.ofs_c_->write( color_320x240->imageData, color_320x240->widthStep * color_320x240->height );
						if( runtime.record_start_flag_ )
						{
							cvSetImageROI( runtime.color_.image_, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );

							cvCopy( runtime.color_.image_, runtime.color_.buf_ );

							runtime.ofs_c_->write( \
								runtime.color_.buf_->imageData, runtime.color_.buf_->widthStep * runtime.color_.buf_->height );


							::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );

							cvResetImageROI( runtime.color_.image_ );
						}
						else
						{
							cvSetImageROI( runtime.color_.image_, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );

							::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );
							cvResetImageROI( runtime.color_.image_ );

						}
						//resized.release();

					}
					// 画像データの取得
					if( auto rect = std::move( get_image( image_frame_depth_, "DEPTH" ) ) )
					{
						// データのコピーと表示
						memcpy( depth_image->imageData, static_cast< BYTE * >( rect->pBits ), \
							depth_image->widthStep * depth_image->height );
						cvFlip( depth_image, depth_image, 1 );

						//::cvShowImage( "depth", depth_image );
#pragma region 色変換
						if( runtime.depth_.image_->nChannels == 4 )
						{
							for( int y = 0; y < 480; ++y )
							{
								//ピクセル書き換え
								for( int x = 0; x < 640; ++x )
								{
									auto * pixel_ptr = & runtime.depth_.image_->imageData[ x * 4 + 640 * y * 4 ];
									auto const pixel = ( ( UINT16 * )( depth_image->imageData +\
										depth_image->widthStep * y ) )[ x ] / 8;

									pixel_ptr[ 0 ] = 50;
									pixel_ptr[ 1 ] = 50;
									pixel_ptr[ 2 ] = 50;

									if( pixel < 650 && pixel >= 10)
									{
										pixel_ptr[ 0 ] = 0;
										pixel_ptr[ 1 ]  = ( char )( ( pixel - 400 ) * ( 255.0 / 250.0 ) ); 
										pixel_ptr[ 2 ] = 255;
									}
									if( pixel < 1300 && pixel >= 650 )
									{
										pixel_ptr[ 0 ] = 0;
										pixel_ptr[ 1 ] = 255;
										pixel_ptr[ 2 ]  =  ( char )( 255 - ( pixel - 650 )* ( 255.0 / 650.0 ) ); 
									}
									if( pixel < 1950 && pixel >= 1300 )
									{
										pixel_ptr[ 0 ] = ( char )( ( pixel - 1300 ) * ( 255.0 / 650.0 ) );
										pixel_ptr[ 1 ] = 255;
										pixel_ptr[ 2 ]  =  0; 
									}
									if( pixel < 2600 && pixel >= 1950 )
									{
										pixel_ptr[ 0 ] = 255;
										pixel_ptr[ 1 ] = ( char )( 255 - ( pixel - 1950 ) * ( 255.0 / 650.0 ) );
										pixel_ptr[ 2 ]  = ( char )( ( pixel - 1950 ) * ( 255.0 / 650.0 ) ); 
									}
									if( pixel < 3250 && pixel >= 2600 )
									{
										pixel_ptr[ 0 ] = 255;
										pixel_ptr[ 1 ] = 0;
										pixel_ptr[ 2 ]  = ( char )( 255 - ( pixel - 2600 ) * ( 255.0 / 650.0 ) ); 
									}
									if( pixel < 4000 && pixel >= 3250 )
									{
										pixel_ptr[ 0 ] = static_cast< char >( 255 - ( pixel - 3250 ) * ( 255.0 / 650.0 ) );
										pixel_ptr[ 1 ] = static_cast< char >( 255 - ( pixel - 3250 ) * ( 255.0 / 650.0 ) );
										pixel_ptr[ 2 ]  = static_cast< char >( 255 - ( pixel - 3250 ) * ( 255.0 / 650.0 ) ); 
									}

									if( pixel >= 4000 )
									{
										pixel_ptr[ 0 ] = 140;
										pixel_ptr[ 1 ] = 140;
										pixel_ptr[ 2 ]  = 140; 
									}

								}
							}
#pragma endregion

							if( runtime.record_start_flag_ )
							{
								cvSetImageROI( runtime.depth_.image_, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );
								::cvShowImage( runtime.depth_.window_name_.c_str(), runtime.depth_.buf_ );
								cvResetImageROI( runtime.depth_.image_ );//なくても良いかも
							}
							else
							{
								::cvShowImage( runtime.depth_.window_name_.c_str(), runtime.depth_.image_ );
							}
						}
						else
						{
							::cvShowImage( runtime.depth_.window_name_.c_str(), depth_image );
						}

						if( runtime.record_start_flag_ )
						{
							cvSetImageROI( depth_image, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );

							cvCopy( depth_image, runtime.depth_.buf_ );

							runtime.ofs_d_->write( runtime.depth_.buf_->imageData, runtime.depth_.buf_->widthStep * runtime.depth_.buf_->height );

							cvResetImageROI( depth_image );//なくても良いかも
						}
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

		video_end = true;

		resized.release();
		depth_image.release();

		//avi保存を使う時
		//std::cout << vw_thread.get_id() << "・・・・";
		//vw_thread.join();
		//std::cout << " Video Thread End" << std::endl;
		//VIDEOスレッドの終了と同時にAVIもcloseされる...
	}

	void end_task( vector< int > & end_sign, vector< thread > & kinect_thread_obj )
	{
		for( auto & it : end_sign )
			it = 1;

		for( auto & it : kinect_thread_obj )
		{
			it.join();
			std::cout << it.get_id() << " Kinect Thread END" << std::endl;
		}

	}


	struct rect
	{
		int x_, y_;
		int width_, height_;

		rect() : x_( 0 ), y_( 0 ), width_( 0 ), height_( 0 )
		{
		}

		rect( int const x, int const y, int const width, int const height ) : \
			x_( x ), y_( y ), width_( width ), height_( height ) 
		{
		}

	};

	std::vector< rect > get_rect_to_draw( std::string const & filename )
	{	
		std::ifstream area( filename );
		//読み込み範囲 個数\n x1:..\nx2:..\ny1:..\ny2:..; ":"でx1などの文字列を飛ばす

		if( area.is_open() )
		{
			int kinect_count = 0;
			area >> kinect_count;

			std::vector< rect > result( kinect_count );

			for( int i = 0; i < kinect_count; ++i )
			{
				std::string input;

				area >> input;

				int const x = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );

				area >> input;

				int const width = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );

				area >> input;
				int const y = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );

				area >> input;

				int const height = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );

				result[ i ].width_ = width - x;
				result[ i ].height_ = height - y;
				result[ i ].x_ = x;
				result[ i ].y_ = y;

			}
			return result;
		}
		return std::vector< rect >();	
	}

	void write_settingfile( std::string const & current_time, std::vector< std::string > const  & output_list )
	{
		std::ofstream ofs( std::string( "setting_" ) + current_time + ".txt" );

		ofs << current_time << std::endl;

		for( auto it = output_list.cbegin(); it != output_list.cend(); ++it )
		{
			ofs << * it << std::endl;
		}


		ofs.close();

	}

	void draw()
	{
		using namespace std;

		auto const current_time = generate_current_day_and_time();

		std::vector< std::string > setting_write_str;
		//current_time
		//キネクトの台数
		//range
		//drive
		setting_write_str.push_back( current_time );

		ofstream dlog( std::string( "C:/" ) + "debug_log_" + current_time + ".txt" );
		//解像度の設定
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;


		// アクティブなKinectの数を取得する
		int kinect_count = 0;
		::NuiGetSensorCount( std::addressof( kinect_count ) );

		//撮影範囲ファイルを読み込む
		std::string area_filename;
		std::getline( std::cin, area_filename );

		auto const list = get_rect_to_draw( area_filename );

		if( list.size() != static_cast< std::size_t >( kinect_count ) )
		{
			std::cout << "Kinectの数と設定ファイルに書かれたKinectの数が不一致です．" << std::endl;
			return;
		}
		setting_write_str.push_back( boost::lexical_cast< std::string >( kinect_count ) );

		// Kinectのインスタンスを生成する
		typedef std::vector< Runtime > Runtimes;
		Runtimes runtime( kinect_count );
		//runtime.push_back( Runtime() );

		{
			bool color_view = false;

			int mode = 1;

			//cin >> mode;

			if( mode != 0 )
				color_view = true;

			init( runtime, current_time, color_view );
		}
		bool continue_flag = true;
		int count = 0;

		long now_angle = 0;
		string input;
		bool input_come = false;

		vector< int > go_sign( kinect_count, 0 );	//スレッド管理用
		vector< int > ready_sign( kinect_count, 0 );
		vector< int > end_sign( kinect_count, 0 );

		thread input_wait_thread( wait_input, ref( input_come ), ref( input ) );
		vector< thread > kinect_thread_obj( kinect_count );
		vector< mouse_info > mouse( kinect_count );

		//ドライブの情報
		ifstream drive_info( "./drive.txt" );


		for( int i = 0; i < kinect_count; ++i )	
		{

			mouse[ i ].x1_ = list[ i ].x_;
			mouse[ i ].x2_ = list[ i ].x_ + list[ i ].width_;
			mouse[ i ].y1_ = list[ i ].y_;
			mouse[ i ].y2_ = list[ i ].y_ + list[ i ].height_;

			runtime[ i ].color_.buf_ = cvCreateImage( cvSize( mouse[ i ].x2_ - mouse[ i ].x1_, \
				mouse[ i ].y2_ - mouse[ i ].y1_ ),IPL_DEPTH_8U, 4 );
			runtime[ i ].depth_.buf_ = cvCreateImage( cvSize( mouse[ i ].x2_ - mouse[ i ].x1_, \
				mouse[ i ].y2_ - mouse[ i ].y1_ ),IPL_DEPTH_16U, 1 );


			setting_write_str.push_back( ( std::string )"width:" + boost::lexical_cast< std::string >( mouse[ i ].x2_ - mouse[ i ].x1_ ) );
			setting_write_str.push_back( ( std::string )"height:" + boost::lexical_cast< std::string >( mouse[ i ].y2_ - mouse[ i ].y1_ ) );

		}

		for( int i = 0; i < kinect_count; ++i )
		{
			drive_info >> runtime[i].drive_;
			setting_write_str.push_back( runtime[ i ].drive_ );
			string const filename_d = string( "depth" ) + "_" + current_time  + "_" + boost::lexical_cast< string >\
				( i ) + ".txt";
			string const filename_c = string( "color" ) +  "_" + current_time + "_" + boost::lexical_cast< string >\
				( i ) + ".txt";
			string const filename_ts = string( "timestamp_" ) + "_" + current_time \
				+ boost::lexical_cast< string >\
				( i ) + ".txt";

			runtime[ i ].ofs_c_ = boost::shared_ptr< ofstream >( new ofstream() );
			runtime[ i ].ofs_d_ = boost::shared_ptr< ofstream >( new ofstream() );
			runtime[ i ].ofs_d_->open( runtime[i].drive_ + ":/recorded_data/" + filename_d, ios::binary );
			runtime[ i ].ofs_c_->open( runtime[i].drive_ + ":/recorded_data/" + filename_c, ios::binary );

			runtime[ i ].id_ = i;

			kinect_thread_obj[ i ] = thread( kinect_thread, \
				ref( runtime[ i ] ), ref( go_sign[ i ] ),ref( end_sign[ i ] ), \
				ref( ready_sign[ i ] ), ref( mouse[ i ] ) );
		}


		sound_recorder::sound_recorder snd_rec( runtime[ 0 ].drive_ + ":/recorded_data/" + current_time + ".wav" );


		//セッティングファイルの書き出し
		write_settingfile( current_time, setting_write_str );

		//開始 ->find_ifで書き換え可能コード
		for( auto & i : go_sign )
			i = 1;
		boost::timer::cpu_timer timer;

		//ここまで準備
		std::cout << "ready" << std::endl;

		std::string start_command = "";

		//std::cin >> start_command;

		start_command = "start";


		while ( continue_flag )
		{
			int flag = 1;

			for( auto const i : ready_sign )
			{
				flag = flag & i;
			}

			//終了信号
			if( input_come )
			{
				if( input == "end" )
				{
					continue_flag = false;
					break;
				}
				if( input == "start" )
				{
					snd_rec.open_file();

					for( int i = 0; i < runtime.size(); ++i )
					{
						runtime[ i ].record_start_flag_ = true;
					}
					input = "";
				}

				input_come = false;
			}

			//cout << "hoge" << ++count << endl;
			int key = ::cvWaitKey( 10 );
			if ( key == 'q' ) {
				continue_flag = false;
			}

			if( flag == 1 )
			{
				//全部待機済みなので、次のフレーム
				for( auto & i : ready_sign )
					i = 0;
				for( auto & i : go_sign )
					i = 1;

				//ここで0.033秒待つ
				auto const dif_time =  static_cast< boost::timer::nanosecond_type >( 0.035 * 1000000000 ) - timer.elapsed().wall;
				if( dif_time > 0 )
				{
					auto const sleep_time = static_cast< long long >( dif_time * 0.000000001 * 1000 );
					std::this_thread::sleep_for( \
						std::chrono::milliseconds( sleep_time ) );
				}

				dlog << boost::format( "%2ld" ) %  ( timer.elapsed().wall * 0.000000001 * 1000 ) << endl;
				timer.start();
			}
		}

		end_task( end_sign, kinect_thread_obj );
		//スレッドの終了待ち
		input_wait_thread.join();

		std::cout << "Waiting for Kinect Shutdown" << std::endl;
		// 終了処理
		for( auto const & i : runtime )
		{
			//set_mortor( 10, * runtime[ 0 ].kinect );
			i.kinect_->NuiShutdown();	
		}
		snd_rec.stop_recording();

		//Windowを閉じる
		::cvDestroyAllWindows();

		std::cout << "PROGRAM WAS CLOSED" << std::endl;
	}
}
int main()
{
	recording::draw();


	/*using namespace std;

	if( auto volume = filesystem::get_last_volume_by_GB() )
	{
	std::cout << volume.get() << std::endl;
	}*/

	return 0;
}