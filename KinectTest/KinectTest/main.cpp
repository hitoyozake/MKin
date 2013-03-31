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

#include <boost\shared_ptr.hpp>
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
		};

		image_data color_;
		image_data depth_;

		boost::shared_ptr< std::ofstream > ofs_d_;
		boost::shared_ptr< std::ofstream > ofs_c_;

		boost::shared_ptr< video::vfw_manager > vw_;

		int id_;
	};

	void set_mortor( long const angle, INuiSensor & kinect )
	{
		long const max_angle = 27;
		long const min_angle = -27;		
		auto const value = std::max( min_angle, std::min( max_angle, angle ) );
		kinect.NuiCameraElevationSetAngle( value );
		Sleep( 100 );
	}

	//現在の日時を取得
	//std::string generate_current_time()
	//{
	//	auto const today = boost::gregorian::day_clock::local_day();
	//	std::string today_as_iso = boost::gregorian::to_iso_string( today );
	//
	//	using boost::posix_time::ptime;
	//	using boost::posix_time::second_clock;
	//	ptime now = second_clock::local_time(); // 実行しているロケールの現在時刻
	//	std::string time = boost::posix_time::to_iso_string( now );
	//
	//	return today_as_iso + time;
	//
	//}

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



	void init( std::vector< Runtime > & runtime, std::string const & current_time, bool const color_view = false )
	{
		using namespace std;
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

		for( size_t i = 0; i < runtime.size(); ++i )
		{
			NuiCreateSensorByIndex( i, & runtime[ i ].kinect_ );

			runtime[i].kinect_->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH
				| NUI_INITIALIZE_FLAG_USES_AUDIO );

			std::string drive = "";//"F:\\recorded_data\\";
			runtime[ i ].vw_ = boost::shared_ptr< video::vfw_manager >
				( new video::vfw_manager( drive + to_string( i ) + "_" + current_time + "_output.avi", to_string( i ) + "_" + current_time + "_output.avi", \
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
			::cvNamedWindow( "depth",  CV_WINDOW_KEEPRATIO );

			//深度==============================================================
			::NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_640x480, x, y );	

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
	}


	boost::optional< NUI_LOCKED_RECT > get_image( NUI_IMAGE_FRAME const & image_frame, std::string const & kind )
		// 画像データの取得
	{
		NUI_LOCKED_RECT rect;
		auto hRes = image_frame.pFrameTexture->LockRect( 0, & rect, 0, 0 );

		if(hRes != S_OK){
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
				std::cout << "ending : queue SIZE : " << image_queue.size() << std::endl;
			auto image = image_queue.front();
			image_queue.pop();
			vfw->write( true, image );
		}
		vfw->close();
	}


	void kinect_thread( Runtime & runtime, int & go_sign, int & end_sign, int & ready_sign )
	{
		using namespace std;
		//go_signをオフにするのはこっち. readyをオフにするのはメイン

		int count = 0;
		char prev_frame[ 640 * 480 ];

		//ビデオライタのスレッド
		bool video_end = false, video_queue_writing = false;
		queue< cv::Ptr< IplImage > > image_queue;

		thread vw_thread( video_thread, runtime.vw_, std::ref( image_queue ), \
			std::ref( video_queue_writing ), std::ref( video_end ) );

		cv::Ptr< IplImage > depth_image = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );
		cv::Ptr< IplImage > resized = cvCreateImage( cvSize( runtime.vw_->width(), runtime.vw_->height() ), IPL_DEPTH_8U, 4 );

		cout << "test" << endl;

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

				// カメラデータの取得
				NUI_IMAGE_FRAME image_frame_depth_;
				NUI_IMAGE_FRAME image_frame_color_;

				NUI_IMAGE_FRAME * image_frame_depth = & image_frame_depth_;
				NUI_IMAGE_FRAME * image_frame_color = & image_frame_color_;

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
					runtime.ofs_c_->write( runtime.color_.image_->imageData, runtime.color_.image_->widthStep * runtime.color_.image_->height );
					//resized.release();

					runtime.ofs_d_->write( depth_image->imageData, depth_image->widthStep * depth_image->height );
					Sleep( 400 );
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

						::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );

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
						runtime.ofs_c_->write( runtime.color_.image_->imageData, runtime.color_.image_->widthStep * runtime.color_.image_->height );
						//resized.release();

					}
					// 画像データの取得
					if( auto rect = std::move( get_image( image_frame_depth_, "DEPTH" ) ) )
					{
						// データのコピーと表示
						memcpy( depth_image->imageData, (BYTE*)rect->pBits, \
							depth_image->widthStep * depth_image->height );
						cvFlip( depth_image, depth_image, 1 );

						//::cvShowImage( "depth", depth_image );
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
										pixel_ptr[ 1 ] = ( char )( 255 - ( pixel - 1950 )* ( 255.0 / 650.0 ) );
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
							::cvShowImage( runtime.depth_.window_name_.c_str(), runtime.depth_.image_ );

						}
						else
						{
							::cvShowImage( runtime.depth_.window_name_.c_str(), depth_image );
						}
						runtime.ofs_d_->write( ( char * )depth_image->imageData, depth_image->widthStep * depth_image->height );
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

		std::cout << vw_thread.get_id() << "・・・・";
		vw_thread.join();
		std::cout << " Video Thread End" << std::endl;
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

	void draw()
	{
		using namespace std;

		auto const current_time = generate_current_day_and_time();

		ofstream dlog( "debug_log_" + current_time + ".txt" );
		//解像度の設定
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

		// アクティブなKinectの数を取得する
		int kinect_count = 0;
		::NuiGetSensorCount( & kinect_count );

		// Kinectのインスタンスを生成する
		typedef std::vector< Runtime > Runtimes;
		Runtimes runtime( kinect_count );
		//runtime.push_back( Runtime() );

		{
			bool color_view = false;

			cout << "COLOR VIEW MODE : ";
			int mode = 0;

			cin >> mode;

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

		for( int i = 0; i < kinect_count; ++i )
		{
			//NearModeの設定

			string const drive = "F:\\recorded_data\\";

			string const filename_d = string( "depth_" ) + "_" + current_time  + boost::lexical_cast< string >\
				( i ) + ".txt";
			string const filename_c = string( "color_" ) +  "_" + current_time + boost::lexical_cast< string >\
				( i ) + ".txt";
			string const filename_ts = string( "timestamp_" ) + "_" + current_time \
				+ boost::lexical_cast< string >\
				( i ) + ".txt";

			runtime[ i ].ofs_c_ = boost::shared_ptr< ofstream >( new ofstream() );
			runtime[ i ].ofs_d_ = boost::shared_ptr< ofstream >( new ofstream() );
			runtime[ i ].ofs_d_->open( filename_d, ios::binary );
			runtime[ i ].ofs_c_->open( drive + filename_c, ios::binary );
			
			runtime[ i ].id_ = i;

			kinect_thread_obj[ i ] = thread( kinect_thread, \
				ref( runtime[ i ] ), ref( go_sign[ i ] ),ref( end_sign[ i ] ), \
				ref( ready_sign[ i ] ) );
		}

		//開始 ->find_ifで書き換え可能コード
		for( auto & i : go_sign )
			i = 1;
		boost::timer::cpu_timer timer;
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