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
#include <Windows.h>
#include <tchar.h>
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

#include "hstgrm.h"

#pragma comment( lib, "x86/Kinect10.lib" )
#pragma comment( lib, "libboost_timer-vc110-mt-gd-1_51.lib" )

#define NO_MINMAX

namespace recording
{
	cpp_plot::wnd_proc_messenger hstgrm;
	int hstgrm_kinect_id = 0;
	const TCHAR szWndClass[] = _T( "TestProgram" );
	// Processes messages for the main window
	LRESULT CALLBACK WndProc(
		HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam )
	{
		HDC hdc;
		PAINTSTRUCT ps;
		HPEN hPen = CreatePen( PS_SOLID, 1, RGB( 0, 0, 0xFF ) );      // 論理ペンへのハンドル
		HBRUSH hBrush;

		if( hstgrm.quit_ )
		{
			PostQuitMessage( 0 );
			return 0;
		}

		switch( msg )
		{
		case WM_PAINT:
			hdc = BeginPaint( hWnd, std::addressof( ps ) );

			// ストックブラシを取得，登録
			hBrush = ( HBRUSH )GetStockObject( BLACK_BRUSH );
			SelectObject( hdc, hBrush );
			SelectObject( hdc, hPen );

			for( int i = 0; i < hstgrm.histgram_.size(); ++i )
			{
				Rectangle( hdc, 20 + i * ( 580 / ( ( std::max )( 1, ( int )hstgrm.histgram_.size() ) ) ), \
					400 - (int)( 0.0015 * hstgrm.histgram_[ i ] ), 20 + ( i + 1 ) * ( 580 / ( ( std::max )( 1, ( int )hstgrm.histgram_.size() ) ) ) - 1, 400 );
			}
			EndPaint( hWnd, std::addressof( ps ) );
			return 0;
		case WM_LBUTTONDOWN:// マウスの左ボタンが押されたときに送られてくる
			//pack.update_ = true;
			//hstgrm.quit_ = true;
			return 0;
		case WM_DESTROY:
			PostQuitMessage( 0 );
			return 0;
		case WM_TIMER:
			InvalidateRect( hWnd, NULL, TRUE );
			return 0;
		
		}

		return DefWindowProc( hWnd, msg, wParam, lParam );
	}

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
				cout << "mouse Left button holding......." << endl;
				mouse->flag_ = false;
				mouse->x1_ = x;
				mouse->y1_ = y;
				// When Left button is pressed, ...
				break;

			case CV_EVENT_LBUTTONUP:
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
				cout << "mouse Left button released......" << endl;
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
	}

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

	int window_manager( int & end_sign )
	{
		HWND hWnd;
		WNDCLASS wc;
		MSG msg;
		BOOL bRet;
		int nCmdShow = 10;
		HINSTANCE hInstance = GetModuleHandle( 0 );
		wc.style = CS_HREDRAW | CS_VREDRAW;
		wc.lpfnWndProc = WndProc;
		wc.cbClsExtra = 0;
		wc.cbWndExtra = 0;
		wc.hInstance = hInstance;
		wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
		wc.hCursor = LoadCursor( NULL, IDC_ARROW );
		wc.hbrBackground = ( HBRUSH )( COLOR_WINDOW + 1 );
		wc.lpszMenuName = NULL;
		wc.lpszClassName = szWndClass;

		if( !RegisterClass( std::addressof( wc ) ) ) return FALSE;

		hWnd = CreateWindow(
			szWndClass,
			_T( "Histgram" ),
			WS_OVERLAPPEDWINDOW,
			CW_USEDEFAULT, CW_USEDEFAULT,
			640, 480, //縦横
			NULL,
			NULL,
			hInstance,
			NULL );

		if( !hWnd ) return FALSE;

		ShowWindow( hWnd, nCmdShow );
		UpdateWindow( hWnd );

		int const TIMER_ID = 100;
		int const TIMER_ELAPSE = 100; // 更新間隔
		SetTimer( hWnd, TIMER_ID, TIMER_ELAPSE, NULL );
		while( bRet = GetMessage( std::addressof( msg ), NULL, 0, 0 ) && end_sign == 0 )
		{
			if( bRet == -1 )
			{
				return FALSE;
			}
			
		
			DispatchMessage( std::addressof( msg ) );
		}
	}


	//mouse_infoは別スレッドで書き換えられるので注意
	void kinect_thread( Runtime & runtime, int & go_sign, int & end_sign, int & ready_sign, mouse_info const & mi )
	{
		using namespace std;
		//go_signをオフにするのはこっち. readyをオフにするのはメイン

		int count = 0;
		char prev_frame[ 640 * 480 ];

		queue< cv::Ptr< IplImage > > image_queue;

		bool rect_init = false;

		cv::Ptr< IplImage > depth_image = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );
		cv::Ptr< IplImage > color_320x240 = cvCreateImage( cvSize( 320, 240 ), IPL_DEPTH_8U, 4 );
		cout << "test" << endl;

		// カメラデータの取得用フレームオブジェクト
		NUI_IMAGE_FRAME image_frame_depth_;
		NUI_IMAGE_FRAME image_frame_color_;

		NUI_IMAGE_FRAME * image_frame_depth = & image_frame_depth_;
		NUI_IMAGE_FRAME * image_frame_color = & image_frame_color_;

		
	std::vector< int > v( 128, 0 );

	for( auto it = v.begin(); it != v.end(); ++it )
	{
		*it = rand() % 256;
	}

	//update_graph( v )

	while( end_sign == 0 )
	{

		while( go_sign == 1 )
		{
			++count;
			std::cout << "foo" << std::endl;
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
				if( auto rect = std::move( get_image( image_frame_color_, "COLOR" ) ) )
				{
					// データのコピーと表示
					memcpy( runtime.color_.image_->imageData, (BYTE*)rect->pBits, \
						runtime.color_.image_->widthStep * runtime.color_.image_->height );

					//左右反転
					cvFlip( runtime.color_.image_, runtime.color_.image_, 1 );

					cvSetImageROI( runtime.color_.image_, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );

					::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );
					cvResetImageROI( runtime.color_.image_ );
				}
				// 画像データの取得
				if( auto rect = std::move( get_image( image_frame_depth_, "DEPTH" ) ) )
				{
					// データのコピーと表示
					memcpy( depth_image->imageData, static_cast< BYTE * >( rect->pBits ), \
						depth_image->widthStep * depth_image->height );
					cvFlip( depth_image, depth_image, 1 );

					cvSetImageROI( depth_image, cvRect( mi.x1_, mi.y1_, mi.x2_ - mi.x1_, mi.y2_ - mi.y1_ ) );


					if( runtime.id_ == hstgrm_kinect_id )
					{
						std::fill( hstgrm.buf_.begin(), hstgrm.buf_.end(), 0 );

						//ヒストグラムを作成する
						for( int x = 0; x < depth_image->width; ++x )
						{
							for( int y = 0; y < depth_image->height; ++y )
							{
								auto value = ( ( ( reinterpret_cast< UINT16 * >( depth_image->imageData + y * depth_image->widthStep ) )[ x ] ) / 8 ) / 40;
								if( value >= 128 )
									std::cout << value << std::endl;
								else
									++hstgrm.buf_[ value ];
							}
						}
					}
					std::copy( hstgrm.buf_.cbegin(), hstgrm.buf_.cend(), hstgrm.histgram_.begin() );


					::cvShowImage( runtime.depth_.window_name_.c_str(), depth_image );
					cvResetImageROI( depth_image );//なくても良いかも
					
					
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
		/*if( bRet == -1 )
		{
			break;
		}*/
		Sleep( 2 );
	}

	depth_image.release();
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

	void draw()
	{
		using namespace std;

		auto const current_time = generate_current_day_and_time();

		ofstream dlog( "debug_log_" + current_time + ".txt" );
		//解像度の設定
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;
		
		
		// アクティブなKinectの数を取得する
		int kinect_count = 0;
		::NuiGetSensorCount( std::addressof( kinect_count ) );

		auto const list = get_rect_to_draw( "area.txt" );

		if( list.size() != static_cast< std::size_t >( kinect_count ) )
		{
			std::cout << "Kinectの数と設定ファイルに書かれたKinectの数が不一致です．" << std::endl;
			return;
		}

		// Kinectのインスタンスを生成する
		typedef std::vector< Runtime > Runtimes;
		Runtimes runtime( kinect_count );
		//runtime.push_back( Runtime() );
		
		hstgrm.histgram_.resize( 128, 0 );
		hstgrm.buf_.resize( 128, 0 );
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


		for( int i = 0; i < kinect_count; ++i )	
		{
			mouse[ i ].event_ = false;

			mouse[ i ].x1_ = list[ i ].x_;
			mouse[ i ].x2_ = list[ i ].x_ + list[ i ].width_;
			mouse[ i ].y1_ = list[ i ].y_;
			mouse[ i ].y2_ = list[ i ].y_ + list[ i ].height_;

			runtime[ i ].color_.buf_ = cvCreateImage( cvSize( mouse[ i ].x2_ - mouse[ i ].x1_, \
				mouse[ i ].y2_ - mouse[ i ].y1_ ),IPL_DEPTH_8U, 4 );
			runtime[ i ].depth_.buf_ = cvCreateImage( cvSize( mouse[ i ].x2_ - mouse[ i ].x1_, \
				mouse[ i ].y2_ - mouse[ i ].y1_ ),IPL_DEPTH_16U, 1 );
		}

		for( int i = 0; i < kinect_count; ++i )
		{
			mouse[ i ].event_ = true;

			runtime[ i ].id_ = i;
			
			kinect_thread_obj[ i ] = thread( kinect_thread, \
				ref( runtime[ i ] ), ref( go_sign[ i ] ),ref( end_sign[ i ] ), \
				ref( ready_sign[ i ] ), ref( mouse[ i ] ) );
		}
		std::thread hstgrm_window = std::thread( window_manager, std::ref( end_sign[ 0 ] ) );

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

				if( input == "0" )
				{
					hstgrm_kinect_id = 0;
				}
				if( input == "1" )
				{
					hstgrm_kinect_id = 1;
				}
				if( input == "2" )
				{
					hstgrm_kinect_id = 2;
				}
				if( input == "3" )
				{
					hstgrm_kinect_id = 3;
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
		hstgrm_window.join();

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