// 複数のKinectのカメラ画像を表示する
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <fstream>
#include <boost\shared_ptr.hpp>

#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>

#include <boost/lexical_cast.hpp>

// NuiApi.hの前にWindows.hをインクルードする
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define NO_MINMAX

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

void init( std::vector< Runtime > & runtime )
{
	using namespace std;
	NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

	for( size_t i = 0; i < runtime.size(); ++i )
	{
		NuiCreateSensorByIndex( i, & runtime[ i ].kinect_ );

		runtime[i].kinect_->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH );

		runtime[i].color_.event_ = ::CreateEvent( 0, TRUE, FALSE, 0 );
		runtime[i].depth_.event_ = ::CreateEvent( 0, TRUE, FALSE, 0 );

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
		::cvNamedWindow( runtime[ i ].color_.window_name_.c_str() );


		//深度==============================================================
		::NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_320x240, x, y );	

		runtime[i].kinect_->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_320x240,
			0, 2, runtime[ i ].depth_.image_, & runtime[ i ].depth_.stream_handle_ );
		// ウィンドウ名を作成
		runtime[ i ].depth_.window_name_ = "MultiKinect[" + boost::lexical_cast< string >\
			( i + 1 ) + "] Depth";

		// 画面サイズを取得

		// OpenCVの初期設定
		runtime[i].depth_.image_ = ::cvCreateImage( cvSize( x, y ), IPL_DEPTH_16U, 1 );
		::cvNamedWindow( runtime[ i ].depth_.window_name_.c_str() );
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
			Sleep( 300 );
		}
	}
}

void kinect_thread( Runtime & runtime, int & go_sign, int & end_sign, int & ready_sign )
{
	//go_signをオフにするのはこっち. readyをオフにするのはメイン
	
	int count = 0;
	char prev_frame[ 640 * 480 ];

	while( end_sign == 0 )
	{
		while( go_sign == 1 )
		{
			++count;

			go_sign = 0; 
			// データの更新を待つ
			::WaitForSingleObject( runtime.color_.stream_handle_, 100 );
			::WaitForSingleObject( runtime.depth_.stream_handle_, 100 );

			// カメラデータの取得
			NUI_IMAGE_FRAME image_frame_depth_;
			NUI_IMAGE_FRAME image_frame_color_;

			NUI_IMAGE_FRAME * image_frame_depth = & image_frame_depth_;
			NUI_IMAGE_FRAME * image_frame_color = & image_frame_color_;

			{
				auto hRes = runtime.kinect_->NuiImageStreamGetNextFrame( runtime.depth_.stream_handle_, 0, image_frame_depth );
				if( hRes != S_OK ){
					printf(" ERR: [%d]DEPTH%dフレーム取得失敗. NuiImageStreamGetNextFrame() returns %d.\n", runtime.id_, count, hRes);
					ready_sign = 1;
					continue;
				}
			}
			{
				auto hRes = runtime.kinect_->NuiImageStreamGetNextFrame( runtime.color_.stream_handle_, 0, image_frame_color );
				if( hRes != S_OK ){
					printf(" ERR: [%d]COLOR%dフレーム取得失敗. NuiImageStreamGetNextFrame() returns %d.\n", runtime.id_, count, hRes );
					ready_sign = 1;
					continue;
				}
			}
			// 画像データの取得
			if( auto rect = std::move( get_image( image_frame_color_, "COLOR" ) ) )
			{
				// データのコピーと表示
				memcpy( runtime.color_.image_->imageData, (BYTE*)rect->pBits, \
					runtime.color_.image_->widthStep * runtime.color_.image_->height );
				::cvShowImage( runtime.color_.window_name_.c_str(), runtime.color_.image_ );
				runtime.ofs_c_->write\
				( ( char * )rect->pBits, runtime.color_.image_->widthStep * runtime.color_.image_->height );
			}
			// 画像データの取得
			if( auto rect = std::move( get_image( image_frame_depth_, "DEPTH" ) ) )
			{
				// データのコピーと表示
				memcpy( runtime.depth_.image_->imageData, (BYTE*)rect->pBits, \
					runtime.depth_.image_->widthStep * runtime.depth_.image_->height );
				::cvShowImage( runtime.depth_.window_name_.c_str(), runtime.depth_.image_ );
				runtime.ofs_d_->write( ( char * )rect->pBits, runtime.depth_.image_->widthStep * runtime.depth_.image_->height );
			}

			// カメラデータの解放
			runtime.kinect_->NuiImageStreamReleaseFrame( runtime.color_.stream_handle_, image_frame_color );
			runtime.kinect_->NuiImageStreamReleaseFrame( runtime.depth_.stream_handle_, image_frame_depth );
			ready_sign = 1;

		}
		Sleep( 3 );
	}
}

void end_task( vector< int > & end_sign, vector< thread > & kinect_thread_obj )
{
	for( auto & it : end_sign )
		it = 1;
	for( auto & it : kinect_thread_obj )
		it.join();
}


void draw()
{
	using namespace std;

	ofstream dlog( "debugLog.txt" );

	NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

	// アクティブなKinectの数を取得する
	int kinect_count = 0;
	::NuiGetSensorCount( & kinect_count );

	// Kinectのインスタンスを生成する
	typedef std::vector< Runtime > Runtimes;
	Runtimes runtime( kinect_count );
	//runtime.push_back( Runtime() );
	init( runtime );

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
		string const filename_d = string( "depth_" ) + boost::lexical_cast< string >\
			( i ) + ".txt";
		string const filename_c = string( "color_" ) + boost::lexical_cast< string >\
			( i ) + ".txt";

		runtime[ i ].ofs_c_ = boost::shared_ptr< ofstream >( new ofstream() );
		runtime[ i ].ofs_d_ = boost::shared_ptr< ofstream >( new ofstream() );

		runtime[ i ].ofs_d_->open( filename_d, ios::binary );
		runtime[ i ].ofs_c_->open( filename_c, ios::binary );

		runtime[ i ].id_ = i;

		kinect_thread_obj[ i ] = thread( kinect_thread, \
			ref( runtime[ i ] ), ref( go_sign[ i ] ),ref( end_sign[ i ] ), \
			ref( ready_sign[ i ] ) );
	}

	//開始 ->find_ifで書き換え可能コード
	for( auto & i : go_sign )
		i = 1;

	boost::timer timer;
	while ( continue_flag )
	{
		int flag = 1;

		for( auto const i : ready_sign )
		{
			flag = flag & i;
		}

		if( flag == 1 )
		{
			dlog << timer.elapsed() << endl;
			timer.restart();
			//全部待機済みなので、次のフレーム
			for( auto & i : ready_sign )
				i = 0;
			for( auto & i : go_sign )
				i = 1;
			Sleep( 20 );
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
	}

	timer.elapsed();
	end_task( end_sign, kinect_thread_obj );
	//スレッドの終了待ち
	input_wait_thread.join();

	// 終了処理
	for( auto const & i : runtime )
	{
		//set_mortor( 10, * runtime[ 0 ].kinect );
		i.kinect_->NuiShutdown();	
	}
	//Windowを閉じる
	::cvDestroyAllWindows();

}

int main()
{
	draw();
	return 0;
}