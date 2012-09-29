// 複数のKinectのカメラ画像を表示する
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/optional.hpp>

#include <boost/lexical_cast.hpp>

// NuiApi.hの前にWindows.hをインクルードする
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>

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

void set_mortor( long const angle, INuiSensor & kinect )
{
	long const max_angle = 30;
	long const min_angle = -30;		
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
		NuiCreateSensorByIndex( i, & runtime[ i ].kinect );

		runtime[i].kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH );

		runtime[i].color_.event_ = ::CreateEvent( 0, TRUE, FALSE, 0 );
		runtime[i].depth_.event_ = ::CreateEvent( 0, TRUE, FALSE, 0 );

		//Color=============================================================
		runtime[i].kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, resolution,
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
		runtime[i].kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
			0, 2, runtime[ i ].depth_.image_, & runtime[ i ].depth_.stream_handle_ );

		// ウィンドウ名を作成
		runtime[ i ].depth_.window_name_ = "MultiKinect[" + boost::lexical_cast< string >\
			( i + 1 ) + "] Depth";

		// 画面サイズを取得
		::NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_640x480, x, y );			

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


void depth()
{
	try {
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

		// アクティブなKinectの数を取得する
		int kinectCount = 0;
		::NuiGetSensorCount( &kinectCount );

		// Kinectのインスタンスを生成する
		typedef std::vector< Runtime > Runtimes;
		Runtimes runtime( kinectCount );

		init( runtime );

		bool continue_flag = true;
		int count = 0;
		while ( continue_flag ) {
			for ( Runtimes::iterator it = runtime.begin(); it != runtime.end(); ++it ) {
				// データの更新を待つ
				::WaitForSingleObject( it->color_.stream_handle_, 300 );
				::WaitForSingleObject( it->depth_.stream_handle_, 300 );

				// カメラデータの取得
				NUI_IMAGE_FRAME image_frame_depth_;
				NUI_IMAGE_FRAME image_frame_color_;

				NUI_IMAGE_FRAME * image_frame_depth = & image_frame_depth_;
				NUI_IMAGE_FRAME * image_frame_color = & image_frame_color_;

				{
					auto hRes = it->kinect->NuiImageStreamGetNextFrame( it->depth_.stream_handle_, 0, image_frame_depth );
					if( hRes != S_OK ){
						printf(" ERR: DEPTH次フレーム取得失敗. NuiImageStreamGetNextFrame() returns %d.", hRes);
						continue;
					}
				}
				{
					auto hRes = it->kinect->NuiImageStreamGetNextFrame( it->color_.stream_handle_, 0, image_frame_color );
					if( hRes != S_OK ){
						printf(" ERR: COLOR次フレーム取得失敗. NuiImageStreamGetNextFrame() returns %d.", hRes );
						continue;
					}
				}
				// 画像データの取得
				if( auto rect = get_image( image_frame_color_, "COLOR" ) )
				{
					// データのコピーと表示
					memcpy( it->color_.image_->imageData, (BYTE*)rect->pBits, it->color_.image_->widthStep * it->color_.image_->height );
					::cvShowImage( it->color_.window_name_.c_str(), it->color_.image_ );
				}
				// 画像データの取得
				if( auto rect = get_image( image_frame_depth_, "DEPTH" ) )
				{
					// データのコピーと表示
					memcpy( it->depth_.image_->imageData, (BYTE*)rect->pBits, it->depth_.image_->widthStep * it->depth_.image_->height );
					::cvShowImage( it->depth_.window_name_.c_str(), it->depth_.image_ );
				}

				// カメラデータの解放
				it->kinect->NuiImageStreamReleaseFrame( it->color_.stream_handle_, image_frame_color );
				it->kinect->NuiImageStreamReleaseFrame( it->depth_.stream_handle_, image_frame_depth );

				cout << "hoge" << ++count << endl;
				int key = ::cvWaitKey( 10 );
				if ( key == 'q' ) {
					continue_flag = false;
				}
			}
		}

		// 終了処理
		for( auto const & i : runtime )
		{
			i.kinect->NuiShutdown();
			
		}

		::cvDestroyAllWindows();
	}
	catch ( std::exception & ex ) {
		std::cout << ex.what() << std::endl;
	}
}

int main()
{
	depth();
	return 0;
}