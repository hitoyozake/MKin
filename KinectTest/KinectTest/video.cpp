#include "video.h"
#pragma comment(lib, "vfw32.lib")

namespace video
{
	void vfw_manager::reset()
	{
		if( ! closed_ )
		{
			close();
		}

		avi_stream_ = nullptr;
		avi_tmp_stream_ = nullptr;
		closed_ = true;
	}

	vfw_manager::vfw_manager( std::string const & filename, std::string const & title, int const width, \
		int const height, int const time_scale, int const frame_rate,
		int const total_frame_count ) : frame_count_( 0 ), total_frames_( total_frame_count ), closed_( true )
	{
		AVIFileInit();

		using namespace std;
		{
			AVISTREAMINFO si =
			{
				streamtypeVIDEO,
				comptypeDIB,
				0,                  /* ストリームフラグ: なし   */
				0,                  /* 機能フラグ: 未使用       */
				0,                  /* ストリームの優先順位     */
				0,                  /* ストリームの言語         */
				time_scale,     /* ストリームの時間単位[秒]: TIME=1秒   */
				frame_rate,     /* ストリームのレート[frames/TIME]*/
				0,                  /* 最初のサンプル番号[frame]*/
				total_frames_,  /* ストリームの長さ[frames] */
				0,                  /* オーディオスキュー[秒]   */
				0,                  /* バッファサイズ           */
				(DWORD)-1,          /* 品質[-1, 0～10000]: デフォルト   */
				0,                  /* サンプルのサイズ[byte]: 可変     */
				{ 0, 0, width, height },     /* 表示領域 */
				0,                  /* ファイルを編集した回数   */
				0,                  /* フォーマットを変更した回数   */
				WCHAR( '\0' )
			};

			asi_ = std::move( si );
		}
		fill( asi_.szName, & asi_.szName[ 64 ], 0 );

		int const title_length = std::max< int >( 63, static_cast< int >( title.length()) );

		title.copy( asi_.szName, title_length, 0 );	

		{
			BITMAPINFOHEADER bmih = 
			{
				sizeof(BITMAPINFOHEADER),   /* パレットを使用しないなら構造体の大きさ */
				width,      /* 幅(RGBの場合px)              */
				height,     /* 高さ(RGBの場合px)            */
				1,          /* 面の数(必ず1)                */
				32,         /* 1ピクセルあたりのビット数    */
				BI_RGB,     /* 圧縮方法: 非圧縮24bit        */
				width * height *4,  /* 画像の大きさ[byte]           */
				0,          /* x方向の解像度[px/m]          */
				0,          /* y方向の解像度[px/m]          */
				0,          /* カラーテーブルで使用する色数 */
				0           /* カラーテーブル中で重要な色の数 */
			};

			bmp_ih_ = std::move( bmih );

			//圧縮設定
			memset( addressof( comp_vars_ ), 0, sizeof( COMPVARS ) );
			comp_vars_.cbSize = sizeof( COMPVARS );
			comp_vars_.dwFlags = ICMF_COMPVARS_VALID;
			comp_vars_.fccHandler = comptypeDIB;
			comp_vars_.lQ = ICQUALITY_DEFAULT;

			auto tmp = comp_vars_;

			if( false && !ICCompressorChoose( NULL, ICMF_CHOOSE_DATARATE \
				| ICMF_CHOOSE_KEYFRAME,\
				addressof( bmp_ih_ ), NULL, addressof( comp_vars_ ),\
				NULL ) )
			{
				return; //エラー?
			}

			auto show_data = [ this ]()
			{
				cout << comp_vars_.fccHandler << endl;
				cout << comp_vars_.lDataRate << endl;
				cout << comp_vars_.lKey << endl;
				cout << comp_vars_.cbState << endl;
				cout << comp_vars_.lpState << endl;	//現在使われていない?
				cout << comp_vars_.lQ << endl;

				cout << comp_vars_.lpbiIn << endl;

			};

			//show_data();
		}

		int const fcc = 1668707181;
		int const lQ = 7500; //75%
		asi_.fccHandler = fcc;//comp_vars_.fccHandler;
		opt_.fccType = streamtypeVIDEO;
		opt_.fccHandler = fcc;//comp_vars_.fccHandler;
		opt_.dwKeyFrameEvery = comp_vars_.lKey;
		opt_.dwQuality = lQ;//comp_vars_.lQ;
		opt_.dwBytesPerSecond = 0;//comp_vars_.lDataRate;
		opt_.dwFlags = 0;
		/*( comp_vars_.lDataRate > 0 ? \
		AVICOMPRESSF_DATARATE : 0 ) |
		( comp_vars_.lKey > 0 ? AVICOMPRESSF_KEYFRAMES : 0 );
		*/
		opt_.lpFormat = NULL;
		opt_.cbFormat = 0;
		opt_.lpParms = 0;//comp_vars_.lpState;
		opt_.cbParms = 0;//comp_vars_.cbState;

		//AVIファイルを開く
		if( AVIFileOpen( addressof( pavi_file_ ), filename.c_str(), \
			OF_CREATE | OF_WRITE | OF_SHARE_DENY_WRITE, NULL ) != 0 )
		{
			cout << "can not open AVI FILE" << endl;
			return;
		}

		if( AVIFileCreateStream( pavi_file_,\
			addressof( avi_stream_ ), addressof( asi_ ) ) != 0 )
		{
			cout << "AVIFileCreateStream Failed" << endl;
		}

		if( AVIMakeCompressedStream(\
			addressof( avi_tmp_stream_ ), avi_stream_, addressof( opt_ ), NULL ) != 0 )
		{
			cout << "AVIMakeCompressedStream FAILED" << endl;
			return;
		}

		if( AVIStreamSetFormat( avi_tmp_stream_, 0, & bmp_ih_, \
			sizeof( BITMAPINFOHEADER ) ) != 0 )
		{
			cout << "AVIStreamSetFormat FAILED" << endl;
			return;
		}
		width_ = width;
		height_ = height;
		image_.resize( width * height );

		closed_ = false;
	}

	bool vfw_manager::write( bool const reverse, cv::Ptr< IplImage > image )
	{
		using namespace std;

		if( total_frames_ <= frame_count_ )
			return false;

		for( int y = 0; y < height_; ++y )
		{
			tagRGBQUAD * line = nullptr;

			if( ! reverse )
			{
				line = addressof( image_ [ width_ * y ] );
			}
			else
			{
				line = addressof( image_ [ width_ * ( height_ - y - 1 ) ] );
			}

			for( int x = 0; x < width_; ++x )
			{
				memcpy( & line[ x ].rgbBlue, & image->imageData[ x * image->nChannels + y * image->widthStep ],
					3 );
				//int const pixel = x * image->nChannels + y * image->widthStep;
				//line[ x ].rgbBlue = image->imageData[ pixel ];
				//line[ x ].rgbGreen = image->imageData[ pixel + 1 ];
				//line[ x ].rgbRed = image->imageData[ pixel + 2 ];

				//line[ x ].rgbBlue = buf[ x + y * width_ ].b_;
				//line[ x ].rgbGreen = buf[ x + y * width_ ].g_;
				//line[ x ].rgbRed = buf[ x + y * width_ ].r_;
				line[ x ].rgbReserved = 0;
			}
		}

		if( AVIStreamWrite( \
			avi_tmp_stream_,
			frame_count_,
			1,
			( void * )( addressof( image_[ 0 ] ) ), \
			height_ * width_ * 4, \
			AVIIF_KEYFRAME, //keyフレーム設定、いじったほうが容量節約
			NULL, \
			NULL ) != 0 )
		{
			return false;
		}
		++frame_count_;
		return true;
	}
	vfw_manager::~vfw_manager()
	{
		if( ! closed_ )
		{
			close();
		}
	}

	void vfw_manager::close()
	{
		if( avi_tmp_stream_ )
		{
			AVIStreamRelease( avi_tmp_stream_ );
			avi_tmp_stream_ = nullptr;
		}
		if( avi_stream_ )
		{
			AVIStreamRelease( avi_stream_ );
			avi_stream_ = nullptr;
		}
		if( pavi_file_ )
		{
			AVIFileRelease( pavi_file_ );
			pavi_file_ = nullptr;
		}

		ICCompressorFree( std::addressof( comp_vars_ ) );
		AVIFileExit();

		closed_ = true;
		std::cout << "AVI WAS SAVED" << std::endl;
	}
}