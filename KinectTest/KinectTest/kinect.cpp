#include "kinect.h"

namespace kinect
{
	boost::optional< NUI_LOCKED_RECT > kinect_manager::get_image( NUI_IMAGE_FRAME const & \
		image_frame )
	{
		NUI_LOCKED_RECT rect;

		return ( image_frame.pFrameTexture->LockRect( \
			0, std::addressof( rect ), 0, 0 ) != S_OK ) ? \
			boost::none : boost::optional\
			< decltype( rect ) >( std::move( rect ) );
	}

	void kinect_manager::write_flag( bool const flag )
	{
		write_flag_ = flag;
	}


	cv::Ptr< IplImage > kinect_manager::get_color_image_from_pre_frame() const
	{
		return kinect_.color_.image_;
	}

	cv::Ptr< IplImage > kinect_manager::get_depth_image_from_pre_frame() const
	{
		return kinect_.depth_.image_;
	}

	//true = 深度画像・色画像ともに更新完了
	//
	bool kinect_manager::update_frame( bool const show_debug_message )
	{
		using namespace std;

		//深度画像の取得**************************************
		NUI_IMAGE_FRAME image_frame_depth;

		{
			auto const result = kinect_.device_->\
				NuiImageStreamGetNextFrame( kinect_.depth_.stream_handle_, \
				0, addressof( image_frame_depth ) );

			if( result != S_OK )
			{
				if( show_debug_message )
				{
					cout << "ERROR : CAN NOT GET NEXT FRAME[DEPTH]" << endl; 
				}
				return false;
			}
		}

		//色画像の取得***************************************
		NUI_IMAGE_FRAME image_frame_color;

		{
			auto const result = kinect_.device_->\
				NuiImageStreamGetNextFrame( kinect_.color_.stream_handle_, \
				0, addressof( image_frame_color ) );

			if( result != S_OK )
			{
				if( show_debug_message )
				{
					cout << "ERROR : CAN NOT GET NEXT FRAME[COLOR]" << endl;
				}
				return false;
			}
		}
		
		//画像を取り出してメンバ変数に格納****************************************
		if( auto const color_image = std::move( get_image( image_frame_color ) ) )
		{
			memcpy( kinect_.color_.image_->imageData, static_cast< BYTE * >\
				( color_image->pBits ),\
				kinect_.color_.image_->widthStep * kinect_.color_.image_->height );	
		}
		if( auto const depth_image = std::move( get_image( image_frame_depth ) ) )
		{
			memcpy( kinect_.depth_.image_->imageData, static_cast< BYTE * >\
				( depth_image->pBits ),\
				kinect_.depth_.image_->widthStep * kinect_.depth_.image_->height );				
		}
		//*************************************************************
		return true;
	}


	void kinect_manager::kinect::init( int const index, DWORD const nui_init_flags, \
		bool const create_window )
	{
		using namespace std;
		
		NuiCreateSensorByIndex( index, addressof( device_ ) );

		device_->NuiInitialize( nui_init_flags );

		color_.event_ = CreateEvent( nullptr, TRUE, FALSE, nullptr );
		depth_.event_ = CreateEvent( nullptr, TRUE, FALSE, nullptr );
		
		{
			auto const color_resolution = \
				NUI_IMAGE_RESOLUTION_640x480;

			//colorの初期化
			init_stream( NUI_IMAGE_TYPE_COLOR, color_resolution,
				color_.image_, color_.stream_handle_, create_window, true );

		}
		{
			auto const depth_resolution = \
				NUI_IMAGE_RESOLUTION_640x480;

			//colorの初期化
			init_stream( NUI_IMAGE_TYPE_DEPTH, depth_resolution,
				depth_.image_, depth_.stream_handle_, create_window, false );
		}
	}

	void kinect_manager::kinect::init_stream( NUI_IMAGE_TYPE nui_image_type, \
				NUI_IMAGE_RESOLUTION const resolution, HANDLE image, HANDLE stream_handle, bool const create_window, bool const is_color )
	{
		device_->NuiImageStreamOpen( nui_image_type, resolution, \
			0, 2, image, std::addressof( stream_handle ) );

		if( create_window )
		{
			DWORD x = 0, y = 0;
			NuiImageResolutionToSize( resolution, x, y );

			int channel = 1;
			int ipl_depth = IPL_DEPTH_16U;

			if( is_color )
			{
				channel = 4;
				ipl_depth = IPL_DEPTH_8U;
			}

			image = cvCreateImage( cvSize( x, y ), \
				ipl_depth, channel );
		}
	}

	bool kinect_manager::is_near_mode() const{ return kinect_.near_mode_; }
	
	void kinect_manager::set_near_mode()
	{
		kinect_.near_mode_ = true;
		kinect_.device_->NuiImageStreamSetImageFrameFlags( \
			 kinect_.depth_.stream_handle_,
			 NUI_IMAGE_STREAM_FLAG_SUPPRESS_NO_FRAME_DATA //これで 無効フレーム抑制
			 | NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE
			 );

	}
	
	void  kinect_manager::unset_near_mode()
	{
		 kinect_.near_mode_ = false;

		 kinect_.device_->NuiImageStreamSetImageFrameFlags( \
			 kinect_.depth_.stream_handle_,
			 NUI_IMAGE_STREAM_FLAG_SUPPRESS_NO_FRAME_DATA //これで 無効フレーム抑制
			);
	}
	
	//kinect ctor
	kinect_manager::kinect::kinect() : \
		device_( nullptr ), near_mode_( false )
	{
	}
	
	//image_receiver ctor
	kinect_manager::kinect::image_receiver::image_receiver() :\
		event_( NULL ), stream_handle_( NULL ), image_( nullptr )
	{
	}
}