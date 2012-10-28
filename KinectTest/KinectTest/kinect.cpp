#include "kinect.h"

namespace kinect
{
	void kinect_manager::kinect::init( int const index, int const nui_init_flags )
	{
		using namespace std;
		
		NuiCreateSensorByIndex( index, addressof( device_ ) );

		device_->NuiInitialize( nui_init_flags );

		color_.event_ = CreateEvent( nullptr, TRUE, FALSE, nullptr );
		depth_.event_ = CreateEvent( nullptr, TRUE, FALSE, nullptr );

	}

	void kinect_manager::kinect::init_stream( NUI_IMAGE_TYPE nui_image_type, \
				NUI_IMAGE_RESOLUTION const resolution, HANDLE & image, HANDLE & stream_handle, bool const create_window, bool const is_color )
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
}