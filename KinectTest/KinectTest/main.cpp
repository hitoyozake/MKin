// ������Kinect�̃J�����摜��\������
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/lexical_cast.hpp>

// NuiApi.h�̑O��Windows.h���C���N���[�h����
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define NO_MINMAX

struct Runtime
{
	INuiSensor *        kinect;         // Kinect�̃C���X�^���X

	struct image_data
	{
		HANDLE event_;	//�f�[�^�X�V�C�x���g�n���h��
		HANDLE stream_handle_;  //�摜�f�[�^�̃n���h��
		std::string window_name_;
		cv::Ptr< IplImage > image_; //�\���f�[�^
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

		// �E�B���h�E�����쐬
		runtime[i].color_.window_name_= "MultiKinect[" + boost::lexical_cast< string >\
			( i + 1 ) + "] Color";

		// ��ʃT�C�Y���擾
		DWORD x = 0, y = 0;
		::NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_640x480, x, y );			

		// OpenCV�̏����ݒ�
		runtime[i].color_.image_ = ::cvCreateImage( cvSize( x, y ), IPL_DEPTH_8U, 4 );
		::cvNamedWindow( runtime[ i ].color_.window_name_.c_str() );


		//�[�x==============================================================
		runtime[i].kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
			0, 2, runtime[ i ].depth_.image_, & runtime[ i ].depth_.stream_handle_ );

		// �E�B���h�E�����쐬
		runtime[ i ].depth_.window_name_ = "MultiKinect[" + boost::lexical_cast< string >\
			( i + 1 ) + "] Depth";

		// ��ʃT�C�Y���擾
		::NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION_640x480, x, y );			

		// OpenCV�̏����ݒ�
		runtime[i].depth_.image_ = ::cvCreateImage( cvSize( x, y ), IPL_DEPTH_16U, 1 );
		::cvNamedWindow( runtime[ i ].depth_.window_name_.c_str() );
	}
}


boost::optional< NUI_LOCKED_RECT > get_image( NUI_IMAGE_FRAME const & image_frame, std::string const & kind )
	// �摜�f�[�^�̎擾
{
	NUI_LOCKED_RECT rect;
	auto hRes = image_frame.pFrameTexture->LockRect( 0, & rect, 0, 0 );

	if(hRes != S_OK){
		printf(" ERR: %s�t���[���o�b�t�@���b�N���s. res=%d.", kind.c_str(), hRes );
		return boost::none;
	}
	else
		return boost::optional< NUI_LOCKED_RECT >( std::move( rect ) );
}


void draw()
{
	try {
		NUI_IMAGE_RESOLUTION const resolution = NUI_IMAGE_RESOLUTION_640x480;

		// �A�N�e�B�u��Kinect�̐����擾����
		int kinectCount = 0;
		::NuiGetSensorCount( &kinectCount );

		// Kinect�̃C���X�^���X�𐶐�����
		typedef std::vector< Runtime > Runtimes;
		Runtimes runtime( kinectCount );

		init( runtime );

		bool continue_flag = true;
		int count = 0;

		long now_angle = 0;

		string input;

		//cv::VideoWriter video_writer( "capture.avi",  CV_FOURCC('F', 'L', 'V', '1'), 30,  cvSize( 640, 480 ) );
		

		while ( continue_flag )
		{
			for( size_t i = 0; i < runtime.size(); ++i )
			{
				// �f�[�^�̍X�V��҂�
				::WaitForSingleObject( runtime[ i ].color_.stream_handle_, 300 );
				::WaitForSingleObject( runtime[ i ].depth_.stream_handle_, 300 );

				// �J�����f�[�^�̎擾
				NUI_IMAGE_FRAME image_frame_depth_;
				NUI_IMAGE_FRAME image_frame_color_;

				NUI_IMAGE_FRAME * image_frame_depth = & image_frame_depth_;
				NUI_IMAGE_FRAME * image_frame_color = & image_frame_color_;

				{
					auto hRes = runtime[ i ].kinect->NuiImageStreamGetNextFrame( runtime[ i ].depth_.stream_handle_, 0, image_frame_depth );
					if( hRes != S_OK ){
						printf(" ERR: DEPTH���t���[���擾���s. NuiImageStreamGetNextFrame() returns %d.", hRes);
						continue;
					}
				}
				{
					auto hRes = runtime[ i ].kinect->NuiImageStreamGetNextFrame( runtime[ i ].color_.stream_handle_, 0, image_frame_color );
					if( hRes != S_OK ){
						printf(" ERR: COLOR���t���[���擾���s. NuiImageStreamGetNextFrame() returns %d.", hRes );
						continue;
					}
				}
				// �摜�f�[�^�̎擾
				if( auto rect = get_image( image_frame_color_, "COLOR" ) )
				{
					// �f�[�^�̃R�s�[�ƕ\��
					memcpy( runtime[ i ].color_.image_->imageData, (BYTE*)rect->pBits, \
						runtime[ i ].color_.image_->widthStep * runtime[ i ].color_.image_->height );
					::cvShowImage( runtime[ i ].color_.window_name_.c_str(), runtime[ i ].color_.image_ );
				}
				// �摜�f�[�^�̎擾
				if( auto rect = get_image( image_frame_depth_, "DEPTH" ) )
				{
					// �f�[�^�̃R�s�[�ƕ\��
					memcpy( runtime[ i ].depth_.image_->imageData, (BYTE*)rect->pBits, \
						runtime[ i ].depth_.image_->widthStep * runtime[ i ].depth_.image_->height );
					::cvShowImage( runtime[ i ].depth_.window_name_.c_str(), runtime[ i ].depth_.image_ );
				}

				// �J�����f�[�^�̉��
				runtime[ i ].kinect->NuiImageStreamReleaseFrame( runtime[ i ].color_.stream_handle_, image_frame_color );
				runtime[ i ].kinect->NuiImageStreamReleaseFrame( runtime[ i ].depth_.stream_handle_, image_frame_depth );

				if( i == 0 )
				{
					//video_writer << cv::Mat( runtime[ i ].depth_.image_ );
					cvSaveImage( (boost::lexical_cast< string >( count ) + ".png" ).c_str(), runtime[ i ].depth_.image_ );
				}


				cout << "hoge" << ++count << endl;
				int key = ::cvWaitKey( 10 );
				if ( key == 'q' ) {
					continue_flag = false;
				}
					//���[�^����


			}
		}
		
		// �I������
		for( auto const & i : runtime )
		{
			//set_mortor( 10, * runtime[ 0 ].kinect );
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
	draw();
	return 0;
}