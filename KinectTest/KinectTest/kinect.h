#pragma once

#include "precompiled.h"

namespace kinect
{
	class kinect_manager
	{
	public:
		kinect_manager() : id_( -1 )
		{}

		bool is_near_mode() const;
		void set_near_mode();
		void unset_near_mode();

	private:
		boost::shared_ptr< std::ofstream > ofs_d_;
		boost::shared_ptr< std::ofstream > ofs_c_;

		int id_;

		struct kinect
		{
			struct image_receiver
			{
				HANDLE event_;	//�f�[�^�X�V�C�x���g�n���h��
				HANDLE stream_handle_;  //�摜�f�[�^�̃n���h��
				cv::Ptr< IplImage > image_; //�\���f�[�^

				std::string id_str_;	//openCV��WindowName�ȂǂɎg��
			};

			INuiSensor * device_;         // Kinect�̃C���X�^���X
			image_receiver color_;
			image_receiver depth_;

			bool near_mode_;

			static int const KINECT_NUI_INIT_FLAG_DEFAULT = \
				NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH;

			void init_stream( NUI_IMAGE_TYPE nui_image_type, \
				NUI_IMAGE_RESOLUTION const resolution, HANDLE & image, HANDLE & stream_handle, bool const create_window, bool is_color );

			//recommended to use KINECT_NUI_FLAG_DEFAULT
			void init( int const index, int const nui_init_flags );

			kinect()
			{
			}
		};

		kinect kinect_;
	};
}
