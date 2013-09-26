#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>


#include <windows.h>
#include <windowsX.h>
#include <dsound.h>


#pragma comment(lib, "winmm.lib")

#pragma comment(lib, "dsound.lib")
#pragma comment(lib, "dxguid.lib")
#define	RIFFCC( _x )	(((_x >> 24) & 0xFF) + ((_x >> 8) & 0xFF00) + ((_x << 8) & 0xFF0000) + ((_x << 24) & 0xFF000000))

namespace sound_recorder
{
	class sound_recorder
	{
	public:
		void open_file();

		void stop_recording();

		sound_recorder( std::string const & filename ) : recording_( false ), reading_( false ), \
			filename_( filename )
		{
			put_pos_ = 0;
			data_len_ = 0;
			cap_buf_size_ = 10;
			data_buff_ = nullptr;

		}

	private:
		std::string filename_;
		int save_open_wave_file( std::string const & filename, WAVEFORMATEX & wf, bool const over_write );
		int save_wave_file_header();
		bool open_sound_cap_device( unsigned int const dev_list_index, WAVEFORMATEX & wf );
		void close_wave_file();
		DWORD write_wave_file( void * data, DWORD const size );
		void capture_event_task( bool & flag );
		DWORD put_capture_data( BYTE * buf, DWORD const len, DWORD const pos );
		DWORD writed_wavefile( void * data, DWORD const size );

		struct  wave_file_header{
			DWORD		RiffId;						//!< RIFF�t�@�C�����ʎq (RIFF).
			DWORD		FileSize;					//!< �t�@�C���T�C�Y - 8.
			DWORD		FileType;					//!< �t�@�C���^�C�v ("WAVE").
			DWORD		FormatId;					//!< �t�H�[�}�b�g���ʎq ("fmt ").
			DWORD		FormatSize;					//!< �t�H�[�}�b�g�E�T�C�Y - 8.
			WORD		FormatType;					//!< �t�H�[�}�b�g�^�C�v.
			WORD		Channels;					//!< �`�����l����.
			DWORD		SamplesPerSec;				//!< �T���v�����O����.
			DWORD		AvgBytesPerSec;				//!< 1�b������̃o�C�g��.
			WORD		BlockAlign;					//!< 1�`�����l���̃o�C�g��.
			WORD		BitsPerSample;				//!< 1�f�[�^������̃r�b�g��.
		};

		struct wave_fact_chunk{
			DWORD		Id;							//!< �f�[�^���ʎq("fact").
			DWORD		Size;						//!< �`�����N�T�C�Y - 8.
			DWORD		Samples;					//!< �S�T���v����.
		};

		bool recording_;
		bool reading_;
		HANDLE file_handle_;
		DWORD cap_buf_size_;

		static std::size_t const dev_num = 2;
		static std::size_t const dev_table_num = 10;
		static int const BUFSIZE = 19200;


		LPDIRECTSOUNDCAPTURE8 cap_dev_;
		LPDIRECTSOUNDCAPTUREBUFFER cap_buf_;
		HANDLE cap_event_[ dev_num ];

		boost::thread thread_;
		wave_file_header wave_file_header_;
		DWORD data_len_; //wav�f�[�^�̃o�C�g��
		DWORD put_pos_;
		BYTE * data_buff_;

	public:
		static GUID dev_guid_[ dev_num ][ dev_table_num ];	//���o�̓f�o�C�X��GUID�e�[�u��
		static char dev_name_[dev_num ][ dev_table_num ][ 128 ];	//���O�e�[�u��
		static DWORD dev_cnt_[ 2 ];

	};
}