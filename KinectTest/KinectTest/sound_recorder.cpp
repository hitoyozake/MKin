#include "sound_recorder.h"


namespace sound_recorder
{

	GUID sound_recorder::dev_guid_[ dev_num ][ dev_table_num ];	//���o�̓f�o�C�X��GUID�e�[�u��
	char sound_recorder::dev_name_[dev_num ][ dev_table_num ][ 128 ];	//���O�e�[�u��
	DWORD sound_recorder::dev_cnt_[ 2 ];

	DWORD sound_recorder::put_capture_data( BYTE * buf, DWORD const len, DWORD const pos )
	{
		//�p�ӂ��Ă����o�b�t�@�����傫���ꍇ�͊m�ۂ��Ȃ���
		if( pos + len > cap_buf_size_ )
		{
			auto const wk_len = cap_buf_size_ - pos;
			memcpy( std::addressof( data_buff_[ pos ] ), buf, wk_len );

			//���̃f�[�^�̏������݊J�n�ʒu(�X�V�p)
			return len - wk_len;
		}
		else
		{
			memcpy( std::addressof( data_buff_[ pos ] ), buf, len );

			return pos + len >= cap_buf_size_ ? 0 : pos + len ;
		}
	}


	void sound_recorder::capture_event_task( bool & flag )
	{
		while( flag )
		{
			switch( WaitForMultipleObjects( 2, cap_event_, 0, INFINITE ) )
			{
			case WAIT_OBJECT_0 + 0:
			case WAIT_OBJECT_0 + 1:
				DWORD n = 0;
				DWORD read_pos = 0;
				DWORD dummy = 0;
				cap_buf_->GetCurrentPosition( std::addressof( dummy ), std::addressof( read_pos ) );

				//wav�t�@�C���ɏ����o���퓬�f�[�^�̈ʒu��ۑ�
				DWORD const wav_pos = put_pos_;
				if( put_pos_ > read_pos )
				{
					n = ( cap_buf_size_ + read_pos ) - put_pos_;
				}
				else
				{
					if( put_pos_ < read_pos )
					{
						n = read_pos - put_pos_;
					}
				}
				DWORD const len = ( n >= ( cap_buf_size_ >> 1 ) ) ? ( cap_buf_size_ >> 1 ) : n;
				//! �f�[�^��ǂݍ��ޗ̈�����b�N����.
				void * buf1 = nullptr;
				void * buf2 = nullptr;
				DWORD	len1 = 0;
				DWORD	len2 = 0;
				cap_buf_->Lock( put_pos_, len, &buf1, &len1, &buf2, &len2, 0 );

				//! �ǂݍ��݃o�b�t�@�ɃL���v�`�������f�[�^���R�s�[����.
				if( buf1 && len1 ){
					put_pos_ = put_capture_data( static_cast< BYTE * >( buf1 ), len1, put_pos_ );
				}
				//! �L���v�`���E�o�b�t�@�̐擪�ɖ߂����ꍇ�A�c��̃f�[�^���R�s�[����.
				if( buf2 && len2 ){
					put_pos_ = put_capture_data( static_cast< BYTE * >( buf2 ), len2, put_pos_ );
				}
				//! �ǂݏo���̊��������o�b�t�@���A�����b�N����.
				cap_buf_->Unlock( buf1, len1, buf2, len2 ); 


				if( wav_pos > put_pos_ )
				{
					//! �o�b�t�@�̍Ō�ɂ���f�[�^��WAV�t�@�C���ɏ�������.
					write_wave_file( std::addressof( data_buff_[wav_pos] ), cap_buf_size_ - wav_pos );

					//! �o�b�t�@�̐擪�ɂ���c��̃f�[�^��WAV�t�@�C���ɏ�������.
					if( put_pos_ > 0 )
					{
						write_wave_file( data_buff_, put_pos_ );
					}
				}
				else
				{
					//! �L���v�`�������f�[�^��WAV�t�@�C���ɏ�������.
					write_wave_file( std::addressof( data_buff_[ wav_pos ] ), put_pos_ - wav_pos );
				}

				break;
			}
		}
		std::cout << "end" << std::endl;
	}

	void sound_recorder::stop_recording()
	{
		recording_ = false;

		Sleep( 1000 );

		if( cap_dev_ || cap_dev_ )
		{
			//�L���v�`�������̒�~
			if( cap_buf_ )
			{
				cap_buf_->Stop();
			}

			//�C�x���g�Ď��^�X�N���~

			//DSound�̃L���v�`���p�I�u�W�F�N�g�̊J��
			if( cap_buf_ )
			{
				cap_buf_->Release();
				cap_buf_ = NULL;
			}

			if( cap_dev_ )
			{
				cap_dev_->Release();
				cap_dev_ = NULL;
			}

			for( int i = 0; i < dev_num; ++i )
			{
				if( cap_event_[ i ] )
				{
					CloseHandle( cap_event_[ i ] );
					cap_event_[ i ] = NULL;
				}
			}


		}

		close_wave_file();

		if( data_buff_ )
		{
			delete [] data_buff_;
			data_buff_ = nullptr;
		}
		thread_.join();
	}



	DWORD sound_recorder::write_wave_file( void * data, DWORD const size )
	{
		DWORD len = 0;

		if( file_handle_ )
		{
			//�t�@�C���Ƀf�[�^����������
			if( WriteFile( file_handle_, data, size, \
				std::addressof( len ), 
				NULL ) )
			{
				data_len_ += len;
				return len;
			}
		}
		return 0;
	}

	void sound_recorder::close_wave_file()
	{
		if( file_handle_ )
		{
			// wav�t�@�C���̏����o�����̏ꍇ��wav�t�@�C���̃w�b�_���X�V����
			if( ! reading_ )
			{
				save_wave_file_header();
			}
			//�I�[�v�����Ă���t�@�C�������
			CloseHandle( file_handle_ );
		}

		file_handle_ = NULL;

	}

	BOOL CALLBACK DSEnumCallback( LPGUID guid, LPCTSTR desc, LPCTSTR module, LPVOID user )
	{
		auto const idx = reinterpret_cast< DWORD >( user );
		auto const num = sound_recorder::dev_cnt_[ idx ];

		if( ! guid || ! desc )
		{
			return TRUE;
		}
		//�f�o�C�X��GUID��ۑ�
		sound_recorder::dev_guid_[ idx ][ num ] = * guid;

		//���𑝂₷
		++sound_recorder::dev_cnt_[ idx ];

		return TRUE;

	}

	bool sound_recorder::open_sound_cap_device( unsigned int const dev_list_index, WAVEFORMATEX & wf )
	{
		DirectSoundCaptureEnumerate( DSEnumCallback, ( void * ) 1 );

		if( DirectSoundCaptureCreate8( std::addressof( dev_guid_[ 1 ][ dev_list_index ] ), \
			std::addressof( cap_dev_ ), NULL ) != S_OK )
			return false;

		DSCBUFFERDESC desc;
		memset( std::addressof( desc ), 0, sizeof desc );
		desc.dwSize = sizeof desc;
		desc.dwFlags = 0;
		desc.dwBufferBytes = BUFSIZE;
		desc.lpwfxFormat = std::addressof( wf );

		if( cap_dev_->CreateCaptureBuffer( std::addressof( desc ), \
			std::addressof( cap_buf_ ), NULL ) != DS_OK )
			return false;

		//�m�ۂ��ꂽ�L���v�`���o�b�t�@�̃T�C�Y���擾
		DSCBCAPS caps;
		memset( std::addressof( caps ), 0, sizeof caps );
		caps.dwSize = sizeof caps;

		if( cap_buf_->GetCaps( std::addressof( caps ) ) == DS_OK )
		{
			cap_buf_size_ = caps.dwBufferBytes;
		}

		//�L���v�`���C�x���g�̃I�u�W�F�N�g���쐬
		for( int i = 0; i < dev_num; ++i )
		{
			cap_event_[ i ] = CreateEvent( NULL, FALSE, FALSE, NULL );
		}

		LPDIRECTSOUNDNOTIFY8 notify;
		if( cap_buf_->QueryInterface( IID_IDirectSoundNotify, reinterpret_cast< LPVOID * >( \
			std::addressof( notify ) ) ) != DS_OK )
		{
			return false;
		}

		DSBPOSITIONNOTIFY pos[ 2 ];
		pos[ 0 ].dwOffset = ( cap_buf_size_ / 2 ) - 1;
		pos[ 0 ].hEventNotify = cap_event_[ 0 ];
		pos[ 1 ].dwOffset = cap_buf_size_ - 1;
		pos[ 1 ].hEventNotify = cap_event_[ 1 ];
		notify->SetNotificationPositions( dev_num, pos );
		notify->Release();


		return true;
	}


	int sound_recorder::save_wave_file_header()
	{
		DWORD len = 0;

		LARGE_INTEGER pos1, pos2;
		pos1.QuadPart = 0;

		SetFilePointerEx( file_handle_, pos1, std::addressof( pos2 ), FILE_BEGIN );

		wave_file_header_.FileSize += data_len_;
		if( ! WriteFile( file_handle_, std::addressof( wave_file_header_ ), \
			sizeof( wave_file_header_ ), std::addressof( len ), NULL ) )
		{
			return -1;
		}

		//�t�@�N�g���`�����N�E�w�b�_���t�@�C���ɏ����o��
		wave_fact_chunk fact;

		memset( std::addressof( fact ), 0, sizeof fact );
		fact.Id = RIFFCC( 'fact' );
		fact.Size = 4;
		fact.Samples = data_len_ / wave_file_header_.BlockAlign;

		if( ! WriteFile( file_handle_, std::addressof( fact ), \
			sizeof fact, std::addressof( len ), NULL ) )
		{
			return -1;
		};

		wave_fact_chunk data;
		memset( std::addressof( data ), 0, sizeof data );
		data.Id = RIFFCC( 'data' );
		data.Size = data_len_;
		if( ! WriteFile( file_handle_, std::addressof( data ), sizeof data, std::addressof( len ), NULL ) )
		{
			return -1;
		}
		return 0;
	}


	int sound_recorder::save_open_wave_file( std::string const & filename,\
		WAVEFORMATEX & wf, bool const over_write )
	{
		//�I�[�f�B�I�E�t�H�[�}�b�g���s���ȏꍇ�A�G���[�ɂ���.
		if(( wf.wFormatTag      != WAVE_FORMAT_PCM )
			|| ( wf.nChannels       ==  0 )
			|| ( wf.nSamplesPerSec  ==  0 )
			|| ( wf.nAvgBytesPerSec ==  0 )
			|| ( wf.nBlockAlign     ==  0 )
			|| ( wf.wBitsPerSample  ==  0 ) )
		{
			return -1;
		}

		//�㏑�����邩�ǂ���
		auto const file_create_mode = over_write ? CREATE_ALWAYS : CREATE_NEW;

		file_handle_ = CreateFile( filename.c_str(), GENERIC_WRITE, FILE_SHARE_READ, NULL,
			file_create_mode, FILE_FLAG_RANDOM_ACCESS, NULL );

		if( file_handle_ == INVALID_HANDLE_VALUE )
		{
			file_handle_ = NULL;
			return -1;
		}

		memset( std::addressof( wave_file_header_ ), 0, sizeof( wave_file_header_ ) );

		wave_file_header_.RiffId         = RIFFCC( 'RIFF' );
		wave_file_header_.FileSize       = (16+12) + (8+4) + 8;		// WAV�w�b�_ + FACT + DATA.
		wave_file_header_.FileType       = RIFFCC( 'WAVE' );
		wave_file_header_.FormatId       = RIFFCC( 'fmt ' );
		wave_file_header_.FormatSize     = 16;
		wave_file_header_.FormatType     = wf.wFormatTag;
		wave_file_header_.Channels       = wf.nChannels;
		wave_file_header_.SamplesPerSec  = wf.nSamplesPerSec;
		wave_file_header_.AvgBytesPerSec = wf.nAvgBytesPerSec;
		wave_file_header_.BlockAlign     = wf.nBlockAlign;
		wave_file_header_.BitsPerSample  = wf.wBitsPerSample;

		//wav�t�@�C���̃w�b�_�����t�@�C���ɏ����o��
		if( save_wave_file_header() == -1 )
		{
			CloseHandle( file_handle_ );
			file_handle_ = NULL;
			return -1;
		}
		return 0;
	}


	void sound_recorder::open_file()
	{
		using namespace std;

		if( ! recording_ )
		{
			WAVEFORMATEX wf;

			wf.wFormatTag = WAVE_FORMAT_PCM;
			wf.nChannels = 2;
			wf.wBitsPerSample = 16;
			wf.nBlockAlign = 4;
			wf.nSamplesPerSec = 48000;
			wf.nAvgBytesPerSec = 48000 * wf.nBlockAlign;
			wf.cbSize = 0;


			//�㏑�����[�h�ŊJ��
			auto const result = save_open_wave_file( filename_, wf, true );

			if( result != 0 )
			{
				cout << "FILE OPEN ERROR" << endl;
				return;
			}


			//�ʒu�̏�����
			put_pos_ = 0;
			unsigned int num = 0;

			if( ! open_sound_cap_device( num, std::ref( wf ) ) )
			{
				std::cout << "�J����}�C�N������܂���ł���" << std::endl;
				return;
			}

			//�ǂݍ��݃o�b�t�@���m��
			if( data_buff_ == nullptr )
			{
				data_buff_ = new BYTE[ cap_buf_size_ ];
			}

			//�L���v�`���̊J�n
			cap_buf_->Start( DSCBSTART_LOOPING );
			recording_ = true;
			thread_ = boost::thread( & sound_recorder::capture_event_task, this, boost::ref( this->recording_ ) );

			//�T�E���h���̓f�o�C�X���I�[�v��

		}
	}

}