#include <Windows.h>
#include <NuiApi.h>
#include <d3d9.h>
#include <dsound.h>

class kinect_audio
{
private:
	void stop_sound();
public:
	void start();
	void device_enum()
	{

	}

};

void kinect_audio::start()
{
	LPDIRECTSOUNDCAPTURE8 cap_dev;
	LPDIRECTSOUNDCAPTUREBUFFER cap_buf;

	WAVEFORMATEX  wfm = { WAVE_FORMAT_PCM, 1, 44100, 1, 8, 0 };


}