#pragma once

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Windows.h>
#include <Vfw.h>

namespace video
{
	class vfw_manager
	{
	public:

		vfw_manager( std::string const & filename, std::string const & title, int const width, \
			int const height, int const time_scale, int const frame_rate,
			int const total_frame_count );

		bool write( bool const reverse, cv::Ptr< IplImage > image );
		~vfw_manager();
		void reset();

	private:
		void close();

		AVISTREAMINFO asi_;
		BITMAPINFOHEADER bmp_ih_;
		AVICOMPRESSOPTIONS opt_;
		COMPVARS comp_vars_;
		PAVIFILE pavi_file_;
		PAVISTREAM avi_stream_;
		PAVISTREAM avi_tmp_stream_;
		int frame_count_;
		int total_frames_;
		int width_, height_;

		std::vector< tagRGBQUAD > image_;
	};
}

