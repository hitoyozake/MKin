#pragma once

#include <vector>
#include <iostream>
#include <numeric>

namespace cpp_plot
{
	struct wnd_proc_messenger
	{
		bool quit_;
		std::vector< int > histgram_;
		std::vector< int > buf_;


		wnd_proc_messenger();
	};
	extern wnd_proc_messenger wnd_messenger;

};

