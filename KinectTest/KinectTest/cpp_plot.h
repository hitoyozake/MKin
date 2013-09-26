#pragma once

#include <vector>
#include <iostream>
#include <numeric>

namespace cpp_plot
{
	struct wnd_proc_messenger
	{
		bool update_;
		bool quit_;
		std::vector< int > histgram_;

		wnd_proc_messenger();
	};
};

