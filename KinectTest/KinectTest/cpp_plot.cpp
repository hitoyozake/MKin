#include "cpp_plot.h"


namespace cpp_plot
{
	wnd_proc_messenger::wnd_proc_messenger() : \
		update_( false ), quit_( false )
	{
	}

	wnd_proc_messenger wnd_messenger[ 4 ];
}