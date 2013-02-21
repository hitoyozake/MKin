#include "filesystem.h"

namespace filesystem
{
	boost::optional< unsigned long long > get_last_volume_by_GB()
	{
		TCHAR sz_root[ 16 ];
		ULARGE_INTEGER i64_used;
		ULARGE_INTEGER i64_free;
		ULARGE_INTEGER i64_avail;
		ULARGE_INTEGER i64_total;

		strcpy( sz_root, TEXT( "e:\\" ) );

		//ディスク容量を取得する
		GetDiskFreeSpaceEx( sz_root, std::addressof( i64_free ), \
			std::addressof( i64_total ), std::addressof( i64_avail ) );

		if( constant::DEBUG_MODE )
		{
			std::cout << "***********************************" << std::endl;
			std::cout << "DRIVE NAME : " << sz_root << std::endl;
			std::cout << "DISK AVAIABLE VOLUME : " << ( i64_avail.QuadPart / ( 1024 * 1024 * 1024 ) ) << std::endl;
		}
		return boost::optional< unsigned long long >( i64_avail.QuadPart / ( 1024 * 1024 * 1024 ) );
	}

}