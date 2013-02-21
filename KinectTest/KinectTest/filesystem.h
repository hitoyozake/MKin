#pragma once

#include <iostream>
#include <string>
#include <fstream>

#include <Windows.h>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include "const.h"

namespace filesystem
{
	boost::optional< unsigned long long > get_last_volume_by_GB();
}