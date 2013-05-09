#pragma once

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

void background_sub( IplImage * depth_img1, IplImage * depth_img2, IplImage * result );
