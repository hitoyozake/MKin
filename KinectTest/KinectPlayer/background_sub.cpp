#include "background_sub.h"

void background_sub( IplImage * depth_img1, IplImage * depth_img2, IplImage * result )
{
	for( int y = 0; y < depth_img1->height; ++y )
	{
		for( int x = 0; x < depth_img1->width; ++x )
		{
			auto const pixel1 = ( ( ( UINT16 * )( depth_img1->imageData +\
				depth_img1->widthStep * y ) )[ x ] ) >> 3;
			
			auto const pixel2 = ( ( ( UINT16 * )( depth_img2->imageData +\
				depth_img1->widthStep * y ) )[ x ] ) >> 3;

			( ( ( UINT16 * )( depth_img2->imageData +\
				depth_img1->widthStep * y ) )[ x ] )  = abs( pixel1 - pixel2 );
			
		}
	}
	cout << "e" << endl;
}
