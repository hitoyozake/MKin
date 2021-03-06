#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

//ノイズさえ消してしまえば
//画像の面積計算/領域数え上げだけで
//問題ないかな？あって誤差レベル？


int calc_square( IplImage * image )
{
	int counter = 0;

	for( int i = 0; i < image->width * image->height; ++i )
	{
		if( image->imageData[ i ] != 0 )
		{
			++counter;
		}
	}
	return counter;
}


//画像の和を求める
void add_image( IplImage * src1, IplImage * src2, IplImage * result )
{
	for( int i = 0; i < src1->width * src2->height; ++i )
	{
		result->imageData[ i ] = src1->imageData[ i ] || src2->imageData[ i ];
	}
}

//
void prepare( IplImage * src, std::string const & debug_window_name, bool const debug_mode )
{
	IplImage * image = cvCreateImage( cvSize( src->width, src->height ), src->depth, 1 );
	IplImage * final_result = cvCreateImage( cvSize( src->width, src->height ), src->depth, 1 );

	cvErode( src, image, NULL, 3 );  //収縮回数3

	if( debug_mode )
	{
		cvShowImage( debug_window_name.c_str(), image );
	}

	cvDilate( image, final_result, NULL, 3 ); //膨張回数3

	if( debug_mode )
	{
		cvShowImage( debug_window_name.c_str(), final_result );
	}
}

void save_image( IplImage * image, std::string const & filename )
{
	using namespace std;

	std::ofstream ofs( filename, std::fstream::binary );

	ofs.write( image->imageData, image->widthStep * image->height );

}

void read_image( IplImage * result, std::string const & filename )
{
	using namespace std;

	std::ifstream ifs( filename, std::fstream::binary );

	ifs.read( result->imageData, result->widthStep * result->height );
}


int main()
{
	std::cout << "hello" << std::endl;
	return 0;
}
