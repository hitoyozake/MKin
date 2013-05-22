#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

//�m�C�Y���������Ă��܂���
//�摜�̖ʐόv�Z/�̈搔���グ������
//���Ȃ����ȁH�����Č덷���x���H


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


//�摜�̘a�����߂�
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

	cvErode( src, image, NULL, 3 );  //���k��3

	if( debug_mode )
	{
		cvShowImage( debug_window_name.c_str(), image );
	}

	cvDilate( image, final_result, NULL, 3 ); //�c����3

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
