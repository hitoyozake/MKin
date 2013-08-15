// ������Kinect�̃J�����摜��\������
#include <iostream>
#include <string>
#include <vector>
#include <array>
//#include <thread>

#include <fstream>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include <boost/lexical_cast.hpp>
// NuiApi.h�̑O��Windows.h���C���N���[�h����
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <glut.h>

#include "pcl_manager.h"
#include "global_parameter.h"
#define NO_MINMAX

#pragma comment( lib, "glut32.lib" )
#pragma comment( lib, "C:/Program Files/Microsoft SDKs/Kinect/v1.6/lib/x86/Kinect10.lib" )



int const KINECT_NUM = 4;

struct mouse_info
{
	int x1_, x2_, y1_, y2_;
	int flag_;

	mouse_info() : x1_( 0 ), x2_( 10 ), y1_( 0 ), y2_( 10 )
	{

	}
};

struct rect
{
	int x_, y_;
	int width_, height_;

	rect() : x_( 0 ), y_( 0 ), width_( 0 ), height_( 0 )
	{
	}

	rect( int const x, int const y, int const width, int const height ) : \
		x_( x ), y_( y ), width_( width ), height_( height ) 
	{
	}

};



struct Runtime
{
	INuiSensor * kinect;         // Kinect�̃C���X�^���X

	struct image_data
	{
		HANDLE event_;	//�f�[�^�X�V�C�x���g�n���h��
		HANDLE stream_handle_;  //�摜�f�[�^�̃n���h��
		std::string window_name_;
		cv::Ptr< IplImage > image_; //�\���f�[�^
	};

	image_data color_;
	image_data depth_;
};



struct graph
{
	struct data
	{
		std::string window_name_;
		cv::Ptr< IplImage > image_;
	};

	data depth_;
	data color_;

	std::string filename_;
};


void viewer_thread( pcl_manager & manager )
{
	std::cout << "viewer_thread launced" << std::endl;
	while( ! manager.end_mes_ )
	{
		Sleep( 15 );
		if( manager.inited_ )
		{
			std::cout << "hogehoge" << std::endl;
		//manager.viewer_->spinOnce( 100 );
		}

	}
	std::cout << "loop end" << std::endl;
}

void init( std::vector< graph > & graph )
{
	using namespace std;

	for( size_t i = 0; i < graph.size(); ++i )
	{
		//�[�x==============================================================

		// �E�B���h�E�����쐬
	
		graph[ i ].filename_ = string( "depth_" ) + boost::lexical_cast< string >\
			( i ) + ".txt";

		// OpenCV�̏����ݒ�
		graph[ i ].color_.image_ = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 4 );
		graph[ i ].depth_.image_ = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1  );
	}
}


void wait_input( bool & input_come, std::string & input )
{
	while( input != "end" )
	{
		if( ! input_come )
		{
			std::cin >> input;
			input_come = true;
			Sleep( 600 );
		}
	}
}


std::vector< std::string > get_filelist_from_current_dir()
{
	namespace fs = boost::filesystem;

	std::vector< std::string > result;

	const fs::path path( fs::current_path() );
	fs::directory_iterator end;
	for( fs::directory_iterator i( fs::current_path() ); i != end; ++i )
	{

		std::cout << i->path() << std::endl;

		if( ! fs::is_directory( i->path() ) )
		{
			result.push_back( i->path().string() );
		}
	}

	return result;
}


std::vector< std::string > get_recorded_filelist( std::vector< std::string > const & filelist )
{
	std::vector< std::string > result;

	using boost::regex;

	regex reg_ex( "([0-9]*[a-z]*[A-Z]*)*.txt$");

	boost::smatch match;

	for( int i = 0; i < filelist.size(); ++i )
	{
		//cout << i << endl;
		//���K�\���Ńf�[�^�t�@�C����T��
		//if( boost::regex_search( ( std::string )( i.c_str() ), match, reg_ex ) )
		if( filelist[ i ].find( "depth" ) != std::string::npos )
		{
			result.push_back( filelist[ i ] );
		}
	}
	return result;
}

void create_back_image( ifstream & back_depth, IplImage * result )
{
	int const FRAME_NUM = 300;
	auto tmp_image = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1  );
	
	back_depth.read( result->imageData ,640 * 480 * 2 );


	for( int i = 0; i < FRAME_NUM; ++i )
	{
		back_depth.read( tmp_image->imageData ,640 * 480 * 2 );

		for( int y = 0; y < result->height; ++y )
		{
			for( int x = 0; x < result->width; ++x )
			{
				( ( UINT16 * )( result->imageData +\
				result->widthStep * y ) )[ x ] =
				std::max( ( ( UINT16 * )( result->imageData +\
				result->widthStep * y ) )[ x ], ( ( UINT16 * )( tmp_image->imageData +\
				tmp_image->widthStep * y ) )[ x ] );
				
				
			}
		}
	}

	
}

std::vector< rect > get_rect_to_draw( std::string const & filename )
{	
	std::ifstream area( filename );
	//�ǂݍ��ݔ͈� ��\n x1:..\nx2:..\ny1:..\ny2:..; ":"��x1�Ȃǂ̕�������΂�

	if( area.is_open() )
	{
		int kinect_count = 0;
		area >> kinect_count;
		
		std::vector< rect > result( kinect_count );

		for( int i = 0; i < kinect_count; ++i )
		{
			std::string input;

			area >> input;

			int const x = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );

			area >> input;

			int const width = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );
 
			area >> input;
			int const y = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );
			
			area >> input;

			int const height = boost::lexical_cast< int >( input.substr( input.find( ":" ) + 1 ) );

			result[ i ].width_ = width;
			result[ i ].height_ = height;
			result[ i ].x_ = x;
			result[ i ].y_ = y;

		}
	}
	return std::vector< rect >();	
}



void draw()
{
	::cvNamedWindow( "show2d",  CV_WINDOW_KEEPRATIO );


	double const pi = 3.141592653;

	using namespace std;
	int const kinect_count = 1;
	bool const use_mouse = true;
	vector< ifstream >  ifs_depth( 4 );//( kinect_count );
	vector< ifstream >  ifs_color( 4 );//( kinect_count );
	vector< ifstream >  ifs_depth_back( 4 );//( kinect_count );
	vector< ifstream >  ifs_color_back( 4 );//( kinect_count );
	//std::ifstream ifs_timestamp( "debug_log.txt");
	
	auto const filelist = get_recorded_filelist( get_filelist_from_current_dir() );

	std::cout << "program started" << std::endl;
	//baby __20130523T104214
	//back            122852


	ifs_depth_back[ 0 ].open( "D:/rec2/depth__20130523T122852_0.txt", ios::binary  );
	ifs_depth_back[ 1 ].open( "D:/rec2/depth__20130523T122852_1.txt", ios::binary  );
	ifs_depth_back[ 2 ].open( "D:/rec2/depth__20130523T122852_2.txt", ios::binary  );
	ifs_depth_back[ 3 ].open( "D:/rec2/depth__20130523T122852_3.txt", ios::binary  );
	
	ifs_color[ 0 ].open( "C:/rec/color__20130523T104214_0.txt", ios::binary  );
	ifs_color[ 1 ].open( "C:/rec/color__20130523T104214_1.txt", ios::binary  );
	ifs_color[ 2 ].open( "C:/rec/color__20130523T104214_2.txt", ios::binary  );
	ifs_color[ 3 ].open( "C:/rec/color__20130523T104214_3.txt", ios::binary  );
	ifs_depth[ 0 ].open( "D:/rec2/depth__20130523T104214_0.txt", ios::binary  );
	ifs_depth[ 1 ].open( "D:/rec2/depth__20130523T104214_1.txt", ios::binary  );
	ifs_depth[ 2 ].open( "D:/rec2/depth__20130523T104214_2.txt", ios::binary  );
	ifs_depth[ 3 ].open( "D:/rec2/depth__20130523T104214_3.txt", ios::binary  );
	

	IplImage * depth_back[ 4 ];

	//�����p�摜�̈���m��
	//�����쐬�p�摜�𐶐�
	for( int i = 0; i < 4; ++i )
	{
		depth_back[ i ] = cvCreateImage( cvSize( 640, 480 ),  IPL_DEPTH_16U, 1 );

		create_back_image( ifs_depth_back[ i ], depth_back[ i ] );

	}

	if( ifs_color[ 0 ].fail() || ifs_color[ 1 ].fail() || \
		ifs_depth[ 0 ].fail() || ifs_depth[ 1 ].fail() )
		return;

	bool input_come = false;
	std::string input;

	boost::thread thread( wait_input, std::ref( input_come ), std::ref( input ) );

	try {

		std::vector< graph > graph( ifs_depth.size() );

		bool continue_flag = true;
		int count = 0;
		
		long now_angle = 0;

		init( graph );

		bool pause = false;
		bool quick = false;

		pcl_manager pcl_mn;

		bool first_time = true;
		bool next = true;
		bool icp = false;
		bool simple = true;
		bool flag_2d = false;

		std::array< gp::global_parameter, 4 > global_param;

		IplImage * mod_color = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 4  );

		while ( ( graph.size() > 0 ) && continue_flag )
		{
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr[ 4 ];
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr final_cloud( new pcl::PointCloud< pcl::PointXYZRGB > );

			for( int i = 0; i < 4; ++i )
			{
				cloud_ptr[ i ] = pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );
			}

			
			if( ! pause && next )
			{
				for( size_t i = 0; i < graph.size(); ++i )
				{
					ifs_depth[ i ].read( graph[ i ].depth_.image_->imageData, 640 * 480 * 2 ); 

					//if( count % 2 == 0 )
						ifs_color[ i ].read( graph[ i ].color_.image_->imageData, 640 * 480 * 4 ); 

					if( ifs_depth[ i ].eof() )
							continue_flag = false;

					if( 1)//! quick )
					{
						if( first_time )
						{
							first_time = false;
							pcl_mn.init( pcl_mn.convert_RGB_and_depth_to_cloud( \
								graph[ i ].color_.image_,
								graph[ i ].depth_.image_ ), "hoge" );
							pcl_mn.rotate_and_move_and_convert_RGB_and_depth_to_cloud_with_sub( \
								graph[ i ].color_.image_,
								graph[ i ].depth_.image_, global_param[ i ], cloud_ptr[ i ], simple, depth_back[ i ] );
						}
						else
						{

							/*pcl_mn.rotate_and_move_and_convert_RGB_and_depth_to_cloud( \
								graph[ i ].color_.image_,
								graph[ i ].depth_.image_, 0, 0, 0, cloud_ptr );
						*/	
							pcl_mn.rotate_and_move_and_convert_RGB_and_depth_to_cloud_with_sub( \
								graph[ i ].color_.image_,
								graph[ i ].depth_.image_, global_param[ i ], cloud_ptr[ i ], simple, depth_back[ i ], count % 2 );
							//icp���Ȃ��ꍇ�͊ȈՕ`��
						}
					}
				}
				
				cout << "frame : " << ++count << endl;
				//�ۗ�
				//pcl_mn.iterative_closest_point( cloud_ptr[ 0 ], cloud_ptr[ 1 ], final_cloud );
				pcl_mn.locked_ = true;
				
				if( icp && count >= 1 )
				{
					final_cloud->clear();
					auto const transform = pcl_mn.iterative_closest_point( cloud_ptr[ 0 ],
						cloud_ptr[ 1 ], final_cloud );

					pcl::transformPointCloud( * cloud_ptr[ 1 ], * cloud_ptr[ 1 ],
						transform );

					pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp_cloud( new pcl::PointCloud< pcl::PointXYZRGB > );

					* tmp_cloud = * cloud_ptr[ 0 ] + * cloud_ptr[ 1 ];
					final_cloud->clear();
					auto const transform2 = pcl_mn.iterative_closest_point( tmp_cloud, cloud_ptr[ 2 ], final_cloud );

					pcl::transformPointCloud( * cloud_ptr[ 2 ], * cloud_ptr[ 2 ],
						transform2 );
					
					tmp_cloud->clear();
					* tmp_cloud = * cloud_ptr[ 0 ] + * cloud_ptr[ 1 ];
					* tmp_cloud += * cloud_ptr[ 2 ];

					auto const transform3 = pcl_mn.iterative_closest_point( tmp_cloud, cloud_ptr[ 3 ], final_cloud );

					pcl::transformPointCloud( * cloud_ptr[ 3 ], * cloud_ptr[ 3 ],
						transform3 );
					
					final_cloud->clear();
					for( int i = 0; i < 4; ++i )
					{
						* final_cloud += * cloud_ptr[ i ];
					}
				}
				else
				{
					* final_cloud = * cloud_ptr[ 0 ] + * cloud_ptr[ 1 ];
					* final_cloud += * cloud_ptr[ 2 ];
					* final_cloud += * cloud_ptr[ 3 ];


					if( flag_2d )
					{
						pcl_mn.show_point_cloud_to_2d( "show2d", final_cloud );
						flag_2d = false;

					}
				
				}
				pcl_mn.update( final_cloud, "hoge" );
				pcl_mn.spin_once();
				for( int i = 0; i < 2; ++i )
					cloud_ptr[ i ]->clear();
				final_cloud->clear();
				pcl_mn.locked_ = false;
				cout << "nf" << endl;
			}

			//openCV�p
			cvWaitKey( 1 );
			
			pcl_mn.spin_once();
			if( ! quick )
				next = false;
			icp = false;
			simple = true;

			if( input_come )
			{
				if ( input == "end" ) {
					continue_flag = false;
					pcl_mn.end_mes_ = true;
					thread.join();
				}
				else if( input == "p" )
				{
					pause = ! pause;
				}
				else if( input == "n" )
				{
					next = true;
					//simple = false;
					quick = ! quick;
				}
				else if( input == "cmd" )
				{
					next = true;
					//����
					//�摜�ԍ�0-3
					//move�E�E�E���s�ړ��Arotate�E�E�E��]
					//xyz
					//���W�� or �p�x(radian �� pi ����)
					int num = 0;
					if( !( cin >> num ) )
					{
						input_come = false;
						continue;
					}
					
					num = std::max( 0, std::min( 3, num ) );
					
					std::string operate;
					if( !( cin >> operate ) )
					{
						input_come = false;
						continue;
					}
					std::string axis;
					cin >> axis;

					double deg = 0.0;

					if( !( cin >> deg ) )
					{
						input_come = false;
						continue;
					}
					if( operate == "move" )
					{
						if( axis == "x" )
							global_param[ num ].x_ += deg;
						if( axis == "y" )
							global_param[ num ].y_ += deg;
						if( axis == "z" )
							global_param[ num ].z_ += deg;	
					}
					if( operate == "rotate" )
					{
						if( axis == "x" )
							global_param[ num ].x_theta_ += deg * pi;
						if( axis == "y" )
							global_param[ num ].y_theta_ += deg * pi;
						if( axis == "z" )
							global_param[ num ].z_theta_ += deg * pi;	
					}
					if( operate == "icp" )
					{
						simple = false;
						icp = true;
					}
					if( operate == "2d" )
					{
						flag_2d = true;
					}

				}
				input_come = false;

			}
		}
		
	}
	catch ( std::exception & ex ) {
		std::cout << ex.what() << std::endl;
	}
	cvDestroyAllWindows();
}

int main()
{
	INuiSensor * kinect;
	NuiCreateSensorByIndex( 0, & kinect );
	kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH
				| NUI_INITIALIZE_FLAG_USES_AUDIO );
	HANDLE depth;
	HANDLE color;
	HANDLE event_c = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 4 );
	HANDLE event_d = ::cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );

	NUI_IMAGE_FRAME image_frame_depth_;
	NUI_IMAGE_FRAME image_frame_color_;

	NUI_IMAGE_FRAME * image_frame_depth = & image_frame_depth_;
	NUI_IMAGE_FRAME * image_frame_color = & image_frame_color_;

	NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
				0, 2, event_d, & depth );
	NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
				0, 2, event_c, & color );
	
	NuiImageStreamSetImageFrameFlags( \
		color, \
		NUI_IMAGE_STREAM_FLAG_SUPPRESS_NO_FRAME_DATA //����� �����t���[���}��
		| NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE //Near���[�h
		);
	NuiImageStreamSetImageFrameFlags( \
		depth, \
		NUI_IMAGE_STREAM_FLAG_SUPPRESS_NO_FRAME_DATA //����� �����t���[���}��
		| NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE //Near���[�h
		);

	::WaitForSingleObject( event_c, INFINITE );
	::WaitForSingleObject( event_d, INFINITE );

	kinect->NuiImageStreamGetNextFrame( event_c, 0,  image_frame_depth );
	kinect->NuiImageStreamGetNextFrame( event_d, 0,  image_frame_color );

	cout << "foofoo" << endl;
	draw();
	kinect->NuiShutdown();
	return 0;
}