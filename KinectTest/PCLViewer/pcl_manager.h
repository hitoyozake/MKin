#include <iostream>
#include <cmath>
#include <Windows.h>
//==========
#include <NuiApi.h>
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/make_shared.hpp>
#include <boost/progress.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/registration/icp.h>

#include "global_parameter.h"

int const KINECT_RANGES_TABLE_LEN = 2048; // or 1024 == 10bit

struct pcl_manager
{
public:
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer_;
	bool locked_;
	bool end_mes_;
	bool inited_;

	void loop()
	{

	}

	
	pcl_manager()
	{
		locked_ = false;
		end_mes_ = false;
		inited_ = false;
	}

	void init( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr calib_cloud,
		std::string const & viewer_id )
	{
		viewer_ = boost::make_shared\
			< pcl::visualization::PCLVisualizer >();

		viewer_->addPointCloud( calib_cloud, viewer_id );

		viewer_->setPointCloudRenderingProperties( \
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, \
			viewer_id );
		viewer_->addCoordinateSystem( 1.0 );
		viewer_->initCameraParameters();
		viewer_->setBackgroundColor( 0.1, 0.1, 0.1 );

		spin_once();
		locked_ = false;
		end_mes_ = false;
		inited_ = true;
	}

	Eigen::Matrix4f iterative_closest_point( pcl::PointCloud< pcl::PointXYZRGB >::Ptr const in1, 
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr in2, pcl::PointCloud< pcl::PointXYZRGB >::Ptr result )
	{

		std::cout << "ICP" << std::endl;
		boost::progress_timer timer;
		pcl::IterativeClosestPoint< pcl::PointXYZRGB, pcl::PointXYZRGB > icp;
		
		icp.setInputCloud( in1 );
		icp.setInputTarget( in2 );
		
		icp.setMaximumIterations( 15 );
		icp.setMaxCorrespondenceDistance( 0.01 );
		icp.align( * result );

		std::cout << "ICP END" << std::endl;

		std::cout << icp.getFinalTransformation() << std::endl;

		return icp.getFinalTransformation();
	}


	pcl::PointCloud< pcl::PointXYZRGB >::Ptr 
		convert_RGB_and_depth_to_cloud( cv::Ptr< IplImage > const & color, \
		cv::Ptr< IplImage > const & depth )
	{
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );
		static double a = 1.0;

		for( int y = 0; y < color->height; ++y )
		{
			for( int x = 0; x < color->width; ++x )
			{
			
				long color_x = x, color_y = y;
				//( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
						depth->widthStep * y ) )[ x ] ) >> 3  ), 
				HRESULT result =	NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
				 NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, 0, std::addressof( color_x ), std::addressof( color_y ) );

				if( result == S_OK )
				{
					pcl::PointXYZRGB basic_point;
					auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
					basic_point.x = x * 0.0004;
					basic_point.y = ( y * 0.0004 ) / ( 1 + y * 20 );
					basic_point.z = ( ( ( ( UINT16 * )( depth->imageData +\
						depth->widthStep * y ) )[ x ] ) >> 3  ) * 0.0005;// / ( y * a + 1 );
					a += 100.0;
					basic_point.r = pixel_ptr[ 2 ];
					basic_point.g = pixel_ptr[ 1 ];
					basic_point.b = pixel_ptr[ 0 ];
					//画像内の場合

					cloud_ptr->points.push_back( basic_point );
				}
				else
				{

					if( result == E_POINTER )
					{
						std::cout << "NU" << endl;
					}
					else if( result == E_NUI_DEVICE_NOT_READY )
					{
						std::cout << "NUIDEVREADY_NOT" << endl;
						
					}
					else if( result == E_INVALIDARG )
					{
						std::cout << "INV" << endl;
					}
					else
					{
						std::cout << "NAZO:";
						std::cout << result << endl;
					}
				}
			}
		}

		cloud_ptr->width = static_cast< int >( cloud_ptr->points.size() );
		cloud_ptr->height = 1;

		return cloud_ptr;
	}

	//背景差分機能付き
	void rotate_and_move_and_convert_RGB_and_depth_to_cloud_with_sub( cv::Ptr< IplImage > const & color, \
		cv::Ptr< IplImage > const & depth, gp::global_parameter const & g_param, pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, \
		bool const light, IplImage * const & depth_back, bool const only_depth = false )
	{
		const double pi = 3.141592653;/*
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );*/
		
		double py = 0;

		//背景差分マスクを作成

		IplImage * mask = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_16U, 1 );

		for( int y = 0; y < color->height; ++y )
		{
				for( int x = 0; x < color->width; ++x )
				{
					auto const d = ( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
						depth->widthStep * y ) )[ x ] ) );

					auto const back_d = ( UINT16 )( ( ( ( UINT16 * )( depth_back->imageData +\
						depth_back->widthStep * y ) )[ x ] ) );

					auto const threshold = 100;
					if( abs( d - back_d ) < threshold )
					{
						( UINT16 )( ( ( ( UINT16 * )( mask->imageData +\
						depth->widthStep * y ) )[ x ] ) ) = 0;
						//背景と一緒
						continue;
					}
					else
					{
						( UINT16 )( ( ( ( UINT16 * )( mask->imageData +\
						depth->widthStep * y ) )[ x ] ) ) = d;
					}
				}
		}

		//収縮と膨張
		cvErode( mask, mask, NULL, 2 );  //収縮回数2
		cvDilate( mask, mask, NULL, 4 );  //膨張回数3

		for( int y = 0; y < color->height; ++y )
		{
			if( ! light || light && y % 2 == 0 )
			{
				for( int x = 0; x < color->width; ++x )
				{
					if( ! light || light && x % 2 == 0 )
					{
						long color_x = x, color_y = y;
						//注意 : Kinectを接続していなければ動かない

						auto const d = ( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
							depth->widthStep * y ) )[ x ] ) >> 3 );
						
						auto const back_d = ( UINT16 )( ( ( ( UINT16 * )( depth_back->imageData +\
							depth_back->widthStep * y ) )[ x ] ) >> 3  );

						auto const mask_d = ( UINT16 )( ( ( ( UINT16 * )( mask->imageData +\
							mask->widthStep * y ) )[ x ] ) >> 3  );

						auto const threshold = 100;
						if( abs( d - back_d ) < threshold || mask_d == 0 )
						{
							//背景と一緒
							continue;
						}


						HRESULT result = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
							NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), std::addressof( color_x ), std::addressof( color_y ) );
						if( result == S_OK && color_x >= 0 && color_x < 640 && \
							color_y >= 0 && color_y < 480 )
						{
							//画面内の場合

							auto const real_point = NuiTransformDepthImageToSkeleton( x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), NUI_IMAGE_RESOLUTION_640x480 );
							pcl::PointXYZRGB basic_point;

							auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
							basic_point.x = real_point.x;
							basic_point.y = real_point.y;
							basic_point.z = real_point.z - 1.35;

							//basic_point.y = ( 1.0 + y * y * 0.000003 ); 

							basic_point.r = 120;only_depth ? 120 : pixel_ptr[ 2 ];
							basic_point.g = 120;only_depth ? 120 : pixel_ptr[ 1 ];
							basic_point.b = 140;only_depth ? 140 : pixel_ptr[ 0 ];
							//110.125

							cloud_ptr->points.push_back( basic_point );
						}
						else
						{

							if( result == E_POINTER )
							{
								std::cout << "NU" << endl;
							}
							else if( result == E_NUI_DEVICE_NOT_READY )
							{
								std::cout << "NUIDEVREADY_NOT" << endl;

							}
							else if( result == E_INVALIDARG )
							{
								std::cout << "INV" << endl;
							}
							else
							{
								//std::cout << "NAZO:";
								//std::cout << result << endl;
							}
						}
					}
				}
			}
		}

		cvRelease( ( void ** )& mask );

		cloud_ptr->width = static_cast< int >( cloud_ptr->points.size() );
		cloud_ptr->height = 1;
		rotate_cloud_x_axis( cloud_ptr, g_param.x_theta_ );
		rotate_cloud_y_axis( cloud_ptr, g_param.y_theta_ );
		rotate_cloud_z_axis( cloud_ptr, g_param.z_theta_ );

		for( auto it = cloud_ptr->begin(); it != cloud_ptr->end(); ++it )
		{
			it->x += g_param.x_;
			it->y += g_param.y_;
			it->z += g_param.z_;
		}
		std::cout << "foo" << std::endl;
	}


	//y軸回転
	void rotate_cloud_y_axis( pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, double const theta )
	{
		double const sin_theta = sin( theta );
		double const cos_theta = cos( theta );

		for( auto it = cloud_ptr->begin(); it != cloud_ptr->end(); ++it )
		{
			auto const x = it->x; auto const z = it->z;
				it->x = cos_theta * x + sin_theta * z;
				it->z = - sin_theta * x + cos_theta * z;
		}
	}

	//x軸回転
	void rotate_cloud_x_axis( pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, double const theta )
	{
		double const sin_theta = sin( theta );
		double const cos_theta = cos( theta );
		
		//R( α, 0,0 )
		Eigen::Matrix4f rotate;
		rotate << 1, 0, 0, 0, \
				0, cos_theta, - sin_theta, 0,
				0, sin_theta, cos_theta, 0,
				0, 0, 0, 1;

		pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, rotate );
	}

	//z軸回転
	void rotate_cloud_z_axis( pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, double const theta )
	{
		double const sin_theta = sin( theta );
		double const cos_theta = cos( theta );

				//R( α, 0,0 )
		Eigen::Matrix4f rotate;
		rotate << \
				cos_theta, - sin_theta, 0, 0, \
				sin_theta, cos_theta,   0, 0,
				0,0,1,0,
				0, 0, 0, 1;

		pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, rotate );

	}

	void fusion_cloud( pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, pcl::PointCloud< pcl::PointXYZRGB >::Ptr & result )
	{
		* result += * cloud_ptr;
	}

	void rotate_and_move_and_convert_RGB_and_depth_to_cloud( cv::Ptr< IplImage > const & color, \
		cv::Ptr< IplImage > const & depth, gp::global_parameter const & g_param, pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, \
		bool const light )
	{
		const double pi = 3.141592653;/*
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );*/
		
		double py = 0;
		for( int y = 0; y < color->height; ++y )
		{
			if( ! light || light && y % 4 == 0 )
			{
				for( int x = 0; x < color->width; ++x )
				{
					if( ! light || light && x % 4 == 0 )
					{
						long color_x = x, color_y = y;
						//注意 : Kinectを接続していなければ動かない
						HRESULT result = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
							NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), std::addressof( color_x ), std::addressof( color_y ) );
						if( result == S_OK && color_x >= 0 && color_x < 640 && \
							color_y >= 0 && color_y < 480 )
						{
							//画面内の場合

							auto const real_point = NuiTransformDepthImageToSkeleton( x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), NUI_IMAGE_RESOLUTION_640x480 );
							pcl::PointXYZRGB basic_point;

							auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
							basic_point.x = real_point.x;
							basic_point.y = real_point.y;
							basic_point.z = real_point.z - 1.35;

							//basic_point.y = ( 1.0 + y * y * 0.000003 ); 

							basic_point.r = pixel_ptr[ 2 ];
							basic_point.g = pixel_ptr[ 1 ];
							basic_point.b = pixel_ptr[ 0 ];
							//110.125

							cloud_ptr->points.push_back( basic_point );
						}
						else
						{

							if( result == E_POINTER )
							{
								std::cout << "NU" << endl;
							}
							else if( result == E_NUI_DEVICE_NOT_READY )
							{
								std::cout << "NUIDEVREADY_NOT" << endl;

							}
							else if( result == E_INVALIDARG )
							{
								std::cout << "INV" << endl;
							}
							else
							{
								//std::cout << "NAZO:";
								//std::cout << result << endl;
							}
						}
					}
				}
			}
		}

		cloud_ptr->width = static_cast< int >( cloud_ptr->points.size() );
		cloud_ptr->height = 1;
		rotate_cloud_x_axis( cloud_ptr, g_param.x_theta_ );
		rotate_cloud_y_axis( cloud_ptr, g_param.y_theta_ );
		rotate_cloud_z_axis( cloud_ptr, g_param.z_theta_ );

		for( auto it = cloud_ptr->begin(); it != cloud_ptr->end(); ++it )
		{
			it->x += g_param.x_;
			it->y += g_param.y_;
			it->z += g_param.z_;
		}
	}

	void update( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr const & cloud, std::string const & viewer_id )
	{
		viewer_->updatePointCloud( cloud, viewer_id );
	}

	void spin_once()
	{	
		viewer_->spinOnce();	//描画更新
	}
private:
};




