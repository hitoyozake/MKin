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
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, \
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
				HRESULT result = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
				 NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] ) ), std::addressof( color_x ), std::addressof( color_y ) );
				
				if( result == S_OK )
				{
					auto const real_point = NuiTransformDepthImageToSkeleton( x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), NUI_IMAGE_RESOLUTION_640x480 );
					pcl::PointXYZRGB basic_point;
					auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
					basic_point.x = real_point.x;
					basic_point.y = real_point.y;
					basic_point.z = real_point.z; - 1.35;//( ( ( ( UINT16 * )( depth->imageData +\
						depth->widthStep * y ) )[ x ] ) >> 3  ) * 0.0005;// / ( y * a + 1 );
					a += 100.0;
					basic_point.r = pixel_ptr[ 2 ];
					basic_point.g = pixel_ptr[ 1 ];
					basic_point.b = pixel_ptr[ 0 ];
					//âÊëúì‡ÇÃèÍçá

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

	//Point CLoud -> 2D
	void show_point_cloud_to_2d( std::string const & window_name, 
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr )
	{
		std::cout << "show2d" << std::endl;
		int const width = 640;
		int const height = 480;
		IplImage * image = cvCreateImage( cvSize( width, height ), IPL_DEPTH_16U, 1 );
		

		int cnt = 0;

		for( auto it = cloud_ptr->begin(); it != cloud_ptr->end(); ++it )
		{
			Vector4 real_point;

			real_point.x = it->x;
			real_point.y = it->y;
			real_point.z = it->z + 1.35;
			//real_point.w = 100.0f;

			unsigned short depth = 0;
			long x = 0, y = 0;

			NuiTransformSkeletonToDepthImage( real_point, std::addressof( x ), std::addressof( y ), std::addressof( depth ) , NUI_IMAGE_RESOLUTION_640x480 );
			++cnt;

			if( x >= 0 && x < width && y >= 0 && y < height )
			{
			//	//ÉsÉNÉZÉãÇÃèëÇ´çûÇ›
			//	if( cnt % 1000 == 0 )
			//	{
			//		std::cout << "x:" << x << ",y:" << y << ",z :" << depth << std::endl;
			//	}
				( UINT16 )( ( ( ( UINT16 * )( image->imageData +\
						image->widthStep * y ) )[ x ] ) ) = (UINT16)( depth );
			}
		}

		cvShowImage( window_name.c_str(), image );

		cvReleaseImage( std::addressof( image ) );

	}

	//îwåiç∑ï™ã@î\ïtÇ´
	void rotate_and_move_and_convert_RGB_and_depth_to_cloud_with_sub( cv::Ptr< IplImage > const & color, \
		cv::Ptr< IplImage > const & depth, gp::global_parameter const & g_param, pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, \
		bool const light, IplImage * const & depth_back, bool const only_depth = false )
	{
		double const pi = 3.141592653;/*
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );*/
		
		double py = 0;

		//îwåiç∑ï™É}ÉXÉNÇçÏê¨

		IplImage * mask = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 1 );

		//for( int y = 0; y < color->height; ++y )
		//{
		//		for( int x = 0; x < color->width; ++x )
		//		{
		//			auto const d = ( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
		//				depth->widthStep * y ) )[ x ] ) );

		//			auto const back_d = ( UINT16 )( ( ( ( UINT16 * )( depth_back->imageData +\
		//				depth_back->widthStep * y ) )[ x ] ) );

		//			auto const threshold = 100;
		//			if( abs( d - back_d ) < threshold )
		//			{
		//				( mask->imageData +\
		//				mask->widthStep * y )[ x ]= 0;
		//				//îwåiÇ∆àÍèè
		//				continue;
		//			}
		//			else
		//			{
		//				( mask->imageData +\
		//				mask->widthStep * y )[ x ] = 100;
		//			}
		//		}
		//}

		////é˚èkÇ∆ñcí£
		//cvErode( mask, mask, NULL, 2 );  //é˚èkâÒêî2

		//cvSmooth( mask, mask, CV_MEDIAN,  13, 13, 0, 0 );

		//cvDilate( mask, mask, NULL, 3 );  //ñcí£âÒêî3

		for( int y = 0; y < color->height; ++y )
		{
			if( ! light || light && y % 2 == 0 )
			{
				for( int x = 0; x < color->width; ++x )
				{
					if( ! light || light && x % 2 == 0 )
					{
						long color_x = x, color_y = y;
						//íçà” : KinectÇê⁄ë±ÇµÇƒÇ¢Ç»ÇØÇÍÇŒìÆÇ©Ç»Ç¢

						//auto const d = ( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
						//	depth->widthStep * y ) )[ x ] ) >> 3 );
						//
						//auto const back_d = ( UINT16 )( ( ( ( UINT16 * )( depth_back->imageData +\
						//	depth_back->widthStep * y ) )[ x ] ) >> 3  );

						//auto const mask_d = ( ( ( ( mask->imageData +\
						//	mask->widthStep * y ) )[ x ] ) );

						//auto const threshold = 100;
						//if( abs( d - back_d ) < threshold || mask_d == 0 )
						//{
						//	//îwåiÇ∆àÍèè
						//	continue;
						//}

						HRESULT result = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
							NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), std::addressof( color_x ), std::addressof( color_y ) );
						if( result == S_OK && color_x >= 0 && color_x < 640 && \
							color_y >= 0 && color_y < 480 )
						{
							//âÊñ ì‡ÇÃèÍçá
							
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


	//yé≤âÒì]
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

	//xé≤âÒì]
	void rotate_cloud_x_axis( pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, double const theta )
	{
		double const sin_theta = sin( theta );
		double const cos_theta = cos( theta );
		
		//R( Éø, 0,0 )
		Eigen::Matrix4f rotate;
		rotate << 1, 0, 0, 0, \
				0, cos_theta, - sin_theta, 0,
				0, sin_theta, cos_theta, 0,
				0, 0, 0, 1;

		pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, rotate );
	}

	//zé≤âÒì]
	void rotate_cloud_z_axis( pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, double const theta )
	{
		double const sin_theta = sin( theta );
		double const cos_theta = cos( theta );

				//R( Éø, 0,0 )
		Eigen::Matrix4f rotate;
		rotate << \
				cos_theta, - sin_theta, 0, 0, \
				sin_theta, cos_theta,   0, 0,
				0,0,1,0,
				0, 0, 0, 1;

		pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, rotate );

	}

	//zé≤âÒì]
	Eigen::Matrix4f get_rotate_matrix( double const x_angle, double const y_angle, double const z_angle )
	{
		double const zsin_theta = sin( z_angle );
		double const zcos_theta = cos( z_angle );
		double const xsin_theta = sin( x_angle );
		double const xcos_theta = cos( x_angle );
		double const ysin_theta = sin( y_angle );
		double const ycos_theta = cos( y_angle );		
		//R( Éø, 0,0 )
		Eigen::Matrix4f zrotate;
		zrotate << \
				zcos_theta, - zsin_theta, 0, 0, \
				zsin_theta, zcos_theta,   0, 0,
				0,0,1,0,
				0, 0, 0, 1;

		Eigen::Matrix4f xrotate;
		xrotate << 1, 0, 0, 0, \
				0, xcos_theta, - xsin_theta, 0,
				0, xsin_theta, xcos_theta, 0,
				0, 0, 0, 1;

		
		Eigen::Matrix4f yrotate;
		yrotate << ycos_theta, 0, ysin_theta, 0, \
				0,             1,          0, 0,
				-ysin_theta,   0, ycos_theta, 0,
				0, 0, 0, 1;

		return zrotate * xrotate * yrotate;
	}

	Eigen::Matrix4f get_move_matrix( double const x, double const y, double const z )
	{
		Eigen::Matrix4f move_matrix;

		move_matrix << 1, 0, 0, x,
			           0, 1, 0, y,
					   0, 0, 1, z,
					   0, 0, 0, 1;
		return move_matrix;
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
						//íçà” : KinectÇê⁄ë±ÇµÇƒÇ¢Ç»ÇØÇÍÇŒìÆÇ©Ç»Ç¢
						HRESULT result = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
							NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), std::addressof( color_x ), std::addressof( color_y ) );
						if( result == S_OK && color_x >= 0 && color_x < 640 && \
							color_y >= 0 && color_y < 480 )
						{
							//âÊñ ì‡ÇÃèÍçá

							auto const real_point = NuiTransformDepthImageToSkeleton( x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), NUI_IMAGE_RESOLUTION_640x480 );
							pcl::PointXYZRGB basic_point;

							auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
							basic_point.x = real_point.x;
							basic_point.y = real_point.y;
							basic_point.z = real_point.z - 1.35;
							
							

							//basic_point.y = ( 1.0 + y * y * 0.000003 ); 

							basic_point.r = 120;//pixel_ptr[ 2 ];
							basic_point.g = 120;//pixel_ptr[ 1 ];
							basic_point.b = 140;//pixel_ptr[ 0 ];
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

	void cloud_task( cv::Ptr< IplImage > const & color, \
		cv::Ptr< IplImage > const & depth, gp::global_parameter const & g_param, pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, \
		bool const light, std::vector< gp::global_parameter > const & base )
	{
		const double pi = 3.141592653;/*
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );*/
		static int debugcounter = 0;
		double py = 0;
		debugcounter++;
		for( int y = 0; y < color->height; ++y )
		{
			if( ( y % 2 ) ) //! light || light && y  == 0 )
			{
				for( int x = 0; x < color->width; ++x )
				{
					if( x % 2 ) //! light || light && x == 0 )
					{
						long color_x = 0, color_y = 0;
						//íçà” : KinectÇê⁄ë±ÇµÇƒÇ¢Ç»ÇØÇÍÇŒìÆÇ©Ç»Ç¢
						HRESULT result = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
							NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), std::addressof( color_x ), std::addressof( color_y ) );
						if( result == S_OK && color_x >= 0 && color_x < color->width && \
							color_y >= 0 && color_y < color->height )
						{
							//âÊñ ì‡ÇÃèÍçá

							auto const real_point = NuiTransformDepthImageToSkeleton( x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), NUI_IMAGE_RESOLUTION_640x480 );
							pcl::PointXYZRGB basic_point;

							auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
							basic_point.x = real_point.x;
							basic_point.y = real_point.y;
							basic_point.z = -real_point.z;
							

							//basic_point.y = ( 1.0 + y * y * 0.000003 ); 
							
							basic_point.r = 0 ? pixel_ptr[ 2 ] : 100;
							basic_point.g = 0 ? pixel_ptr[ 1 ] : 100;
							basic_point.b = 0 ? pixel_ptr[ 0 ] : 150;
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
		
		Eigen::Matrix4f matrix;

		matrix << 1, 0, 0, 0,
			      0, 1, 0, 0,
				  0, 0, 1, 0,
				  0, 0, 0, 1;

		//å¥ì_à⁄ìÆ
		for( auto itt = base.crbegin(); itt != base.crend(); ++itt )
		{
			matrix *= get_rotate_matrix( itt->x_theta_, itt->y_theta_, itt->z_theta_ );
			matrix *= get_move_matrix( itt->x_, itt->y_, itt->z_ );
		//	pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, matrix );
		}

		matrix *= get_rotate_matrix( g_param.x_theta_, g_param.y_theta_, g_param.z_theta_ );
		matrix *= get_move_matrix( g_param.x_, g_param.y_, g_param.z_ );
		
		pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, matrix );
	}

	void cloud_task_sub( cv::Ptr< IplImage > const & color, \
		cv::Ptr< IplImage > const & depth, gp::global_parameter const & g_param, pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, \
		bool const light, std::vector< gp::global_parameter > const & base, IplImage * const & depth_back )
	{
		const double pi = 3.141592653;/*
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );*/
		
		double py = 0;

		IplImage * mask = cvCreateImage( cvSize( depth_back->width, depth_back->height ), IPL_DEPTH_8U, 1 );

		for( int y = 0; y < color->height; ++y )
		{
				for( int x = 0; x < color->width; ++x )
				{
					auto const d = ( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
						depth->widthStep * y ) )[ x ] ) );

					auto const back_d = ( UINT16 )( ( ( ( UINT16 * )( depth_back->imageData +\
						depth_back->widthStep * y ) )[ x ] ) );

					auto const threshold = 20;
					if( abs( d - back_d ) < threshold )
					{
						( mask->imageData +\
						mask->widthStep * y )[ x ]= 0;
						//îwåiÇ∆àÍèè
						continue;
					}
					else
					{
						( mask->imageData +\
						mask->widthStep * y )[ x ] = 100;
					}
				}
		}

		//é˚èkÇ∆ñcí£
		cvErode( mask, mask, NULL, 2 );  //é˚èkâÒêî2

		cvSmooth( mask, mask, CV_MEDIAN,  5, 5, 0, 0 );

		cvDilate( mask, mask, NULL, 3 );  //ñcí£âÒêî3


		for( int y = 0; y < color->height; ++y )
		{
			if( ! light || light && y % 2 == 0 )
			{
				for( int x = 0; x < color->width; ++x )
				{
					if( ! light || light && x % 2 == 0 )
					{
						long color_x = x, color_y = y;
						//íçà” : KinectÇê⁄ë±ÇµÇƒÇ¢Ç»ÇØÇÍÇŒìÆÇ©Ç»Ç¢

						auto const d = ( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
							depth->widthStep * y ) )[ x ] ) >> 3 );
						
						auto const back_d = ( UINT16 )( ( ( ( UINT16 * )( depth_back->imageData +\
							depth_back->widthStep * y ) )[ x ] ) >> 3  );

						auto const mask_d = ( ( ( ( mask->imageData +\
							mask->widthStep * y ) )[ x ] ) );

						auto const threshold = 20;
						if( abs( d - back_d ) < threshold || mask_d == 0 )
						{
							//îwåiÇ∆àÍèè
							continue;
						}

						HRESULT result = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
							NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), std::addressof( color_x ), std::addressof( color_y ) );
						if( result == S_OK && color_x >= 0 && color_x < color->width && \
							color_y >= 0 && color_y < color->height )
						{
							//âÊñ ì‡ÇÃèÍçá

							auto const real_point = NuiTransformDepthImageToSkeleton( x, y, ( ( ( ( UINT16 * )( depth->imageData +\
								depth->widthStep * y ) )[ x ] )  ), NUI_IMAGE_RESOLUTION_640x480 );
							pcl::PointXYZRGB basic_point;

							auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
							basic_point.x = real_point.x;
							basic_point.y = real_point.y;
							basic_point.z = real_point.z - 1.35;
							

							//basic_point.y = ( 1.0 + y * y * 0.000003 ); 

							basic_point.r = 120;//pixel_ptr[ 2 ];
							basic_point.g = 120;//pixel_ptr[ 1 ];
							basic_point.b = 140;//pixel_ptr[ 0 ];
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
		if( depth->width != depth_back->width )
		{
			std::cout << "what" << std::endl;
		}

		cloud_ptr->width = static_cast< int >( cloud_ptr->points.size() );
		cloud_ptr->height = 1;
		
		Eigen::Matrix4f matrix;

		matrix << 1, 0, 0, 0,
			      0, 1, 0, 0,
				  0, 0, 1, 0,
				  0, 0, 0, 1;

		//å¥ì_à⁄ìÆ
		for( auto itt = base.crbegin(); itt != base.crend(); ++itt )
		{
			matrix *= get_rotate_matrix( itt->x_theta_, itt->y_theta_, itt->z_theta_ );
			matrix *= get_move_matrix( itt->x_, itt->y_, itt->z_ );
		//	pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, matrix );

			std::cout << matrix << std::endl;
		}

		matrix *= get_rotate_matrix( g_param.x_theta_, g_param.y_theta_, g_param.z_theta_ );
		matrix *= get_move_matrix( g_param.x_, g_param.y_, g_param.z_ );
		
		pcl::transformPointCloud( * cloud_ptr, * cloud_ptr, matrix );
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr final_cloud( new pcl::PointCloud< pcl::PointXYZRGB > );

		for( auto it = cloud_ptr->begin(); it != cloud_ptr->end(); ++it )
		{
			if( it->z > -0.075 )
			{
				final_cloud->push_back( * it );
			}
		}

		* cloud_ptr = * final_cloud;

		//cvRelease( mask->imageData );
	}

	void update( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr const & cloud, std::string const & viewer_id )
	{
		viewer_->updatePointCloud( cloud, viewer_id );
	}

	void spin_once()
	{	
		viewer_->spinOnce();	//ï`âÊçXêV
	}
private:
};




