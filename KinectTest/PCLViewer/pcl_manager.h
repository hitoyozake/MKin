#include <iostream>
#include <cmath>
#include <NuiApi.h>
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/make_shared.hpp>
#include <string>
#include <opencv2/opencv.hpp>

int const KINECT_RANGES_TABLE_LEN = 2048; // or 1024 == 10bit

struct pcl_manager
{
public:
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer_;
	boost::shared_array< double > table_;

	void calculate_kinect_depth_table()
	{
		double const k1 = 1.1863;
		double const k2 = 2842.5;
		double const k3 = 0.1236;

		for( std::size_t i = 1; i < KINECT_RANGES_TABLE_LEN - 1; ++i )
		{
			//10bitÇÃèÍçá
			//table_[ i ] = k3 * tan( i / k2 + k1 );
			table_[ i ] = 1.0 / ( i * ( - 0.0030711016 ) + 3.3309495161 );
		}
		table_[ 0 ] = table_[ KINECT_RANGES_TABLE_LEN ] = 0;
	}


	pcl_manager() : table_( new double[ KINECT_RANGES_TABLE_LEN ] )
	{
	}

	void init( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr calib_cloud,
		std::string const & viewer_id )
	{
		viewer_ = boost::make_shared\
			< pcl::visualization::PCLVisualizer >();
		viewer_->setBackgroundColor( 0, 0, 0 );

		viewer_->addPointCloud( calib_cloud, viewer_id );

		viewer_->setPointCloudRenderingProperties( \
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, \
			viewer_id );
		viewer_->addCoordinateSystem( 1.0 );
		viewer_->initCameraParameters();
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
					//if( 0 <= color_x && color_x <= 640 && 0 <= color_y && color_y <= 480 )
					//{
					//	pcl::PointXYZRGB basic_point;
					//	auto * pixel_ptr = & color->imageData[ x * 4 + color->width * y * 4 ];
					//	basic_point.x = x * 0.0004;
					//	basic_point.y = y * 0.0004;
					//	basic_point.z = ( ( ( ( UINT16 * )( depth->imageData +\
					//		depth->widthStep * y ) )[ x ] ) >> 3  ) * 0.0005 - 0.0008;

					//	basic_point.r = pixel_ptr[ 2 ];
					//	basic_point.g = pixel_ptr[ 1 ];
					//	basic_point.b = pixel_ptr[ 0 ];
					//	//âÊëúì‡ÇÃèÍçá

					//	cloud_ptr->points.push_back( basic_point );
					//}
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
		cloud_ptr->height = 30;

		return cloud_ptr;
	}

	void rotate_depth( cv::Ptr< IplImage > & depth, double const theta )
	{

	}

	//yé≤âÒì]
	void rotate_cloud_y( pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr, double const theta )
	{
		double const sin_theta = sin( theta );
		double const cos_theta = cos( theta );

		for( auto it = cloud_ptr->begin(); it != cloud_ptr->end(); ++it )
		{
				it->x = cos_theta * it->x + sin_theta * it->z;
				it->z = - sin_theta * it->x + cos_theta * it->z;
		}
	}


	void  
		rotate_and_move_and_convert_RGB_and_depth_to_cloud( cv::Ptr< IplImage > const & color, \
		cv::Ptr< IplImage > const & depth, int const move_x, int const move_y, int const move_z, double const theta, pcl::PointCloud< pcl::PointXYZRGB >::Ptr & cloud_ptr )
	{
		const double pi = 3.141592653;/*
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );*/
		
		double py = 0;
		for( int y = 0; y < color->height; ++y )
		{
			for( int x = 0; x < color->width; ++x )
			{
			
				long color_x = x, color_y = y;
				//( UINT16 )( ( ( ( UINT16 * )( depth->imageData +\
						depth->widthStep * y ) )[ x ] ) >> 3  ), 
				HRESULT result =	NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
				 NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, 0, std::addressof( color_x ), std::addressof( color_y ) );
					//if( 0 <= color_x && color_x <= 640 && 0 <= color_y && color_y <= 480 )
					//{
					//	pcl::PointXYZRGB basic_point;
					//	auto * pixel_ptr = & color->imageData[ x * 4 + color->width * y * 4 ];
					//	basic_point.x = x * 0.0004;
					//	basic_point.y = y * 0.0004;
					//	basic_point.z = ( ( ( ( UINT16 * )( depth->imageData +\
					//		depth->widthStep * y ) )[ x ] ) >> 3  ) * 0.0005 - 0.0008;

					//	basic_point.r = pixel_ptr[ 2 ];
					//	basic_point.g = pixel_ptr[ 1 ];
					//	basic_point.b = pixel_ptr[ 0 ];
					//	//âÊëúì‡ÇÃèÍçá

					//	cloud_ptr->points.push_back( basic_point );
					//}
				if( result == S_OK )
				{
					pcl::PointXYZRGB basic_point;
					auto * pixel_ptr = & color->imageData[ color_x * 4 + color->width * color_y * 4 ];
					basic_point.x = ( 1.0 * ( x + move_x ) * cos( theta ) - 1.0 * ( y + move_y ) * sin( theta ) )* 0.0004;
					basic_point.y = py + ( ( 1 * 0.0004 ) / ( 1.0 + y * 0.01 ) );//( ( 1.0 * ( x + move_x ) * sin( theta ) + 1.0 * ( y + move_y ) * cos( theta ) ) * 0.0004 );
					basic_point.z = ( ( ( ( ( UINT16 * )( depth->imageData +\
						depth->widthStep * y ) )[ x ] ) >> 3  ) + move_z ) * 0.0005;

					//basic_point.y = ( 1.0 + y * y * 0.000003 ); 

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
			py = py + ( ( 1 * 0.0004 ) / ( 1.0 + y * 0.01 ) );
		}

		cloud_ptr->width = static_cast< int >( cloud_ptr->points.size() );
		cloud_ptr->height = 30;
		
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




