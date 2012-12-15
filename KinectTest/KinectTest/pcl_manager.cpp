#include "pcl_manager.h"
#include <opencv2/opencv.hpp>
#include <NuiApi.h>
#include <boost/shared_ptr.hpp>
#pragma comment( lib, "pcl_visualization_debug.lib" )


namespace pcl_manager
{

	class pcl_manager
	{
	public:
		void view( pcl::PointCloud< pcl::PointXYZ >::ConstPtr const & cloud );

		void init_viewer();
	private:
		boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer_;
	};

	void pcl_manager::init_viewer()
	{
		viewer_ = boost::shared_ptr< pcl::visualization::PCLVisualizer >( new \
			pcl::visualization::PCLVisualizer( "3D Viewer" ) );

		viewer_->setBackgroundColor( 0, 0, 0 );
		viewer_->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, \
			1, 0 );
		viewer_->addCoordinateSystem( 1.0 );
		viewer_->initCameraParameters();
	}


	void pcl_manager::view( pcl::PointCloud< pcl::PointXYZ >::ConstPtr const & cloud )
	{
		viewer_->updatePointCloud( cloud, 0 );
		viewer_->spinOnce();	//スクリーン表示の更新
	}

	//depthごとにカラーを添付
	IplImage * convert_depth_to_color( IplImage * depth )
	{
		IplImage * result = cvCreateImage( cvSize(\
			depth->width, depth->height ),\
			IPL_DEPTH_16U, 4 );

		for( int y = 0; y < depth->height; ++y )
		{
			for( int x = 0; x < depth->width; ++x )
			{
				auto const pixel = ( reinterpret_cast< unsigned short * >( \
					( depth->imageData + depth->widthStep * y ) ) )[ x ] / 8;
				auto * pixel_ptr = std::addressof( depth->imageData[ x * 4 +\
					depth->width * y * 4 ] );

				int const b = 0, g = 1, r = 2;

				pixel_ptr[ b ] = 50;
				pixel_ptr[ g ] = 50;
				pixel_ptr[ r ] = 50;

				if( pixel < 650 && pixel >= 400)
				{
					pixel_ptr[ b ] = 0;
					pixel_ptr[ g ]  = static_cast< char >( ( pixel - 400 ) * ( 255.0 / 250.0 ) ); 
					pixel_ptr[ r ] = 255;
				}
				if( pixel < 1300 && pixel >= 650 )
				{
					pixel_ptr[ b ] = 0;
					pixel_ptr[ g ] = 255;
					pixel_ptr[ r ]  =  static_cast< char >( 255 - ( pixel - 650 )* ( 255.0 / 650.0 ) ); 
				}
				if( pixel < 1950 && pixel >= 1300 )
				{
					pixel_ptr[ b ] = static_cast< char >( ( pixel - 1300 ) * ( 255.0 / 650.0 ) );
					pixel_ptr[ g ] = 255;
					pixel_ptr[ r ]  =  0; 
				}
				if( pixel < 2600 && pixel >= 1950 )
				{
					pixel_ptr[ b ] = 255;
					pixel_ptr[ g ] = static_cast< char >( 255 - ( pixel - 1950 )* ( 255.0 / 650.0 ) );
					pixel_ptr[ r ]  = static_cast< char >( ( pixel - 1950 ) * ( 255.0 / 650.0 ) ); 
				}
				if( pixel < 3250 && pixel >= 2600 )
				{
					pixel_ptr[ b ] = 255;
					pixel_ptr[ g ] = 0;
					pixel_ptr[ r ]  = static_cast< char >( 255 - ( pixel - 2600 ) * ( 255.0 / 650.0 ) ); 
				}
				if( pixel < 4000 && pixel >= 3250 )
				{
					pixel_ptr[ b ] = static_cast< char >( 255 - ( pixel - 3250 ) * ( 255.0 / 650.0 ) );
					pixel_ptr[ g ] = static_cast< char >( 255 - ( pixel - 3250 ) * ( 255.0 / 650.0 ) );
					pixel_ptr[ r ]  = static_cast< char >( 255 - ( pixel - 3250 ) * ( 255.0 / 650.0 ) ); 
				}

				if( pixel >= 4000 )
				{
					pixel_ptr[ b ] = 255;
					pixel_ptr[ g ] = 255;
					pixel_ptr[ r ]  = 255; 
				}
			}
		}


		return result;
	}

	//3次元ポイントクラウドのための座標変換
	void convert_color_img_to_point_cloud( IplImage * depth )
	{
		using namespace cv;
		using namespace std;
		
		cv::vector< cv::Mat > channels;

		auto colored_depth = convert_depth_to_color( depth );

		cv::split( colored_depth, channels );

		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr\
			( new pcl::PointCloud< pcl::PointXYZRGB > );
		
		Vector4 point;

		for( int y = 0; y < depth->height; ++y )
		{
			unsigned char * bptr = channels[ 0 ].ptr< unsigned char >( y );
			unsigned char * gptr = channels[ 1 ].ptr< unsigned char >( y );
			unsigned char * rptr = channels[ 2 ].ptr< unsigned char >( y );

			for( int x = 0; x < depth->width; ++x )
			{
				//座標をSkelton座標系 == 直交座標系に変換
				auto point = NuiTransformDepthImageToSkeleton(\
					x, y, static_cast< unsigned short >( \
					depth->imageData[ x + y * depth->widthStep ] ) );

				//Convert to PCL cloud
				pcl::PointXYZRGB basic_point;
				basic_point.x = point.x * 1000;
				basic_point.y = point.y * 1000;
				basic_point.z = point.z * 1000;
				
				basic_point.b = bptr[ x ];
				basic_point.g = gptr[ x ];
				basic_point.r = rptr[ x ];

				if( static_cast< short >( \
					depth->imageData[ x + y * depth->widthStep ] ) != 0 )
				{
					cloud_ptr->points.push_back( basic_point );
				}
			}
		}
	}

	void convert_depth_img_to_point_cloud( IplImage * color )
	{
		cv::vector< cv::Mat > channels;

		cv::split( color, channels );

		pcl::PointCloud< pcl:: PointXYZRGB >::Ptr cloud_ptr( new pcl::PointCloud<\
			pcl::PointXYZRGB > );

		for( int y = 0; y < color->height; ++y )
		{
			unsigned char * bptr = channels[ 0 ].ptr< unsigned char >( y );
			unsigned char * gptr = channels[ 1 ].ptr< unsigned char >( y );
			unsigned char * rptr = channels[ 2 ].ptr< unsigned char >( y );

			for( int x = 0; x < color->width; ++x )
			{
				pcl::PointXYZRGB basic_point;

				basic_point.x = static_cast< float >( x / 100.0 );
				basic_point.y = static_cast< float >( ( color->height - y ) / 100.0 );
				//輝度が云々でz値変動
				basic_point.z = ( bptr[ x ] + gptr[ x ] + rptr[ x ] ) >( 128 * 3 ) ? 3.0 : 0.0;


				basic_point.b = bptr[ x ];
				basic_point.g = gptr[ x ];
				basic_point.r = rptr[ x ];

				cloud_ptr->points.push_back( basic_point );
			}
		}

		cloud_ptr->width = static_cast< int >( cloud_ptr->points.size() );
		cloud_ptr->height = 1;
	}
		
	void draw_point_cloud()
	{

	}


}






