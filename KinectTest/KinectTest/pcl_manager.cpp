#include "pcl_manager.h"
#include <opencv2/opencv.hpp>
#include <NuiApi.h>
#pragma comment( lib, "pcl_visualization_debug.lib" )


namespace pcl_manager
{

	class pcl_manager
	{
	public:
		void view( pcl::PointCloud< pcl::PointXYZ >::ConstPtr const & cloud );
	private:
		pcl::visualization::CloudViewer viewer_;
	};

	void pcl_manager::view( pcl::PointCloud< pcl::PointXYZ >::ConstPtr const & cloud )
	{
		if( ! viewer_.wasStopped() )
			viewer_.showCloud( cloud );
	}

	//depthごとにカラーを添付
	IplImage * convert_depth_to_color( IplImage * depth )
	{
		IplImage * result = cvCreateImage( cvSize(\
			depth->width, depth->height ),\
			IPL_DEPTH_16U, 4 );

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






