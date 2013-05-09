#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include <fstream>

struct pcl_manager
{
public:
	boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer_;

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
		convert_RGB_and_depth_to_cloud( cv::Ptr< IplImage const > const & color, \
		cv::Ptr< IplImage const > const & depth )
	{
		pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_ptr
			( new pcl::PointCloud< pcl::PointXYZRGB > );

		for( int y = 0; y < color->height; ++y )
		{
			for( int x = 0; x < color->width; ++x )
			{
				pcl::PointXYZRGB basic_point;

				basic_point.x = x;
				basic_point.y = y;
				basic_point.z = ( depth->imageData + x * depth->widthStep )[ y ];

				basic_point.r = ( color->imageData + x + 2 )[ y ];
				basic_point.g = ( color->imageData + x + 1 )[ y ];
				basic_point.b = ( color->imageData + x + 0 )[ y ];


				cloud_ptr->points.push_back( basic_point );
			}
		}

		cloud_ptr->width = static_cast< int >( cloud_ptr->points.size() );
		cloud_ptr->height = 1;

		return cloud_ptr;
	}

	void update( pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr const & cloud, std::string const & viewer_id )
	{
		viewer_->updatePointCloud( cloud, viewer_id );
		viewer_->spinOnce();	//ï`âÊçXêV
	}
private:
};






int main()
{
	pcl_manager pcl;

	std::string c;
	c.

	return 0;
}
