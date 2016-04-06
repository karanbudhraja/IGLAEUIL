 // ROS core
 #include <ros/ros.h>
// PCL includes
 #include <pcl/io/io.h>
 #include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/point_cloud.h>
 #include <image_transport/image_transport.h>


using namespace pcl;
using namespace std;

class  PointCloudToPCD {

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Subscriber pcl_sub_;

	public:
	PointCloudToPCD() : 
	it_(nh_){
// Create a ROS subscriber for the input point cloud, contains XYZ, RGB
	pcl_sub_ = nh_.subscribe ("/kinect2/qhd/points", 1, &PointCloudToPCD::cloudCb, this);
	
	}
	
		//void cloudCb (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& input){
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() );
  		pcl::fromROSMsg (*input, *cloudPtr);
		
		cloud = *cloudPtr;
		pcl::io::savePCDFileASCII("cloudbject.pcd", cloud);
		cout << "Saved cloudobject.pcd" << endl;
	}
	
};

int  main (int argc, char** argv)
 {
   ros::init (argc, argv, "pointcloud_to_pcd");

   PointCloudToPCD b;
   ros::spin ();

   return (0);
  }

