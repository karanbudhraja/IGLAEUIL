 // ROS core
 #include <ros/ros.h>
 // PCL includes
 #include <pcl/io/io.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/point_cloud.h>

 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>

using namespace std;

/* ---[ */
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
		//void cloudCb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input){
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
  		pcl::fromROSMsg (*input, cloud);
		 pcl::io::savePCDFileBinary("test_pcd.pcd", cloud);
  		std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
	}
};

int  main (int argc, char** argv)
 {
   ros::init (argc, argv, "pointcloud_to_pcd");

   PointCloudToPCD b;
   ros::spin ();

   return (0);
  }

