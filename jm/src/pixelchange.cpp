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
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
 
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

#include <pcl/segmentation/region_growing_rgb.h>


using namespace pcl;
using namespace std;

stringstream ss;
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
	
	void segmentIt() {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudt(new pcl::PointCloud<pcl::PointXYZRGBA>);
                // Read a PCD file from disk.
                if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("empty.pcd", *cloudt) != 0)
                {
                        return;
                }
		string r,g,b,word;
		set<string> obj;
		cv::Mat im_matrix(cloudt->height, cloudt->width, CV_8UC3);
                for (int y = 0; y < im_matrix.rows; y++) {
                   for (int x = 0; x < im_matrix.cols; x++) {
                        pcl::PointXYZRGBA *point;
                        point = &cloudt->at(x, y);

                        Eigen::Vector3i rgb = point->getRGBVector3i();

                        int rr = rgb[2];
                        int gg = rgb[1];
                        int bb = rgb[0];
			ostringstream convert;
			convert << rr << "-" << gg << "-" << rr;
			word = convert.str();
			obj.insert(word);
		    }
		}
/*
		set<string>::iterator it;
  		for (it = obj.begin(); it != obj.end(); it++) {
      		//	cout << *it << " ";
  		}

		cout << "    " << word << endl;
		if (obj.find("255-0-0") != obj.end()) { 
			cout << "found" << endl;
		} else {
			cout << "it is not found" << endl;
		}
*/
               pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
                // Read a PCD file from disk.
                if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(ss.str(), *cloud) != 0)
                {
                        return;
                }
                string wordc;
                set<string> objColors;
                cv::Mat im_matrix1(cloud->height, cloud->width, CV_8UC3);
                for (int y = 0; y < im_matrix1.rows; y++) {
                   for (int x = 0; x < im_matrix1.cols; x++) {
                        pcl::PointXYZRGBA *point;
                        point = &cloud->at(x, y);

                        Eigen::Vector3i rgb = point->getRGBVector3i();

                        int rr = rgb[2];
                        int gg = rgb[1];
                        int bb = rgb[0];
                        ostringstream convert;
                        convert << rr << "-" << gg << "-" << rr;
                        wordc = convert.str();
			if (obj.find(wordc) != obj.end()) {
			//	cout << wordc<< " is found "<<endl;
			} else {
                           objColors.insert(word);
			}
                    }
                }

		cout << "New Pixels " << endl;
               set<string>::iterator it;
                for (it = objColors.begin(); it != objColors.end(); it++) {
                      cout << *it << " ";
                }

	
		exit(0);


	}
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() );
  		pcl::fromROSMsg (*input, *cloudPtr);
		
		cloud = *cloudPtr;
		pcl::io::savePCDFileASCII(ss.str(), cloud);
		/*
                int x, y;
		pcl::PointXYZRGBA p; 
		pcl::PointCloud<pcl::PointXYZRGBA> cld; 

		for (size_t i = 0; i < cloud.points.size (); ++i) {
		       p.x = cloud.points[i].x;
                       p.y = cloud.points[i].y;
                       p.z = cloud.points[i].z;
                       p.r = cloud.points[i].r;
                       p.g = cloud.points[i].g;
                       p.b = cloud.points[i].b;
		       cld.points.push_back(p); 
		}
	  	cld.width = cld.points.size();
                cld.height = 1;
		cld.is_dense = true;
		cld.points.resize (cld.width * cld.height);

		ss.clear();
	        ss << "test_1.pcd";

    		
//	        pcl::PCDWriter::write ("test_1.pcd", cld, false);
		
//		string fName = "ok_pcd.pcd";
	
		pcl::io::savePCDFileASCII(ss.str(), cld);
		std::cerr << "Saved " << cld.points.size () << " data points to test_pcd.pcd." << std::endl;
*/
		segmentIt();

/*		cv::Mat im_matrix(cloud.height, cloud.width, CV_8UC3);
                for (int y = 0; y < im_matrix.rows; y++) {
	           for (int x = 0; x < im_matrix.cols; x++) {
                    	pcl::PointXYZRGBA *point;
                        point = &cloud.at(x, y);

			Eigen::Vector3i rgb = point->getRGBVector3i();
			
			// im_matrix is (rows, columns)
			im_matrix.at<cv::Vec3b>(y,x)[0] = rgb[2];
			im_matrix.at<cv::Vec3b>(y,x)[1] = rgb[1];
			im_matrix.at<cv::Vec3b>(y,x)[2] = rgb[0];
                        int rr = rgb[2];
			int gg = rgb[1];
			int bb = rgb[0];
			if ((rr - gg) > 10 or ((gg - bb) > 10)){
                //        	cout << rgb[2] << " " << rgb[1] << " " << rgb[0] << endl;
 			}
		   }
		}
		cout << "over" << endl;
*/
	}
	
};

int  main (int argc, char** argv)
 {
   ss << "newobject.pcd";

   ros::init (argc, argv, "pointcloud_to_pcd");

   PointCloudToPCD b;
   ros::spin ();

   return (0);
  }

