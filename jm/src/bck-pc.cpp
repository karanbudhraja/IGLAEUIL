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

 std::stringstream ss;
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
	
	void segmentation(pcl::PointCloud<pcl::PointXYZRGBA> clouddata) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (&clouddata);
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ece; 
	std::vector<pcl::PointIndices> clusters;
	ece.setInputCloud(cloud);
//	ece.setMinClusterSize (10);
//	ece.setClusterTolerance (0.05); 
		// kd-tree object for searches.
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	kdtree->setInputCloud(cloud);	

	// Euclidean clustering object.
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clustering;
	// Set cluster tolerance to 2cm (small values may cause objects to be divided
	// in several clusters, whereas big values may join objects in a same cluster).
		clustering.setClusterTolerance(0.02);
	// Set the minimum and maximum number of points that a cluster can have.
		clustering.setMinClusterSize(100);
		clustering.setMaxClusterSize(25000);
		clustering.setSearchMethod(kdtree);
		clustering.setInputCloud(cloud);
		clustering.extract(clusters);
 
	// For every cluster...
		int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
			// ...add all its points to a new cloud...
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
			for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				cluster->points.push_back(cloud->points[*point]);
			cluster->width = cluster->points.size();
			cluster->height = 1;
			cluster->is_dense = true;
 
		// ...and save it to disk.
			if (cluster->points.size() <= 0)
				break;
		//	std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		//	std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		//	pcl::io::savePCDFileASCII(fileName, *cluster);
 
			currentClusterNum++;
		}
	

	}


	void euclidCluster(pcl::PointCloud<pcl::PointXYZRGBA> cloud) {
 		sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (&cloud);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

		/* Creating the KdTree from input point cloud*/
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud (input_cloud);
		/* Here we are creating a vector of PointIndices, which contains the actual index
		 * information in a vector<int>. The indices of each detected cluster are saved here.
		 * Cluster_indices is a vector containing one instance of PointIndices for each detected 
		 * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
		 */
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		ec.setClusterTolerance (0.06); 
		ec.setMinClusterSize (30);
		ec.setMaxClusterSize (600);
		ec.setSearchMethod (tree);
		ec.setInputCloud (input_cloud);

		/* Extract the clusters out of pc and save indices in cluster_indices.*/
		ec.extract (cluster_indices);

		/* To separate each cluster out of the vector<PointIndices> we have to 
		 * iterate through cluster_indices, create a new PointCloud for each 
		 * entry and write all points of the current cluster in the PointCloud. 
 		*/
		int i = 1;
		std::vector<pcl::PointIndices>::const_iterator it;
		std::vector<int>::const_iterator pit;
		for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
	        	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
 			for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        			//push_back: add a point to the end of the existing vector
                		cloud_cluster->points.push_back(input_cloud->points[*pit]); 
        		}

 		       //Merge current clusters to whole point cloud
 		   *clustered_cloud += *cloud_cluster;
		   cout << i << endl;
		   i++;
  		}
	}
	void segmentIt() {

        // Object for storing the point cloud.
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // Read a PCD file from disk.
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(ss.str(), *cloud) != 0)
        {
                return;
        }

        // kd-tree object for searches.
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        kdtree->setInputCloud(cloud);

        // Euclidean clustering object.
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clustering;
        // Set cluster tolerance to 2cm (small values may cause objects to be divided
        // in several clusters, whereas big values may join objects in a same cluster).
        clustering.setClusterTolerance(0.02);
        // Set the minimum and maximum number of points that a cluster can have.
        clustering.setMinClusterSize(100);
        clustering.setMaxClusterSize(25000);
        clustering.setSearchMethod(kdtree);
        clustering.setInputCloud(cloud);
        std::vector<pcl::PointIndices> clusters;
        clustering.extract(clusters);

        // For every cluster...
        int currentClusterNum = 1;
        for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
        {
                // ...add all its points to a new cloud...
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
                for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
                        cluster->points.push_back(cloud->points[*point]);
                cluster->width = cluster->points.size();
                cluster->height = 1;
                cluster->is_dense = true;

                // ...and save it to disk.
                if (cluster->points.size() <= 0)
                        break;
                std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
                std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
                pcl::io::savePCDFileASCII(fileName, *cluster);

                currentClusterNum++;
        }

	std::cout << currentClusterNum << endl;
	}
		//void cloudCb (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& input){
	void cloudCb (const sensor_msgs::PointCloud2ConstPtr& input){
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() );
  		pcl::fromROSMsg (*input, *cloudPtr);
		
		cloud = *cloudPtr;
                int x, y;
		pcl::PointXYZRGBA p; 
		pcl::PointCloud<pcl::PointXYZRGBA> cld; 

		for (size_t i = 0; i < cloud.points.size (); ++i) {
		       p.x = cloud.points[i].x;
                       p.y = cloud.points[i].y;
                       p.z = cloud.points[i].z;
                       p.r = (uint8_t)cloud.points[i].r;
                       p.g = (uint8_t)cloud.points[i].g;
                       p.b = (uint8_t)cloud.points[i].b;
		       cld.points.push_back(p); 
		}
	  	cld.width = cld.points.size();
                cld.height = 1;
		cld.points.resize (cld.width * cld.height);


	        ss << "test_1.pcd";

    		
//	        pcl::PCDWriter::write ("test_1.pcd", cld, false);
		
//		string fName = "ok_pcd.pcd";
	
		pcl::io::savePCDFileASCII(ss.str(), cld);
		std::cerr << "Saved " << cld.points.size () << " data points to test_pcd.pcd." << std::endl;

		segmentIt();
		cv::Mat im_matrix(cloud.height, cloud.width, CV_8UC3);
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

		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		kdtree.setInputCloud (cloudPtr);

		pcl::PointXYZ searchPoint;

		searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  		searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  		searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);


		pcl::IndicesPtr indices (new std::vector <int>);
		pcl::PassThrough<pcl::PointXYZRGBA> pass;
		pass.setInputCloud (cloudPtr);
  		pass.setFilterFieldName ("z");
  		pass.setFilterLimits (0.0, 1.0);
  		pass.filter (*indices);

		pcl::RegionGrowingRGB<pcl::PointXYZRGBA> reg;
		reg.setInputCloud (cloudPtr);
  		reg.setIndices (indices);
//		colorCluster(cloud);
	//	euclidCluster(cloud);
	//	segmentation(cloud);
 //		pcl::io::savePCDFile ("pcd_new.pcd",cloud,false);
	//	 pcl::io::savePCDFileAscii("test_pcd.pcd", cloud);
  //		std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
	}
	
	void colorCluster(pcl::PointCloud<pcl::PointXYZRGBA> clouddata) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(&clouddata);
		// kd-tree object for searches.
	//	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
		//kdtree->setInputCloud(cloud);
//		pcl::search::Search <pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);

 
	}
};

int  main (int argc, char** argv)
 {
   ros::init (argc, argv, "pointcloud_to_pcd");

   PointCloudToPCD b;
   ros::spin ();

   return (0);
  }

