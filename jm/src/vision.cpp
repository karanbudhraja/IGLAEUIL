#include "vision.h"

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
// PCL includes
 #include <pcl/io/io.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/point_cloud.h>
 #include <cv_bridge/cv_bridge.h>
 #include <cmath>
 #include <vector>
using namespace pcl;
using namespace std;
using namespace cv;

int extractPoints(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudt, cv::Mat& rgbPts1) {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;

        cloud = *cloudt;
        int noPts = 0;
        pcl::PointXYZRGBA p;

//      cv::Mat allPts (1,number_of_lines, 6);

        for (size_t i = 0; i < cloud.points.size (); ++i) {
           p.x = cloud.points[i].x;
           p.y = cloud.points[i].y;
           p.z = cloud.points[i].z;
           p.r = cloud.points[i].r;
           p.g = cloud.points[i].g;
           p.b = cloud.points[i].b;
                       
 	   int r = (int)p.r;
           int b = (int)p.b;
           int g = (int)p.g;
           if ((r - g) > 25 or ((g - b) > 25)){
              rgbPts1.at<float>(noPts,0) = (float)r;
              rgbPts1.at<float>(noPts,1) = (float)g;
              rgbPts1.at<float>(noPts,2) = (float)b;
              noPts++;
           }
          /*                      allPts.at<cv::Vec3b>(1,j)[0] = x;
                                allPts.at<cv::Vec3b>(1,j)[1] = y;
                                allPts.at<cv::Vec3b>(1,j)[2] = z;
                                allPts.at<cv::Vec3b>(1,j)[3] = rr;
                                allPts.at<cv::Vec3b>(1,j)[4] = gg;
                                allPts.at<cv::Vec3b>(1,j)[5] = bb;
            */     //               allPts.at<cv::Vec6b>(1,j)[1] = rr;


      }


   return noPts;
}

void kMeansExec(cv::Mat& rgbPts,cv::Mat& labels,cv::Mat& centers) {

   int clusterCount = 2;
   int attempts = 5;
   cv::kmeans(rgbPts, clusterCount, labels, TermCriteria(TermCriteria::COUNT, 100, 1), attempts, KMEANS_RANDOM_CENTERS, centers );
     cout << "K Means Centrods :: ";
     cerr << centers<< endl;
}

int findColorCentroid(cv::Mat& centers) {
   int rows = centers.rows;
   int cols = centers.cols;
   float cent[rows][cols];
   float sm[rows];
   for (int i = 0; i < rows; i++ ) {
      float largest = centers.at<float>(i,0);
      for(int j = 1; j < cols ; j++) 
         if(centers.at<float>(i,j) > largest or  largest == 0)
            largest = centers.at<float>(i,j);

      sm[i] =  centers.at<float>(i,0) / largest;
      for(int j = 0; j < cols ; j++) {
         cent[i][j] =  centers.at<float>(i,j) / largest;
         if(sm[i] > cent[i][j])
            sm[i] = cent[i][j];
      }  
   }
   float smallest = sm[0];
   int index = 0;
   for (int ij = 1 ; ij < rows; ij++) 
      if(sm[ij] < smallest) {
         smallest = sm[ij];
         index = ij;
      }
   return index;
   
}

vector<int> kMeans(string ss) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudt(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // Read a PCD file from disk.
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(ss, *cloudt) != 0){
                        return {};
        }
        cv::Mat rgbPts1 (cloudt->points.size (),3, CV_32F);

        int noPts = extractPoints(cloudt,rgbPts1);

        cv::Mat rgbPts (noPts,3, CV_32F);
        for (int ij = 0 ; ij < noPts; ij++) {
           for (int ch = 0 ; ch < 3 ; ch++ )
              rgbPts.at<float>(ij,ch)  = rgbPts1.at<float>(ij,ch);
        }
   	cv::Mat labels;
   	cv::Mat centers;
        kMeansExec(rgbPts,labels,centers);

	int index = findColorCentroid(centers);
        for(size_t i = 0; i < cloudt->points.size (); ++i) {
       	   int idx = labels.at<int>(i,0);
           if(idx == 0) {
  	  //    cout << centers.at<float>(idx, 0) << " " << centers.at<float>(idx, 1) << " " << centers.at<float>(idx, 2) << endl;


       	   }
        }

/// Color Attributes////
        int rgbValues[centers.cols];
        cout << "R G B :: ";
        for (int ij = 0; ij < centers.cols; ij++) {
           rgbValues[ij] = int(centers.at<float>(index,ij));
           cout << rgbValues[ij] << " ";
        }
        cout << endl;
        vector<int> rgb(&rgbValues[0], &rgbValues[0]+centers.cols);
        return rgb;
}

vector<int> getDepthParameters(string s) {
   return kMeans(s);
}


