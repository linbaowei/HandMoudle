#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/visualization/pcl_plotter.h>
#include <opencv2/core/core.hpp>
#include <pcl/common/pca.h>
#include <iostream>
using namespace std;

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <time.h>
#include <math.h>
#include <ANN/ANN.h>
#include <fstream>

#include <time.h>

#include <cv.h>
#include <highgui.h>

#include "Geomentry.h"

#define pointcloudnum 500
typedef pcl::PointXYZRGBNormal PointType;

void paramwrite();
void paramRandomWrite(int count);
pcl::PointCloud<PointType> paramread(string filename);
pcl::PointCloud<PointType> generatecylinder(float radius,float length);
pcl::PointCloud<PointType> generatecylinderfirstjoint(float radius,float length);
pcl::PointCloud<PointType> generateSphere(float radius);
float angle2deg(float angle);
Eigen::Affine3f rotationzy(float theta);
Eigen::Affine3f rotation01(float theta);
pcl::PointCloud<PointType> estimateTatchingPointsAndNormals(pcl::PointCloud<PointType> pointcloud);
bool saveDescriptor(pcl::PointCloud<PointType> handmoudle, string filename);
bool loadDescriptor(vector <vector <float> > & Descriptor, string filename);
vector <float> NN(vector<float> a, vector< vector<float> > allfeatures);
float Dist(vector<float> a, vector<float> b);