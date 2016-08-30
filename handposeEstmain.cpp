#include "handmoudle.h"
#include <pcl/filters/voxel_grid.h>
#include <pthread.h>
#include <omp.h>

#define threadNUM 4
vector<pair<float, int> > result;
vector <vector <float> > objectDescriptor;
vector< vector <vector <float> > > allhands;
vector < vector <pair<Eigen::Vector3f, Eigen::Vector3f> > >alltripoints;
vector<vector<pair<float, float> > > allhandpotFeatures(pointcloudnum+1);
pthread_mutex_t mylock=PTHREAD_MUTEX_INITIALIZER;
static int cmp( pair<float, int >  i, pair<float, int > j){
	if( i.first > j.first)
		return 0;
	else
		return 1;
}
static int cmp2( pair<float, float >  i, pair<float, float > j){
	if( i.first > j.first)
		return 0;
	else
		return 1;
}

int main(int argc, char **argv) {
  
  Geomentry geo;
  for(int i = 0; i <= pointcloudnum; ++ i)
  {
    string num = geo.numToString(i);  
    string pointcloudfiles ="./pointclouds/" + num+ ".ply";  
    string descriptorfiles ="./pointclouds/" + num+ ".des";
    string tripointsfiles ="./pointclouds/" + num+ ".tri";
    vector <vector <float> > Descriptor;
    loadDescriptor(Descriptor, descriptorfiles);
    allhands.push_back(Descriptor);
    vector <pair<Eigen::Vector3f, Eigen::Vector3f> > tripoints;
    loadTrianglepointsfiles(tripoints, tripointsfiles);
    alltripoints.push_back(tripoints);
    
    vector<pair<float, float> > ei(3); 
    Eigen::Vector3f centerpoi = tripoints.at(0).first + tripoints.at(1).first + tripoints.at(2).first;
    centerpoi /= 3;
    Eigen::Vector3f centernormal = geo.CrossProduct(tripoints.at(0).first - tripoints.at(1).first, tripoints.at(0).first - tripoints.at(2).first);
    ei[0].first = (centerpoi - tripoints.at(0).first).norm();
    ei[1].first = (centerpoi - tripoints.at(1).first).norm();
    ei[2].first = (centerpoi - tripoints.at(2).first).norm();
    float angle1 = geo.VectorAngle(centernormal, tripoints.at(0).second);
    ei[0].second = angle1 < M_PI/2 ? angle1 : angle1 - M_PI/2;
    float angle2 = geo.VectorAngle(centernormal, tripoints.at(1).second);
    ei[1].second = angle2 < M_PI/2 ? angle2 : angle2 - M_PI/2;
    float angle3 = geo.VectorAngle(centernormal, tripoints.at(2).second);
    ei[2].second = angle3 < M_PI/2 ? angle3 : angle3 - M_PI/2;
    sort(ei.begin(), ei.end(), cmp2);
    allhandpotFeatures[i] = ei;
    //cout << i << endl;
  }
  cerr << "load *.des files successfully!" << endl;
  
  
  
  if(argc < 2)
  {
    cerr << argv[0]<< " cup.ply" << endl;
    return 1;
  }
  string pointcloudstr = argv[1];
  cout << "Loading point cloud File name: " << pointcloudstr << endl;
  
  pcl::PointCloud<PointType> pointcloud = geo.readPointCloud(pointcloudstr);
  pcl::PointCloud<pcl::Normal> pointcloudnormal;
  pointcloudnormal.resize(pointcloud.size()); 
  for(int i = 0; i < pointcloud.size(); ++i)
  {
    pointcloudnormal.at(i).normal_x = pointcloud.at(i).normal_x;
    pointcloudnormal.at(i).normal_y = pointcloud.at(i).normal_y;
    pointcloudnormal.at(i).normal_z = pointcloud.at(i).normal_z;
    pointcloud.at(i).x *= 0.18;
    pointcloud.at(i).y *= 0.18;
    pointcloud.at(i).z *= 0.18;
  }
  pcl::visualization::PCLVisualizer mainview("pointcloud");  
  mainview.setPosition(0,0);	
  mainview.setBackgroundColor(0.9, 0.9, 0.9);
  //pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(pointcloud.makeShared(), 108, 166, 205);
  pcl::visualization::PointCloudColorHandlerRGBField<PointType> object_color(pointcloud.makeShared());
  mainview.addPointCloud (pointcloud.makeShared(), object_color, "objectpointcloud");
  mainview.addPointCloudNormals<PointType, pcl::Normal>(pointcloud.makeShared(), pointcloudnormal.makeShared(), 10, 30, "normal");
  mainview.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "objectpointcloud");
  mainview.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "normal");
  
  
  
  std::cerr << "PointCloud before filtering: " << pointcloud.width * pointcloud.height 
       << " data points (" << pcl::getFieldsList (pointcloud) << ")." << endl;
  pcl::PointCloud<PointType> cloud_filtered;
  // Create the filtering object
  pcl::VoxelGrid< PointType > sor;
  sor.setInputCloud (pointcloud.makeShared());
  sor.setLeafSize (15, 15, 15);
  sor.filter (cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
       << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << endl;
       
//   pcl::PointCloud<pcl::Normal> pointcloudnormalfiltered;
//   pointcloudnormalfiltered.resize(cloud_filtered.size()); 
//   for(int i = 0; i < cloud_filtered.size(); ++i)
//   {
//     pointcloudnormalfiltered.at(i).normal_x = cloud_filtered.at(i).normal_x;
//     pointcloudnormalfiltered.at(i).normal_y = cloud_filtered.at(i).normal_y;
//     pointcloudnormalfiltered.at(i).normal_z = cloud_filtered.at(i).normal_z;
//   }
//   pcl::visualization::PCLVisualizer mainviewdown("cloud_filtered");  
//   mainviewdown.setPosition(0,0);	
//   mainviewdown.setBackgroundColor(0.9, 0.9, 0.9);
//   pcl::visualization::PointCloudColorHandlerRGBField<PointType> objectfiltered_color(cloud_filtered.makeShared());
//   mainviewdown.addPointCloud (cloud_filtered.makeShared(), objectfiltered_color, "objectpointcloudfiltered");
//   mainviewdown.addPointCloudNormals<PointType, pcl::Normal>(cloud_filtered.makeShared(), pointcloudnormalfiltered.makeShared(), 10, 30, "normalfiltered");
//   mainviewdown.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "objectpointcloudfiltered");
//   mainviewdown.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "normalfiltered");
//   while (!mainviewdown.wasStopped ())
//   {
//     mainviewdown.spinOnce ();
//   }
      
  srand((unsigned)time(NULL));
  int OBJPOINTNUM = 50;
  vector< vector <pair<Eigen::Vector3f, Eigen::Vector3f> > > allobjecttripoints(OBJPOINTNUM);
  vector<vector<pair<float, float> > > allobjpotFeatures(OBJPOINTNUM);
  for(int i = 0; i < OBJPOINTNUM; ++i)
  {    
    int index1 = rand()%(cloud_filtered.size());
    int index2 = rand()%(cloud_filtered.size());
    while(index2==index1)
    {
      index2 = rand()%(cloud_filtered.size());
    }
    int index3 = rand()%(cloud_filtered.size());
    while(index3==index2||index3 == index1)
    {
      index3 = rand()%(cloud_filtered.size());
    }
    
    vector <pair<Eigen::Vector3f, Eigen::Vector3f> > objecttripoints(3);
    objecttripoints[0].first = cloud_filtered.at(index1).getArray3fMap();
    objecttripoints[0].second = cloud_filtered.at(index1).getNormalVector3fMap();
    objecttripoints[1].first = cloud_filtered.at(index2).getArray3fMap();
    objecttripoints[1].second = cloud_filtered.at(index2).getNormalVector3fMap();
    objecttripoints[2].first = cloud_filtered.at(index3).getArray3fMap();
    objecttripoints[2].second = cloud_filtered.at(index3).getNormalVector3fMap();
    allobjecttripoints[i] = objecttripoints;
    
    vector<pair<float, float> > ei(3); 
    Eigen::Vector3f centerpoi = cloud_filtered.at(index1).getArray3fMap() + cloud_filtered.at(index2).getArray3fMap() + cloud_filtered.at(index3).getArray3fMap();
    centerpoi /= 3;
    Eigen::Vector3f centernormal = geo.CrossProduct(cloud_filtered.at(index1).getArray3fMap() - cloud_filtered.at(index2).getArray3fMap(), cloud_filtered.at(index1).getArray3fMap() - cloud_filtered.at(index3).getArray3fMap());
    Eigen::Vector3f point1 = cloud_filtered.at(index1).getArray3fMap();
    ei[0].first = (centerpoi - point1).norm();
    Eigen::Vector3f point2 = cloud_filtered.at(index2).getArray3fMap();
    ei[1].first = (centerpoi - point2).norm();
    Eigen::Vector3f point3 = cloud_filtered.at(index3).getArray3fMap();
    ei[2].first = (centerpoi - point3).norm();
    float angle1 = geo.VectorAngle(centernormal, cloud_filtered.at(index1).getNormalVector3fMap());
    ei[0].second = angle1 < M_PI/2 ? angle1 : angle1 - M_PI/2;
    float angle2 = geo.VectorAngle(centernormal, cloud_filtered.at(index2).getNormalVector3fMap());
    ei[1].second = angle2 < M_PI/2 ? angle2 : angle2 - M_PI/2;
    float angle3 = geo.VectorAngle(centernormal, cloud_filtered.at(index3).getNormalVector3fMap());
    ei[2].second = angle3 < M_PI/2 ? angle3 : angle3 - M_PI/2;
    sort(ei.begin(), ei.end(), cmp2);
    allobjpotFeatures[i] = ei;
  }
         

         
         
  vector < pair <Eigen::Vector3f, Eigen::Vector3f> > objecttouchpoints;  //pair <point, normal>  
  for(int i = 0; i < cloud_filtered.size(); ++ i)
  {
    Eigen::Vector3f point = cloud_filtered.at(i).getArray3fMap();
    Eigen::Vector3f normal = cloud_filtered.at(i).getNormalVector3fMap();
    pair <Eigen::Vector3f, Eigen::Vector3f> tmppair;
    tmppair.first = point;
    tmppair.second = normal;
    objecttouchpoints.push_back(tmppair);   
  }  
  
  for(int j = 0; j < objecttouchpoints.size(); ++ j)
  {
    for(int p = 0; p < objecttouchpoints.size(); ++ p)
    {
      if(j != p)
      {
	Eigen::Vector3f p1p2 = objecttouchpoints.at(j).first - objecttouchpoints.at(p).first;
	float d = p1p2.norm();
	if(d < 30)
	  continue;
	float theta1 = geo.VectorAngle(objecttouchpoints.at(j).second, p1p2);
	if(theta1 > M_PI/2)
	  theta1 -= M_PI/2;
	float theta2 = geo.VectorAngle(objecttouchpoints.at(p).second, p1p2);
	if(theta2 > M_PI/2)
	  theta2 -= M_PI/2;
	vector<float> tmpdes(3);
	tmpdes[0] = d;
	tmpdes[1] = theta1;
	tmpdes[2] = theta2;
	//cerr << tmpdes[0] << " " << tmpdes[1] << " " << tmpdes[2] << endl;
	objectDescriptor.push_back(tmpdes);
      }
    }
  }
  

  cout << "----------------------------------------" << endl;
  int64 start=0,end=0;  
  start = cv::getTickCount();    
    
  
  //i: hand pose num
  #pragma omp parallel for 
  for(int i = 0; i < allhands.size(); ++ i)
  {
    if(i%100 == 0)
       cout << i << endl;
    float Ei_j = 0; 
    float w_i = 0;
    //j: feature num of hand pose i
    for(int j = 0; j < allhands.at(i).size(); ++ j)
    {
      float diserror = 10e10;
      int count_denominator = 0;
      int count_numerator = 0;
      //vector <float> nnfeature = NN2(allhands.at(i).at(j), objectDescriptor, 0.5, diserror, count_numerator);
      vector<float> nnfeature = NN(allhands.at(i).at(j), objectDescriptor, diserror);
      float tmp_E = Dist(allhands.at(i).at(j), nnfeature);
      Ei_j += tmp_E;
    }
    Ei_j /= (allhands.at(i).size());
    if(isnan(Ei_j))
      continue;
    pair<float, int> tmpres;
    tmpres.first = Ei_j;
    tmpres.second = i;
    result.push_back(tmpres);
  }
  
  
  
  
  sort(result.begin(), result.end(), cmp);
  for(int i = 0; i < result.size(); ++i)
  {
    cout << result.at(i).first << " " << i << " " << result.at(i).second << endl;
  }
  vector<pair<float, int> >::iterator iter,iterend;
  for (iter = result.begin(), iterend = result.end(); iter != iterend; ++iter)
  {
    
    if(isnan(iter->first))
    {
      result.erase(iter);
    }
  }
  for (iter = result.begin(), iterend = result.end(); iter != iterend; ++iter)
  {
    if(abs(iter->first - 0) < 0.000001)
    {
      result.erase(iter);
    }
  }
  for (iter = result.begin(), iterend = result.end(); iter != iterend; ++iter)
  {
    if(iter->second > pointcloudnum)
    {
      result.erase(iter);
    }
  }
  
  for(int i = 0; i < 20; ++ i)
  {
    pcl::visualization::PCLVisualizer mainviewhand("pointcloud"+i);  
    mainviewhand.setPosition(0,0);	
    mainviewhand.setBackgroundColor(0.9, 0.9, 0.9);
    string handposenum = geo.numToString(result.at(i).second);
    string pointcloudfile ="./pointclouds/"+handposenum+".ply";
    pcl::PointCloud<PointType> handpointcloud0 = geo.readPointCloud(pointcloudfile);
    
    /*Eigen::Affine3f transform_handpointcloud0 = Eigen::Affine3f::Identity();     
    transform_handpointcloud0.translation() << 50.0, i * 100, 0.0;  
    transform_handpointcloud0.rotate (Eigen::AngleAxisf (angle2deg(0), Eigen::Vector3f::UnitZ())); 
    pcl::transformPointCloud (handpointcloud0, handpointcloud0, transform_handpointcloud0); */ 
  
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> handpointcloud0_color(handpointcloud0.makeShared());
    mainviewhand.addPointCloud(handpointcloud0.makeShared(), handpointcloud0_color, "handpointcloud0"+handposenum);
    mainviewhand.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "handpointcloud0"+handposenum);
    mainviewhand.spinOnce ();

  }
  end = cv::getTickCount();  
  cout << "The differences: " << 1000.0*(end - start)/cv::getTickFrequency()<<" ms"<< endl; 
  while (!mainview.wasStopped ())
  {
    mainview.spinOnce ();
  }
  return 1;
}