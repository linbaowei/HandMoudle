#include "handmoudle.h"

int main(int argc, char **argv) {
  
  paramwrite();
  paramRandomWrite(pointcloudnum);
  
  for(int i = 0; i <= pointcloudnum; ++i)
  {
    Geomentry geo;
    string num = geo.numToString(i);
    string filename ="./yaml/" + num+ ".yaml";  
    pcl::PointCloud<PointType> handmoudle = paramread(filename);    
    
    string pointcloudfiles ="./pointclouds/" + num+ ".ply";
    pcl::io::savePLYFileASCII<PointType>(pointcloudfiles, handmoudle);
    
    string descriptorfiles ="./pointclouds/" + num+ ".des";
    saveDescriptor(handmoudle, descriptorfiles);
    
//     pcl::PointCloud<pcl::Normal> handmoudlenormal;
//     handmoudlenormal.resize(handmoudle.size()); 
//     for(int i = 0; i < handmoudle.size(); ++i)
//     {
//       handmoudlenormal.at(i).normal_x = handmoudle.at(i).normal_x;
//       handmoudlenormal.at(i).normal_y = handmoudle.at(i).normal_y;
//       handmoudlenormal.at(i).normal_z = handmoudle.at(i).normal_z;
//     }
//     pcl::visualization::PCLVisualizer mainview("hand");  
//     mainview.setPosition(0,0);	
//     mainview.setBackgroundColor(0,0,0);
//     pcl::visualization::PointCloudColorHandlerRGBField<PointType> handmoudle_color(handmoudle.makeShared());
//     mainview.addPointCloud (handmoudle.makeShared(), handmoudle_color, "handmoudle");  
//     mainview.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "handmoudle");
//     mainview.addPointCloudNormals<PointType, pcl::Normal>(handmoudle.makeShared(), handmoudlenormal.makeShared(), 1, 10, "normal2");
//     mainview.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normal2");
//   
//     mainview.addCoordinateSystem(10);
//     while (!mainview.wasStopped ())
//     {
//       mainview.spinOnce ();
//     }
  }
  return 0;
}