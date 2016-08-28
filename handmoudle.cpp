#include "handmoudle.h"

pcl::PointCloud<PointType> generateSphere(float radius)
{
  pcl::PointCloud<PointType> sphere_cloud;
  int step = 40;
  int r_pc = 0;
  int g_pc = 255;
  int b_pc = 0;
  float px, py, pz;
  for (float phi=0; phi < M_PI; phi+=M_PI/step){
    pz = radius*cos(phi);
    for (float theta=0; theta<2*M_PI;theta+=2*M_PI/step){
      px = radius*sin(phi)*cos(theta);
      py = radius*sin(phi)*sin(theta);
      PointType point;
      point.x = px;
      point.y = py;
      point.z = pz;
      point.r = r_pc;
      point.g = g_pc;
      point.b = b_pc;
      sphere_cloud.push_back(point);
    }
  }
  return sphere_cloud;
}

pcl::PointCloud<PointType> generatecylinder(float radius,float length)
{
  pcl::PointCloud<PointType> cylinder_cloud;
  int step = 40;
  int r_pc = 0;
  int g_pc = 0;
  int b_pc = 255;
  float px, py, pz;
  for (float phi=0; phi <2* M_PI; phi+=2*M_PI/step)
  {
    pz = radius*cos(phi);
    px = radius*sin(phi);
    for(float height = 0; height < length; height += 0.5)
    {
      
      PointType point;
      point.x = px;
      point.y = height;
      point.z = pz;
      point.r = r_pc;
      point.g = g_pc;
      point.b = b_pc;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      if(abs(height- length/2) < 5 && abs(phi - 0) < 0.5 )
      {
	point.r = 255;
	point.g = 0;
	point.b = 0;
      }
      if(abs(height- length/2) < 1 && abs(phi - M_PI) < 0.2)
      {
	point.r = 0;
	point.g = 0;
	point.b = 0;
      }
      cylinder_cloud.push_back(point);
    }      
  }
 
  return cylinder_cloud;
}

pcl::PointCloud<PointType> generatecylinderfirstjoint(float radius,float length)
{
  pcl::PointCloud<PointType> cylinder_cloud;
  int step = 40;
  int r_pc = 0;
  int g_pc = 0;
  int b_pc = 255;
  float px, py, pz;
  for (float phi=0; phi <2* M_PI; phi+=2*M_PI/step)
  {
    pz = radius*cos(phi);
    px = radius*sin(phi);
    for(float height = 0; height < length; height += 0.5)
    {
      
      PointType point;
      point.x = px;
      point.y = height;
      point.z = pz;
      point.r = r_pc;
      point.g = g_pc;
      point.b = b_pc;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      if(abs(length - length/10 - height) < 5 && abs(phi - 0) < 0.5 )
      {
	point.r = 255;
	point.g = 0;
	point.b = 0;
      }
      if(abs(length - length/10 - height) < 1 && abs(phi - M_PI) < 0.2)
      {
	point.r = 0;
	point.g = 0;
	point.b = 0;
      }
      cylinder_cloud.push_back(point);
    }      
  }
 
  return cylinder_cloud;
}

void paramRandomWrite(int count){
  srand((unsigned)time(NULL));
  for(int i = 1; i <= count; ++ i)
  {
    Geomentry geo;
    string num = geo.numToString(i);
    string filename ="./yaml/" + num+ ".yaml";  

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "finger thickness" << 6;
    fs << "joint thickness" << 6.5;
    fs << "hand type" << "right";    
    

    fs << "thumb";
    fs << "{";
    fs << "l0_1" << 62;
    fs << "l1_2" << 26;
    fs << "l2" << 33;
    float r0_1 = rand()%(51-31+1)+31;
    fs << "r0_1" << -r0_1;
    float r0_1_aboutzy = rand()%(65)+0;
    fs << "r0_1_aboutzy" << r0_1_aboutzy;
    float r1_2_aboutzy = rand()%(45)+0;
    fs << "r1_2_aboutzy" << r1_2_aboutzy;
    float r2_aboutzy = rand()%(75)+0;
    fs << "r2_aboutzy" << r2_aboutzy;
    fs << "}";
    
    fs << "index";
    fs << "{";
    fs << "l0_4" << 80;
    fs << "l4_5" << 50;
    fs << "l5_6" << 23;
    fs << "l6" << 20;
    fs << "r0_4" << -20;
    float r4_5_aboutx = rand()%(75)+0;
    fs << "r4_5_aboutx" << r4_5_aboutx;
    float r4_5_aboutz = rand()%(20)+5;
    fs << "r4_5_aboutz" << -r4_5_aboutz;
    float r5_6 = rand()%(75)+0;
    fs << "r5_6" << r5_6;
    float r6 = rand()%(75)+0;
    fs << "r6" << r6;
    fs << "}";
    
    fs << "middle";
    fs << "{";
    fs << "l0_7" << 75;
    fs << "l7_8" << 53;
    fs << "l8_9" << 25;
    fs << "l9" << 23;
    fs << "r0_7" << 0;
    float r7_8_aboutx = rand()%(75)+0;
    fs << "r7_8_aboutx" << r7_8_aboutx;
    float r7_8_aboutz = rand()%(10)+0;
    fs << "r7_8_aboutz" << r7_8_aboutz - 5;
    float r8_9 = rand()%(75)+0;
    fs << "r8_9" << r8_9;
    float r9 = rand()%(75)+0;
    fs << "r9" << r9;
    fs << "}";
    
    fs << "ring";
    fs << "{";
    fs << "l0_10" << 70;
    fs << "l10_11" << 50;
    fs << "l11_12" << 25;
    fs << "l12" << 23;
    fs << "r0_10" << 20;
    float r10_11_aboutx = rand()%(75)+0;
    fs << "r10_11_aboutx" << r10_11_aboutx;
    float r10_11_aboutz = rand()%(20)+5;
    fs << "r10_11_aboutz" << r10_11_aboutz;
    float r11_12 = rand()%(75)+0;
    fs << "r11_12" << r11_12;
    float r12 = rand()%(75)+0;
    fs << "r12" << r12;
    fs << "}";
    
    fs << "pinky";
    fs << "{";
    fs << "l0_13" << 67;
    fs << "l13_14" << 40;
    fs << "l14_15" << 20;
    fs << "l15" << 20;
    fs << "r0_13" << 40;
    float r13_14_aboutx = rand()%(75)+0;
    fs << "r13_14_aboutx" << r13_14_aboutx;
    float r13_14_aboutz = rand()%(40)+5;
    fs << "r13_14_aboutz" << r13_14_aboutz;
    float r14_15 = rand()%(75)+0;
    fs << "r14_15" << r14_15;
    float r15 = rand()%(75)+0;
    fs << "r15" << r15;
    fs << "}";
    
    fs.release();                                 
  }
}

void paramwrite(){
       
	string filename = "./yaml/0.yaml";
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);

        fs << "finger thickness" << 6;
	fs << "joint thickness" << 6.5;
        fs << "hand type" << "right";    
	

	fs << "thumb";
	fs << "{";
	fs << "l0_1" << 62;
        fs << "l1_2" << 26;
	fs << "l2" << 33;
	fs << "r0_1" << -51;
	fs << "r0_1_aboutzy" << 60;
	fs << "r1_2_aboutzy" << 5;
	fs << "r2_aboutzy" << 30;
	fs << "}";
	
	fs << "index";
	fs << "{";
	fs << "l0_4" << 80;
	fs << "l4_5" << 50;
	fs << "l5_6" << 23;
	fs << "l6" << 20;
	fs << "r0_4" << -20;
	fs << "r4_5_aboutx" << 20;
	fs << "r4_5_aboutz" << -10;
	fs << "r5_6" << 45;
	fs << "r6" << 45;
	fs << "}";
	
	fs << "middle";
	fs << "{";
	fs << "l0_7" << 75;
	fs << "l7_8" << 53;
	fs << "l8_9" << 25;
	fs << "l9" << 23;
	fs << "r0_7" << 0;
	fs << "r7_8_aboutx" << 20;
	fs << "r7_8_aboutz" << 0;
	fs << "r8_9" << 45;
	fs << "r9" << 45;
	fs << "}";
	
	fs << "ring";
	fs << "{";
	fs << "l0_10" << 70;
	fs << "l10_11" << 50;
	fs << "l11_12" << 25;
	fs << "l12" << 23;
	fs << "r0_10" << 20;
	fs << "r10_11_aboutx" << 20;
	fs << "r10_11_aboutz" << 10;
	fs << "r11_12" << 45;
	fs << "r12" << 45;
	fs << "}";
	
	fs << "pinky";
	fs << "{";
	fs << "l0_13" << 67;
	fs << "l13_14" << 40;
	fs << "l14_15" << 20;
	fs << "l15" << 20;
	fs << "r0_13" << 40;
	fs << "r13_14_aboutx" << 20;
	fs << "r13_14_aboutz" << 20;
	fs << "r14_15" << 45;
	fs << "r15" << 45;
	fs << "}";
	
        fs.release();                                       // explicit close
        cout << "Write Done." << endl;
}
 
pcl::PointCloud<PointType> paramread(string filename)
{
  cv::FileStorage fs; 
  fs.open(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
      cerr << "Failed to open " << filename << endl;
      exit(1);
  }
  float fingerthickness;
  fs["finger thickness"] >> fingerthickness;
  float jointthickness;
  fs["joint thickness"] >> jointthickness;
  
  float l0_1, l1_2, l2, l3, l0_4, l4_5, l5_6, l6, l0_7, l7_8, l8_9, l9, l0_10, l10_11, l11_12, l12, l0_13, l13_14, l14_15, l15;
  float r0_1, r0_1_aboutzy, r1_2_aboutzy, r2_aboutzy, r0_4, r4_5_aboutx, r4_5_aboutz, r5_6, r6, r0_7, r7_8_aboutx, r7_8_aboutz, r8_9, r9, r0_10, r10_11_aboutx, r10_11_aboutz, r11_12, r12, r0_13, r13_14_aboutx, r13_14_aboutz, r14_15, r15;
  cv::FileNode n = fs["thumb"];
  l0_1 = (float)(n["l0_1"]);	l1_2 = (float)(n["l1_2"]); 			l2 = (float)(n["l2"]);			
  r0_1 = (float)(n["r0_1"]); 	r0_1_aboutzy = (float)(n["r0_1_aboutzy"]); 	r1_2_aboutzy = (float)(n["r1_2_aboutzy"]);	r2_aboutzy = (float)(n["r2_aboutzy"]);	
  n = fs["index"];
  l0_4 = (float)(n["l0_4"]); 	l4_5 = (float)(n["l4_5"]); 			l5_6 = (float)(n["l5_6"]); 			l6 = (float)(n["l6"]);
  r0_4 = (float)(n["r0_4"]); 	r4_5_aboutx = (float)(n["r4_5_aboutx"]); 	r4_5_aboutz = (float)(n["r4_5_aboutz"]); 	r5_6 = (float)(n["r5_6"]);	r6 = (float)(n["r6"]);
  n = fs["middle"];
  l0_7 = (float)(n["l0_7"]); 	l7_8 = (float)(n["l7_8"]); 			l8_9 = (float)(n["l8_9"]); 			l9 = (float)(n["l9"]); 
  r0_7 = (float)(n["r0_7"]); 	r7_8_aboutx = (float)(n["r7_8_aboutx"]); 	r7_8_aboutz = (float)(n["r7_8_aboutz"]); 	r8_9 = (float)(n["r8_9"]);	r9 = (float)(n["r9"]);
  n = fs["ring"];
  l0_10 = (float)(n["l0_10"]); 	l10_11 = (float)(n["l10_11"]); 			l11_12 = (float)(n["l11_12"]); 			l12 = (float)(n["l12"]);
  r0_10 = (float)(n["r0_10"]); 	r10_11_aboutx = (float)(n["r10_11_aboutx"]); 	r10_11_aboutz = (float)(n["r10_11_aboutz"]); 	r11_12 = (float)(n["r11_12"]);	r12 = (float)(n["r12"]);
  n = fs["pinky"];
  l0_13 = (float)(n["l0_13"]); 	l13_14 = (float)(n["l13_14"]); 			l14_15 = (float)(n["l14_15"]); 			l15 = (float)(n["l15"]); 
  r0_13 = (float)(n["r0_13"]);	r13_14_aboutx = (float)(n["r13_14_aboutx"]); 	r13_14_aboutz = (float)(n["r13_14_aboutz"]); 	r14_15 = (float)(n["r14_15"]);	r15 = (float)(n["r15"]);
  
  pcl::PointCloud<PointType> cylinder0_1 = generatecylinderfirstjoint(fingerthickness, l0_1);
  pcl::PointCloud<PointType> cylinder1_2 = generatecylinder(fingerthickness, l1_2);
  pcl::PointCloud<PointType> cylinder2 = generatecylinder(fingerthickness, l2);
  Eigen::Affine3f transform_selfrotation = Eigen::Affine3f::Identity();
  transform_selfrotation.rotate (Eigen::AngleAxisf (angle2deg(-150), Eigen::Vector3f::UnitY())); 
  pcl::transformPointCloud (cylinder0_1, cylinder0_1, transform_selfrotation);
  pcl::transformPointCloud (cylinder1_2, cylinder1_2, transform_selfrotation);
  pcl::transformPointCloud (cylinder2, cylinder2, transform_selfrotation);
  
  pcl::PointCloud<PointType> cylinder0_4 = generatecylinderfirstjoint(fingerthickness, l0_4);
  pcl::PointCloud<PointType> cylinder4_5 = generatecylinder(fingerthickness, l4_5);
  pcl::PointCloud<PointType> cylinder5_6 = generatecylinder(fingerthickness, l5_6);
  pcl::PointCloud<PointType> cylinder6 = generatecylinder(fingerthickness, l6);

  pcl::PointCloud<PointType> cylinder0_7 = generatecylinderfirstjoint(fingerthickness, l0_7);
  pcl::PointCloud<PointType> cylinder7_8 = generatecylinder(fingerthickness, l7_8);
  pcl::PointCloud<PointType> cylinder8_9 = generatecylinder(fingerthickness, l8_9);
  pcl::PointCloud<PointType> cylinder9 = generatecylinder(fingerthickness, l9);

  pcl::PointCloud<PointType> cylinder0_10 = generatecylinderfirstjoint(fingerthickness, l0_10);
  pcl::PointCloud<PointType> cylinder10_11 = generatecylinder(fingerthickness, l10_11);
  pcl::PointCloud<PointType> cylinder11_12 = generatecylinder(fingerthickness, l11_12);
  pcl::PointCloud<PointType> cylinder12 = generatecylinder(fingerthickness, l12);

  pcl::PointCloud<PointType> cylinder0_13 = generatecylinderfirstjoint(fingerthickness, l0_13);
  pcl::PointCloud<PointType> cylinder13_14 = generatecylinder(fingerthickness, l13_14);
  pcl::PointCloud<PointType> cylinder14_15 = generatecylinder(fingerthickness, l14_15);
  pcl::PointCloud<PointType> cylinder15 = generatecylinder(fingerthickness, l15);

  pcl::PointCloud<PointType> sphere0 = generateSphere(jointthickness);
  
  Eigen::Affine3f transform_0_1 = Eigen::Affine3f::Identity();     
  Eigen::Affine3f transform_0_1_selfrotation = Eigen::Affine3f::Identity(); 
  transform_0_1.translation() << 0.0, 0.0, 0.0;  
  transform_0_1.rotate (Eigen::AngleAxisf (angle2deg(r0_1), Eigen::Vector3f::UnitZ())); 
  transform_0_1_selfrotation.translation() << 0.0, 0.0, 0.0;  
  //transform_0_1_selfrotation.rotate (Eigen::AngleAxisf (angle2deg(r0_1), Eigen::Vector3f::UnitY())); 
  Eigen::Affine3f MATr0_1_aboutzy = rotation01(angle2deg(r0_1_aboutzy));
  transform_0_1.matrix().block(0,0,3,3) = MATr0_1_aboutzy.rotation() * transform_0_1.rotation() * transform_0_1_selfrotation.rotation();
  pcl::PointCloud<PointType> cylinder0_1_transformed_cloud;
  pcl::transformPointCloud (cylinder0_1, cylinder0_1_transformed_cloud, transform_0_1);
  Eigen::Vector3f pos1;
  pos1[0] = 0.0; pos1[1] = l0_1; pos1[2] = 0.0;
  pos1 = transform_0_1 * pos1;

  Eigen::Affine3f transform_1_2  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder1_2_transformed_cloud;
  pcl::PointCloud<PointType> sphere1;
  transform_1_2.translation() << pos1[0], pos1[1], pos1[2];     
  Eigen::Affine3f MATr1_2_aboutzy = rotationzy(angle2deg(r1_2_aboutzy));
  transform_1_2.matrix().block(0,0,3,3) = transform_0_1.rotation() * MATr1_2_aboutzy.rotation();
  pcl::transformPointCloud (cylinder1_2, cylinder1_2_transformed_cloud, transform_1_2);
  pcl::transformPointCloud (sphere0, sphere1, transform_1_2);
  Eigen::Vector3f pos2;
  pos2[0] = 0.0; pos2[1] = l1_2; pos2[2] = 0.0;
  pos2 = transform_1_2 * pos2;
 
  Eigen::Affine3f transform_2  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder2_transformed_cloud;
  pcl::PointCloud<PointType> sphere2;
  transform_2.translation() << pos2[0], pos2[1], pos2[2];  
  Eigen::Affine3f MATr_2_aboutzy = rotationzy(angle2deg(r2_aboutzy));
  transform_2.matrix().block(0,0,3,3) = transform_1_2.rotation() * MATr_2_aboutzy.rotation();
  pcl::transformPointCloud (cylinder2, cylinder2_transformed_cloud, transform_2);
  pcl::transformPointCloud (sphere0, sphere2, transform_2);
  
  
  Eigen::Affine3f transform_0_4 = Eigen::Affine3f::Identity();     
  transform_0_4.translation() << 0.0, 0.0, 0.0;  
  transform_0_4.rotate (Eigen::AngleAxisf (angle2deg(r0_4), Eigen::Vector3f::UnitZ()));   
  pcl::PointCloud<PointType> cylinder0_4_transformed_cloud;
  pcl::transformPointCloud (cylinder0_4, cylinder0_4_transformed_cloud, transform_0_4);
  Eigen::Vector3f pos4;
  pos4[0] = 0.0; pos4[1] = l0_4; pos4[2] = 0.0;
  pos4 = transform_0_4 * pos4;
  
  Eigen::Affine3f transform_4_5  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder4_5_transformed_cloud;
  pcl::PointCloud<PointType> sphere4;
  transform_4_5.translation() << pos4[0], pos4[1], pos4[2];  
  transform_4_5.rotate (Eigen::AngleAxisf (angle2deg(r4_5_aboutx), Eigen::Vector3f::UnitX())); 
  Eigen::Affine3f transform_4_5_z  = Eigen::Affine3f::Identity();
  transform_4_5_z.rotate (Eigen::AngleAxisf (angle2deg(r4_5_aboutz), Eigen::Vector3f::UnitZ())); 
  transform_4_5.matrix().block(0,0,3,3) = transform_4_5_z.rotation() * transform_4_5.rotation();
  pcl::transformPointCloud (cylinder4_5, cylinder4_5_transformed_cloud, transform_4_5);
  pcl::transformPointCloud (sphere0, sphere4, transform_4_5);
  Eigen::Vector3f pos5;
  pos5[0] = 0.0; pos5[1] = l4_5; pos5[2] = 0.0;
  pos5 = transform_4_5 * pos5;
  
  Eigen::Affine3f transform_5_6  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder5_6_transformed_cloud;
  pcl::PointCloud<PointType> sphere5;
  transform_5_6.translation() << pos5[0], pos5[1], pos5[2];  
  transform_5_6.rotate (Eigen::AngleAxisf (angle2deg(r5_6+r4_5_aboutx), Eigen::Vector3f::UnitX())); 
  transform_5_6.matrix().block(0,0,3,3) = transform_4_5_z.rotation() * transform_5_6.rotation(); 
  pcl::transformPointCloud (cylinder5_6, cylinder5_6_transformed_cloud, transform_5_6);
  pcl::transformPointCloud (sphere0, sphere5, transform_5_6);
  Eigen::Vector3f pos6;
  pos6[0] = 0.0; pos6[1] = l5_6; pos6[2] = 0.0;
  pos6 = transform_5_6 * pos6;
  
  Eigen::Affine3f transform_6  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder6_transformed_cloud;
  pcl::PointCloud<PointType> sphere6;
  transform_6.translation() << pos6[0], pos6[1], pos6[2];  
  transform_6.rotate (Eigen::AngleAxisf (angle2deg(r6+r5_6+r4_5_aboutx), Eigen::Vector3f::UnitX()));  
  transform_6.matrix().block(0,0,3,3) = transform_4_5_z.rotation() * transform_6.rotation();
  pcl::transformPointCloud (cylinder6, cylinder6_transformed_cloud, transform_6);
  pcl::transformPointCloud (sphere0, sphere6, transform_6);
  
  
  
  Eigen::Affine3f transform_0_7 = Eigen::Affine3f::Identity();     
  transform_0_7.translation() << 0.0, 0.0, 0.0;  
  transform_0_7.rotate (Eigen::AngleAxisf (angle2deg(r0_7), Eigen::Vector3f::UnitZ()));   
  pcl::PointCloud<PointType> cylinder0_7_transformed_cloud;
  pcl::transformPointCloud (cylinder0_7, cylinder0_7_transformed_cloud, transform_0_7);
  Eigen::Vector3f pos7;
  pos7[0] = 0.0; pos7[1] = l0_7; pos7[2] = 0.0;
  pos7 = transform_0_7 * pos7;
  
  Eigen::Affine3f transform_7_8  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder7_8_transformed_cloud;
  pcl::PointCloud<PointType> sphere7;
  transform_7_8.translation() << pos7[0], pos7[1], pos7[2];  
  transform_7_8.rotate (Eigen::AngleAxisf (angle2deg(r7_8_aboutx), Eigen::Vector3f::UnitX())); 
  Eigen::Affine3f transform_7_8_z  = Eigen::Affine3f::Identity();
  transform_7_8_z.rotate (Eigen::AngleAxisf (angle2deg(r7_8_aboutz), Eigen::Vector3f::UnitZ())); 
  transform_7_8.matrix().block(0,0,3,3) = transform_7_8_z.rotation() * transform_7_8.rotation();
  pcl::transformPointCloud (cylinder7_8, cylinder7_8_transformed_cloud, transform_7_8);
  pcl::transformPointCloud (sphere0, sphere7, transform_7_8);
  Eigen::Vector3f pos8;
  pos8[0] = 0.0; pos8[1] = l7_8; pos8[2] = 0.0;
  pos8 = transform_7_8 * pos8;
  
  Eigen::Affine3f transform_8_9  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder8_9_transformed_cloud;
  pcl::PointCloud<PointType> sphere8;
  transform_8_9.translation() << pos8[0], pos8[1], pos8[2];  
  transform_8_9.rotate (Eigen::AngleAxisf (angle2deg(r8_9+r7_8_aboutx), Eigen::Vector3f::UnitX())); 
  transform_8_9.matrix().block(0,0,3,3) = transform_7_8_z.rotation() * transform_8_9.rotation();
  pcl::transformPointCloud (cylinder8_9, cylinder8_9_transformed_cloud, transform_8_9);
  pcl::transformPointCloud (sphere0, sphere8, transform_8_9);
  Eigen::Vector3f pos9;
  pos9[0] = 0.0; pos9[1] = l8_9; pos9[2] = 0.0;
  pos9 = transform_8_9 * pos9;
  
  Eigen::Affine3f transform_9  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder9_transformed_cloud;
  pcl::PointCloud<PointType> sphere9;
  transform_9.translation() << pos9[0], pos9[1], pos9[2];  
  transform_9.rotate (Eigen::AngleAxisf (angle2deg(r9+r8_9+r7_8_aboutx), Eigen::Vector3f::UnitX()));  
  transform_9.matrix().block(0,0,3,3) = transform_7_8_z.rotation() * transform_9.rotation();
  pcl::transformPointCloud (cylinder9, cylinder9_transformed_cloud, transform_9);
  pcl::transformPointCloud (sphere0, sphere9, transform_9);
  
  
  
  Eigen::Affine3f transform_0_10 = Eigen::Affine3f::Identity();     
  transform_0_10.translation() << 0.0, 0.0, 0.0;  
  transform_0_10.rotate (Eigen::AngleAxisf (angle2deg(r0_10), Eigen::Vector3f::UnitZ()));   
  pcl::PointCloud<PointType> cylinder0_10_transformed_cloud;
  pcl::transformPointCloud (cylinder0_10, cylinder0_10_transformed_cloud, transform_0_10);
  Eigen::Vector3f pos10;
  pos10[0] = 0.0; pos10[1] = l0_10; pos10[2] = 0.0;
  pos10 = transform_0_10 * pos10;
  
  Eigen::Affine3f transform_10_11  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder10_11_transformed_cloud;
  pcl::PointCloud<PointType> sphere10;
  transform_10_11.translation() << pos10[0], pos10[1], pos10[2];  
  transform_10_11.rotate (Eigen::AngleAxisf (angle2deg(r10_11_aboutx), Eigen::Vector3f::UnitX())); 
  Eigen::Affine3f transform_10_11_z  = Eigen::Affine3f::Identity();
  transform_10_11_z.rotate (Eigen::AngleAxisf (angle2deg(r10_11_aboutz), Eigen::Vector3f::UnitZ())); 
  transform_10_11.matrix().block(0,0,3,3) = transform_10_11_z.rotation() * transform_10_11.rotation();  
  pcl::transformPointCloud (cylinder10_11, cylinder10_11_transformed_cloud, transform_10_11);
  pcl::transformPointCloud (sphere0, sphere10, transform_10_11);
  Eigen::Vector3f pos11;
  pos11[0] = 0.0; pos11[1] = l10_11; pos11[2] = 0.0;
  pos11 = transform_10_11 * pos11;
  
  Eigen::Affine3f transform_11_12  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder11_12_transformed_cloud;
  pcl::PointCloud<PointType> sphere11;
  transform_11_12.translation() << pos11[0], pos11[1], pos11[2];  
  transform_11_12.rotate (Eigen::AngleAxisf (angle2deg(r11_12+r10_11_aboutx), Eigen::Vector3f::UnitX()));  
  transform_11_12.matrix().block(0,0,3,3) = transform_10_11_z.rotation() * transform_11_12.rotation();
  pcl::transformPointCloud (cylinder11_12, cylinder11_12_transformed_cloud, transform_11_12);
  pcl::transformPointCloud (sphere0, sphere11, transform_11_12);
  Eigen::Vector3f pos12;
  pos12[0] = 0.0; pos12[1] = l11_12; pos12[2] = 0.0;
  pos12 = transform_11_12 * pos12;
  
  Eigen::Affine3f transform_12 = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder12_transformed_cloud;
  pcl::PointCloud<PointType> sphere12;
  transform_12.translation() << pos12[0], pos12[1], pos12[2];  
  transform_12.rotate (Eigen::AngleAxisf (angle2deg(r12+r11_12+r10_11_aboutx), Eigen::Vector3f::UnitX()));  
  transform_12.matrix().block(0,0,3,3) = transform_10_11_z.rotation() * transform_12.rotation();
  pcl::transformPointCloud (cylinder12, cylinder12_transformed_cloud, transform_12);
  pcl::transformPointCloud (sphere0, sphere12, transform_12);
  
  
  
  
  Eigen::Affine3f transform_0_13 = Eigen::Affine3f::Identity();     
  transform_0_13.translation() << 0.0, 0.0, 0.0;  
  transform_0_13.rotate (Eigen::AngleAxisf (angle2deg(r0_13), Eigen::Vector3f::UnitZ()));   
  pcl::PointCloud<PointType> cylinder0_13_transformed_cloud;
  pcl::transformPointCloud (cylinder0_13, cylinder0_13_transformed_cloud, transform_0_13);
  Eigen::Vector3f pos13;
  pos13[0] = 0.0; pos13[1] = l0_13; pos13[2] = 0.0;
  pos13 = transform_0_13 * pos13;
  
  Eigen::Affine3f transform_13_14  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder13_14_transformed_cloud;
  pcl::PointCloud<PointType> sphere13;
  transform_13_14.translation() << pos13[0], pos13[1], pos13[2];  
  transform_13_14.rotate (Eigen::AngleAxisf (angle2deg(r13_14_aboutx), Eigen::Vector3f::UnitX())); 
  Eigen::Affine3f transform_13_14_z  = Eigen::Affine3f::Identity();
  transform_13_14_z.rotate (Eigen::AngleAxisf (angle2deg(r13_14_aboutz), Eigen::Vector3f::UnitZ())); 
  transform_13_14.matrix().block(0,0,3,3) = transform_13_14_z.rotation() * transform_13_14.rotation();
  pcl::transformPointCloud (cylinder13_14, cylinder13_14_transformed_cloud, transform_13_14);
  pcl::transformPointCloud (sphere0, sphere13, transform_13_14);
  Eigen::Vector3f pos14;
  pos14[0] = 0.0; pos14[1] = l13_14; pos14[2] = 0.0;
  pos14 = transform_13_14 * pos14;
  
  Eigen::Affine3f transform_14_15  = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder14_15_transformed_cloud;
  pcl::PointCloud<PointType> sphere14;
  transform_14_15.translation() << pos14[0], pos14[1], pos14[2];  
  transform_14_15.rotate (Eigen::AngleAxisf (angle2deg(r14_15+r13_14_aboutx), Eigen::Vector3f::UnitX()));  
  transform_14_15.matrix().block(0,0,3,3) = transform_13_14_z.rotation() * transform_14_15.rotation();
  pcl::transformPointCloud (cylinder14_15, cylinder14_15_transformed_cloud, transform_14_15);
  pcl::transformPointCloud (sphere0, sphere14, transform_14_15);
  Eigen::Vector3f pos15;
  pos15[0] = 0.0; pos15[1] = l14_15; pos15[2] = 0.0;
  pos15 = transform_14_15 * pos15;
  
  Eigen::Affine3f transform_15 = Eigen::Affine3f::Identity();
  pcl::PointCloud<PointType> cylinder15_transformed_cloud;
  pcl::PointCloud<PointType> sphere15;
  transform_15.translation() << pos15[0], pos15[1], pos15[2];  
  transform_15.rotate (Eigen::AngleAxisf (angle2deg(r15+r14_15+r13_14_aboutx), Eigen::Vector3f::UnitX()));
  transform_15.matrix().block(0,0,3,3) = transform_13_14_z.rotation() * transform_15.rotation();
  pcl::transformPointCloud (cylinder15, cylinder15_transformed_cloud, transform_15);
  pcl::transformPointCloud (sphere0, sphere15, transform_15);
  
  
  
  pcl::PointCloud<PointType> hand_ori;

  hand_ori = sphere0;
  
  
  hand_ori += estimateTatchingPointsAndNormals(cylinder0_1_transformed_cloud);
  hand_ori += estimateTatchingPointsAndNormals(cylinder1_2_transformed_cloud);
  hand_ori += sphere1;  
  hand_ori += estimateTatchingPointsAndNormals(cylinder2_transformed_cloud);
  hand_ori += sphere2;
  
  
  hand_ori += estimateTatchingPointsAndNormals(cylinder0_4_transformed_cloud);
  hand_ori += estimateTatchingPointsAndNormals(cylinder4_5_transformed_cloud);
  hand_ori += sphere4;
  hand_ori += estimateTatchingPointsAndNormals(cylinder5_6_transformed_cloud);
  hand_ori += sphere5;
  hand_ori += estimateTatchingPointsAndNormals(cylinder6_transformed_cloud);
  hand_ori += sphere6;
  
  hand_ori += estimateTatchingPointsAndNormals(cylinder0_7_transformed_cloud);
  hand_ori += estimateTatchingPointsAndNormals(cylinder7_8_transformed_cloud);
  hand_ori += sphere7;
  hand_ori += estimateTatchingPointsAndNormals(cylinder8_9_transformed_cloud);
  hand_ori += sphere8;
  hand_ori += estimateTatchingPointsAndNormals(cylinder9_transformed_cloud);
  hand_ori += sphere9;
  
  
  hand_ori += estimateTatchingPointsAndNormals(cylinder0_10_transformed_cloud);
  hand_ori += estimateTatchingPointsAndNormals(cylinder10_11_transformed_cloud);
  hand_ori += sphere10;
  hand_ori += estimateTatchingPointsAndNormals(cylinder11_12_transformed_cloud);
  hand_ori += sphere11;
  hand_ori += estimateTatchingPointsAndNormals(cylinder12_transformed_cloud);
  hand_ori += sphere12;
  
  
  hand_ori += estimateTatchingPointsAndNormals(cylinder0_13_transformed_cloud);
  hand_ori += estimateTatchingPointsAndNormals(cylinder13_14_transformed_cloud);
  hand_ori += sphere13;
  hand_ori += estimateTatchingPointsAndNormals(cylinder14_15_transformed_cloud);
  hand_ori += sphere14;
  hand_ori += estimateTatchingPointsAndNormals(cylinder15_transformed_cloud);
  hand_ori += sphere15;
  
  return hand_ori;
}

pcl::PointCloud<PointType> estimateTatchingPointsAndNormals(pcl::PointCloud<PointType>  pointcloud)
{
  pcl::PointCloud<PointType> tatchingpoints;
  PointType tmppoint;
  tmppoint.x = 0;
  tmppoint.y = 0;
  tmppoint.z = 0;
  int count = 0;
  
  Eigen::Vector3f backpoint;
  for(int i = 0; i < pointcloud.size(); ++ i)
  {
    if(pointcloud.at(i).r == 255 && pointcloud.at(i).g == 0 && pointcloud.at(i).b == 0)
    {      
      tmppoint.x += pointcloud.at(i).x;
      tmppoint.y += pointcloud.at(i).y;
      tmppoint.z += pointcloud.at(i).z;
      count ++;
      PointType tmppoints;
      tmppoints = pointcloud.at(i);
      tatchingpoints.push_back(tmppoints);
    }
    if(pointcloud.at(i).r == 0 && pointcloud.at(i).g == 0 && pointcloud.at(i).b == 0)
    {      
      backpoint = pointcloud.at(i).getArray3fMap();
    }
  }
  tmppoint.x /= count;
  tmppoint.y /= count;
  tmppoint.z /= count;
  tmppoint.r = 255;
  tmppoint.g = 255;
  tmppoint.b = 0;
  Eigen::Vector3f frontpoint = tmppoint.getArray3fMap();
  
  pcl::PCA<PointType> pca;
  pca.setInputCloud(tatchingpoints.makeShared());
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  Eigen::Vector3f V1 = eigenvectors.col(0);
  Eigen::Vector3f V2 = eigenvectors.col(1);
  Eigen::Vector3f V3 = eigenvectors.col(2);
  
  Geomentry geo;
  if(geo.VectorAngle(backpoint-frontpoint, V3) < M_PI/2)
    V3 = -V3;
  
  tmppoint.normal_x = V3[0];
  tmppoint.normal_y = V3[1];
  tmppoint.normal_z = V3[2];
  pointcloud.push_back(tmppoint);
  return pointcloud;
}


Eigen::Affine3f rotationzy(float theta)
{
  Eigen::Vector3f zaxis;
  zaxis[0] = 0.0; zaxis[1] = 0.0; zaxis[2] = 1.0;
  Eigen::Vector3f yaxis;
  yaxis[0] = 0.0; yaxis[1] = 1.0; yaxis[2] = 0.0;
  Eigen::Vector3f xaxis;
  xaxis[0] = 1.0; xaxis[1] = 0.0; xaxis[2] = 0.0;
  Geomentry geo;
  Eigen::Matrix<float, 3, 3> rotationabouty = geo.RotationAboutVector(yaxis, M_PI/8);
  Eigen::Matrix<float, 3, 3> rotationaboutx = geo.RotationAboutVector(xaxis, M_PI/8);
  Eigen::Vector3f zyaxis = rotationaboutx * rotationabouty * zaxis;
  Eigen::Matrix<float, 3, 3> rotationaboutzy = geo.RotationAboutVector(zyaxis, theta);
  Eigen::Matrix4f M;
  M.block(0,0,3,3) = rotationaboutzy;
  Eigen::Affine3f a  = Eigen::Affine3f::Identity();
  a = M;
  return a;   
}

Eigen::Affine3f rotation01(float theta)
{
  Eigen::Vector3f zaxis;
  zaxis[0] = 0.0; zaxis[1] = 0.0; zaxis[2] = 1.0;
  Eigen::Vector3f yaxis;
  yaxis[0] = 0.0; yaxis[1] = 1.0; yaxis[2] = 0.0;
  Eigen::Vector3f xaxis;
  xaxis[0] = 1.0; xaxis[1] = 0.0; xaxis[2] = 0.0;
  Geomentry geo;
  Eigen::Matrix<float, 3, 3> rotationabouty = geo.RotationAboutVector(yaxis, M_PI/4);
  Eigen::Matrix<float, 3, 3> rotationaboutx = geo.RotationAboutVector(xaxis, M_PI/4);
  Eigen::Vector3f zyaxis = rotationaboutx * rotationabouty * zaxis;
  Eigen::Matrix<float, 3, 3> rotationaboutzy = geo.RotationAboutVector(zyaxis, theta);
  Eigen::Matrix4f M;
  M.block(0,0,3,3) = rotationaboutzy;
  Eigen::Affine3f a  = Eigen::Affine3f::Identity();
  a = M;
  return a;   
}

float angle2deg(float angle)
{
  return angle*M_PI/180;
}

bool saveDescriptor(pcl::PointCloud<PointType> handmoudle, string filename)
{
  //there are 19 touching points in current hand moudle
  vector < pair <Eigen::Vector3f, Eigen::Vector3f> > handtouchpoints;  //pair <point, normal>
  
  for(int i = 0; i < handmoudle.size(); ++ i)
  {
    if(handmoudle.at(i).r == 255 && handmoudle.at(i).g == 255&& handmoudle.at(i).b == 0)
    {
      Eigen::Vector3f point = handmoudle.at(i).getArray3fMap();
      Eigen::Vector3f normal = handmoudle.at(i).getNormalVector3fMap();
      pair <Eigen::Vector3f, Eigen::Vector3f> tmppair;
      tmppair.first = point;
      tmppair.second = normal;
      handtouchpoints.push_back(tmppair);
    }
  }
  
  Geomentry geo;
  ofstream allfeatures (filename.c_str());
  for(int j = 0; j < handtouchpoints.size(); ++ j)
  {
    for(int p = 0; p < handtouchpoints.size(); ++ p)
    {
      if(j != p)
      {
	Eigen::Vector3f p1p2 = handtouchpoints.at(j).first - handtouchpoints.at(p).first;
	float d = p1p2.norm();
	if(d < 50)
	  continue;
	float theta1 = geo.VectorAngle(handtouchpoints.at(j).second, p1p2);
	if(theta1 > M_PI/2)
	  theta1 -= M_PI/2;
	float theta2 = geo.VectorAngle(handtouchpoints.at(p).second, p1p2);
	if(theta2 > M_PI/2)
	  theta2 -= M_PI/2;
	allfeatures << d << " " << theta1 << " " << theta2 << endl;
      }
    }
  }
  allfeatures.close();
  return true;
}

bool loadDescriptor(vector <vector <float> > & Descriptor, string filename)
{
  ifstream featureFile(filename.c_str());
  if (featureFile.is_open())
  {    
    string line;
    while ( featureFile.good() )
    {	
      getline (featureFile,line);     
      string buf; // Have a buffer string
      stringstream ss(line); // Insert the string into a stream
      
      int position = 0;
      vector<float> onefeature(3);
      while (ss >> buf)
      {
	onefeature[position] = atof(buf.c_str());
	position ++;
      }      
      if(line != "")
	Descriptor.push_back(onefeature);
    }
    featureFile.close();
    return true;
  }
  else
  {
    cerr << "Unable to open file: " << filename << endl;
    return false;
  }
}


vector <float> NN(vector<float> a, vector<vector <float> > allfeatures)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Generate pointcloud data
 
  cloud.resize (allfeatures.size());

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = allfeatures.at(i)[0];
    cloud.points[i].y = allfeatures.at(i)[1];
    cloud.points[i].z = allfeatures.at(i)[2];
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud.makeShared());

  pcl::PointXYZ searchPoint;

  searchPoint.x = a[0];
  searchPoint.y = a[1];
  searchPoint.z = a[2];
  
  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  vector <float> nnfeature(3);
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    nnfeature[0] = cloud.points[ pointIdxNKNSearch[0] ].x;
    nnfeature[1] = cloud.points[ pointIdxNKNSearch[0] ].y;
    nnfeature[2] = cloud.points[ pointIdxNKNSearch[0] ].z;
		
  }
  return nnfeature;
}

float Dist(vector<float> a, vector<float> b)
{
  float d = a[0] - b[0];
  float theta1 = a[1] - b[1];
  float theta2 = a[2] - b[2];
  return sqrt(d*d + theta1*theta1 + theta2*theta2);
}