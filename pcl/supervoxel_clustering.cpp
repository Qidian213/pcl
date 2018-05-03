#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <stdlib.h>  
#include <cmath>  
#include <boost/thread/thread.hpp>
#include <limits.h>  
#include <boost/format.hpp>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>  
#include <pcl/filters/passthrough.h>   
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>   
#include <pcl/segmentation/lccp_segmentation.h>  
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/radius_outlier_removal.h>  
#include <pcl/console/time.h>   // TicToc
using namespace cv;

// 定义点云使用的格式：这里用的是XYZ
typedef pcl::PointXYZ PointT; 
typedef pcl::PointCloud<PointT> PointCloud; 

//×××××××××××××××××××××××点云区域选择有关×××××××××××××××××××××//
Point pt1 = Point(0, 0);
Point pt2 = Point(0, 0);
Point pt3 = Point(0, 0);
bool is_selecting = false;
bool object_seted = false;
void cvMouseCallback(int mouseEvent, int x, int y, int flags, void* param) // 鼠标选取区域，左键区域，右键物体 。其中左键区域非必须。
{
  switch (mouseEvent)
  {
    case CV_EVENT_LBUTTONDOWN:
      pt1 = Point(x, y);
      pt2 = Point(x, y);
      cout<<"*******PO1********"<<endl;
      is_selecting = true;
      break;
    case CV_EVENT_MOUSEMOVE:
      if (is_selecting)
         pt2 = Point(x, y);
         //cout<<"*******MOVE********"<<endl;
      break;
    case CV_EVENT_LBUTTONUP:
      pt2 = Point(x, y);
      is_selecting = false;
      object_seted = true;
      cout<<"*******PO2********"<<endl;
      break;
    case CV_EVENT_RBUTTONDOWN:
      pt3 = Point(x, y);
      object_seted = false;  
  }
}
//×××××××××××××××××××××××点云区域选择有关×××××××××××××××××××××//

 pcl::visualization::PCLVisualizer pcd_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr Final_best);
 pcl::visualization::PCLVisualizer pcd_viewer_one(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr Final_best);
void showpcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
int main(int argc, char ** argv)  
{   
    pcl::console::TicToc time; //运行计时

  //×××××××××××××××××××××××点云区域选择×××××××××××××××××××××//
    Mat rgb_mat ,rct_mat ,depth ;

    std::stringstream str_disp;
      str_disp << "../data/"<<argv[1]<<"/disp_" <<argv[3]<< ".png"; 
    std::stringstream str_rgb;
      str_rgb << "../data/"<<argv[2]<<"/rgb_"<<argv[3]<<".png"; 
    
    rgb_mat=cv::imread(str_rgb.str());
    namedWindow("rgb_mat", 0);
    resizeWindow("rgb_mat",rgb_mat.cols/2,rgb_mat.rows/2);
    imshow("rgb_mat",rgb_mat);

    cvSetMouseCallback("rgb_mat", cvMouseCallback);
    //while(! pt3.x);
    pt3.x =500, pt3.y = 200; //因新电脑问题，waitKey(n)没有效果，只能预给定值测试。老电脑运行正常。
    cout<<"P1.X= "<<pt1.x<<"   P1.Y="<<pt1.y<<endl;
    cout<<"P2.X= "<<pt2.x<<"   P2.Y="<<pt2.y<<endl;
    cout<<"P3.X= "<<pt3.x<<"   P3.Y="<<pt3.y<<endl;

    if(pt1.x)
    {
      rct_mat=rgb_mat(Rect(pt1.x,pt1.y,abs(pt2.x-pt1.x),abs(pt2.y-pt1.y)));
      imshow("rct_mat",rct_mat);
     
      Mat disp = cv::imread(str_disp.str(), -1 );
      depth = disp(Rect(pt1.x,pt1.y,abs(pt2.x-pt1.x),abs(pt2.y-pt1.y))); // 使用-1读取原始图像       
    }
    else
    {
      depth = cv::imread(str_disp.str(), -1 );
    }
  //×××××××××××××××××××××××点云区域选择结束×××××××××××××××××××××//

  //×××××××××××××××××××××××点云创建部分×××××××××××××××××××××//
    time.tic();
    PointT object_p ; //选择物体点

    Eigen::Quaterniond q( 0.5, 0, 0, 0 );
    Eigen::Isometry3d Tr(q);
    Tr.pretranslate( Eigen::Vector3d( 0, 0, 0));
    // 相机内参 
    double cx = 193.279*2;
    double cy =  297.746*2;
    double fx = 599.898*2;
    double fy = 599.898*2;
    double depthScale = 1000.0;
 
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 

    for ( int v=0; v<depth.rows; v+=2 )
       for ( int u=0; u<depth.cols; u+=2 )
         {
            unsigned short d = depth.ptr<unsigned short> ( v )[u]; // 深度值
            if ( d==0 || d<8000 ) continue; // 为0表示没有测量到
            Eigen::Vector3d point; 
            double dep=(double)(71832*255)/d;
            point[2] = double(dep)/depthScale; 
            point[0] = (u-cx)*point[2]/fx;
            point[1] = (v-cy)*point[2]/fy; 
            Eigen::Vector3d pointWorld = Tr*point;
                
            PointT p ;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
            pointCloud->points.push_back( p );
            if(((pt3.x-pt1.x)==u) && ((pt3.y-pt1.y)==v)) //选择物体点判断
            { object_p=p; }
          }
    cout<<"object_p.x= "<<object_p.x<<"  object_p.y= "<<object_p.y<<"  object_p.z= "<<object_p.z<<endl;
    std::cout <<  "Create cloud = " << time.toc () << " ms" << std::endl;

    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
  //×××××××××××××××××××××××点云创建部分结束×××××××××××××××××××××//

  //×××××××××××××××××××××××点云滤波×××××××××××××××××××××//
    time.tic();
    PointCloud::Ptr pointCloud_filtered( new PointCloud );
      // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //统计滤波器
    sor.setInputCloud (pointCloud);
    sor.setMeanK (50);              //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh (1.0);   //设置判断是否为离群点的阀值
    sor.filter (*pointCloud_filtered);

//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem; //半径滤波器
//    outrem.setInputCloud(pointCloud);
//    outrem.setRadiusSearch(0.8);  
//    outrem.setMinNeighborsInRadius (2);
//    // apply filter
//    outrem.filter (*pointCloud_filtered);


//    pointCloud_filtered->is_dense = false;
//    cout<<"filtered 点云共有"<<pointCloud_filtered->size()<<"个点."<<endl;
//    std::stringstream str_fiter_save;
//    str_fiter_save <<"filtered_" <<argv[3]<< ".pcd";
//    pcl::io::savePCDFileBinary(str_fiter_save.str(), *pointCloud_filtered );
    std::cout <<  "filter=  " << time.toc () << " ms" << std::endl;
  //×××××××××××××××××××××××点云滤波结束×××××××××××××××××××××//
  
  //×××××××××××××××××××××××超体聚类×××××××××××××××××××××//
    time.tic();
    typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList; 
    //超体聚类 参数依次是粒子距离、晶核距离、颜色容差、  
    float voxel_resolution = 0.007f;   //与seed_resolution一般为2倍关系，参数越小过分割越严重，物体分的越细
    float seed_resolution = 0.014f;  
    float color_importance = 0.0f;  
    float spatial_importance = 1.0f;  
    float normal_importance = 1.0f;  
    bool use_single_cam_transform = false;  
    bool use_supervoxel_refinement = false;  
    unsigned int k_factor = 0;    
  
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);  
    super.setUseSingleCameraTransform(use_single_cam_transform);  
    super.setInputCloud(pointCloud_filtered);
    super.setColorImportance(color_importance);  
    super.setSpatialImportance(spatial_importance);    
    super.setNormalImportance(normal_importance);  
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;   
    super.extract(supervoxel_clusters);  

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;  
    super.getSupervoxelAdjacency(supervoxel_adjacency);  
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud =pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);  
    std::cout <<  "Supervoxel in = " << time.toc () << " ms" << std::endl;
  
//  sv_centroid_normal_cloud ->height = 1;
//  sv_centroid_normal_cloud ->width = sv_centroid_normal_cloud ->points.size();
//    cout<<"Super point cloud size = "<<sv_centroid_normal_cloud ->points.size()<<endl;
//    sv_centroid_normal_cloud ->is_dense = false;
//    pcl::io::savePCDFile( "Super.pcd", *sv_centroid_normal_cloud ); 
  //×××××××××××××××××××××××超体聚类结束×××××××××××××××××××××//
  
  //×××××××××××××××××××××××LCCP分割×××××××××××××××××××××//
    time.tic ();
    float concavity_tolerance_threshold = 9;  
    float smoothness_threshold = 0.1;  
    uint32_t min_segment_size = 0;  
    bool use_extended_convexity = true;  
    bool use_sanity_criterion = true;  
  // PCL_INFO("Starting Segmentation\n");  
    time.tic();
    pcl::LCCPSegmentation<PointT> lccp;  //生成LCCP分割器
  //CC效验beta值
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);  
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold); 
  //CC效验的k邻点 
    lccp.setKFactor(k_factor);  
  //输入超体聚类结果
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency); 
  //SC效验
    lccp.setSanityCheck (1); 
  //最小分割尺寸
    lccp.setMinSegmentSize(min_segment_size);  
    lccp.segment();  
 
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();  
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();  
    lccp.relabelCloud(*lccp_labeled_cloud);  
//   SuperVoxelAdjacencyList sv_adjacency_list;  
//   lccp.getSVAdjacencyList(sv_adjacency_list);

    std::cout <<  "LCCP in=  " << time.toc () << " ms" << std::endl;
    
    cout<<"LCCP point cloud size = "<<lccp_labeled_cloud ->points.size()<<endl;
    lccp_labeled_cloud ->is_dense = false;
    pcl::io::savePCDFile( "LCCP.pcd", *lccp_labeled_cloud ); 
  //×××××××××××××××××××××××LCCP分割结束×××××××××××××××××××××//
  
   //××××××××××××××××××××××得到点选物体×××××××××××××××××××××//
    time.tic ();
    int object_label=1000;
    int label_max2 = 0,pcdn=0;  
    for (int i = 0; i< lccp_labeled_cloud->size(); i++) 
    {  
      float det= 0;
      if (lccp_labeled_cloud->points[i].label>label_max2)  
	  label_max2 = lccp_labeled_cloud->points[i].label;  
      det =abs(object_p.x - lccp_labeled_cloud->points[i].x )+abs(object_p.y-lccp_labeled_cloud->points[i].y)+abs(object_p.z-lccp_labeled_cloud->points[i].z); //通过判断选取点误差确定选取物体的点云label.
      if(det<0.001)
      {
        object_label=lccp_labeled_cloud->points[i].label;
      }
    }
    cout<<"object_label= "<<object_label<<endl;
    
    PointCloud::Ptr pointCloud_seg( new PointCloud );
    PointT object_label_p ;
    int object_cot=0;
    for (int j = 0; j < lccp_labeled_cloud->size(); j++) 
    {  
      if (lccp_labeled_cloud->points[j].label == object_label) 
      {  
	    object_label_p.x = lccp_labeled_cloud->points[j].x;  
	    object_label_p.y = lccp_labeled_cloud->points[j].y;  
	    object_label_p.z = lccp_labeled_cloud->points[j].z;  
	    pointCloud_seg->points.push_back(object_label_p );
	    object_cot++;
	  }  
    }
    std::cout <<  "点选物体 in=  " << time.toc () << " ms" << std::endl;
    cout<<"点选 point cloud size = "<<pointCloud_seg ->points.size()<<endl;
    pointCloud_seg ->height = 1;
    pointCloud_seg ->width = pointCloud_seg ->points.size();
    pointCloud_seg->is_dense = false;
    pcl::io::savePCDFile( "pointCloud_seg.pcd", *pointCloud_seg ); 
   //××××××××××××××××××××××得到点选物体结束×××××××××××××××××××××//
   //××××××××××××××××××××××ICP匹配×××××××××××××××××××××//
    pcl::PCDReader reader;
    PointCloud::Ptr tar_cloud (new PointCloud);
    PointCloud::Ptr tar_filtered (new PointCloud);
    PointCloud::Ptr cloud_tr (new PointCloud);    
    PointCloud::Ptr Final_best(new PointCloud);  
    
    char obj_name[][20] = {"../obj/box1.pcd", "../obj/box2.pcd", "../obj/box3.pcd", "../obj/box4.pcd"};
    double score_best = 10000;
    int num_best = 0;
    
    for(int i=0; i<=3; i++)
    {
      time.tic ();
      pcl::VoxelGrid<pcl::PointXYZ> sor;  //目标点云降采样。
	  reader.read(obj_name[i], *tar_cloud);        // 目标点云
   	  sor.setInputCloud(tar_cloud);
   	  sor.setLeafSize(0.004f, 0.004f, 0.004f);
   	  sor.filter(*tar_filtered);
   	  
      pcl::IterativeClosestPoint<PointT,PointT> icp; //ICP点云匹配
    //icp.setMaximumIterations(iterations);
      icp.setInputTarget (tar_filtered);
      icp.setInputSource (pointCloud_seg);
      icp.setMaxCorrespondenceDistance(50);          // 设置对应点对之间的最大距离（此值对配准结果影响较大）
      icp.setTransformationEpsilon(1e-10);           // 设置两次变化矩阵之间的差值（一般设置为1e-10即可）
      icp.setEuclideanFitnessEpsilon(0.001);         // 设置收敛条件是均方误差和小于阈值，停止迭代；
      icp.setMaximumIterations(500);                  // 最大迭代次数
      
      PointCloud::Ptr Final(new PointCloud);
      icp.align(*Final);
      //icp.align(*pointCloud_seg);
     if (icp.hasConverged ())
     {
        if (icp.getFitnessScore() < score_best)
        {
          score_best = icp.getFitnessScore();
          num_best = i+1;
         // Final_best = Final;
          *Final_best = *Final;
        }
       std::cout << "----------------------------------------------------------"<< std::endl;
       std::cout << "has converged: " << icp.hasConverged() <<std::endl;
       std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
       std::cout << icp.getFinalTransformation() << std::endl;  // 输出 RT 矩阵     
	  }	
	  std::cout <<  "ICP in=  " << time.toc () << " ms" << std::endl;		
      
     }
     std::cout <<  "\nBest score: " << score_best << "\nObject: obj_" << num_best << ".pcd\n";
     //*************************显示有关×××××××××××××××××××//
     reader.read(obj_name[num_best-1], *tar_cloud); 
     pcl::visualization::PCLVisualizer viewer;
     viewer = pcd_viewer_one(tar_cloud,Final_best); //显示点云
     while (!viewer.wasStopped ())
     {
      // cout<<"***************************"<<endl;
       //viewer.spinOnce ();  //不加这一句显示出来的点云不能旋转
     }
     //*************************显示有关×××××××××××××××××××//
//     std::stringstream str_final;
//     str_final <<"final_" <<argv[3]<< ".pcd";
//     pcl::io::savePCDFileBinary(str_final.str(), *Final_best ); 
 
    //××××××××××××××××××××××ICP匹配结束×××××××××××××××××××××//
   
    return 0;  
} 


void showpcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
}
/////**********************************点云显示函数，两个视图的×××××××××××××××××××//
 pcl::visualization::PCLVisualizer  pcd_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr Final_best)
{
      pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two verticaly separated viewports
      int v1 (0);
      int v2 (1);
      viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
      viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
      float bckgr_gray_level = 0.0;  // Black
      float txt_gray_lvl = 1.0 - bckgr_gray_level;
      
    // Set background color
      viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
      viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
      viewer.addCoordinateSystem (1.0); //添加坐标系
    // Set camera position and orientation
      viewer.setCameraPosition(-0.5, 0.5,-2, 0, -1, 0, 0); //前三个为坐标轴原点位置XYZ，后三个为视角，最后一个默认0
   
      // Original point cloud is white
      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (tar_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,(int) 255 * txt_gray_lvl); //颜色配置
      viewer.addPointCloud (tar_cloud, cloud_in_color_h, "cloud_in_v1", v1);//左右视图显示库中点云
      viewer.addPointCloud (tar_cloud, cloud_in_color_h, "cloud_in_v2", v2);
      
        // ICP aligned point cloud is red
      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (Final_best, 180, 20, 20);
      viewer.addPointCloud (Final_best, cloud_icp_color_h, "cloud_icp_v2", v2); //右视图添加ICP旋转后的点云
      viewer.setSize (1280, 1024);  // Visualiser window size
      
      return (viewer);
}
/////**********************************点云显示函数，一个视图的×××××××××××××××××××//
 pcl::visualization::PCLVisualizer  pcd_viewer_one(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr Final_best)
{
      pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two verticaly separated viewports

    // The color we will be using
      float bckgr_gray_level = 0.0;  // Black
      float txt_gray_lvl = 1.0 - bckgr_gray_level;
      
    // Set background color
      viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
      
    // Set camera position and orientation
      viewer.setCameraPosition (-0.5, 0.5,-2, 0, -1, 0, 0); //前三个为坐标轴原点位置XYZ，后三个为视角，最后一个默认0
   
      // Original point cloud is white
      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (tar_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,(int) 255 * txt_gray_lvl); //颜色配置
      viewer.addPointCloud (tar_cloud, cloud_in_color_h, "cloud_in_v1");//视图显示库中点云
 
        // ICP aligned point cloud is red
      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (Final_best, 180, 20, 20);
      viewer.addPointCloud (Final_best, cloud_icp_color_h, "cloud_icp_v1"); //视图添加ICP旋转后的点云
      viewer.addCoordinateSystem (1.0); //添加坐标系
      viewer.setSize (1280, 1024);  // Visualiser window size
      
      return (viewer);
}



