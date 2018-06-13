# pcl
pcl
一，功能描述
  1，视差图点云创建，
  2，鼠标选取点云创建区域与待匹配物体，
  3，点云滤波，
  4，点云超体聚类，
  5，LCCP分割，
  6，点云降采样
  7，ICP匹配
  8，点云显示。
  
二，功能解释
  1，视差图点云创建
     点云创建部分 使用四元数旋转参数 q(0.5,0,0,0) 与位移参数（0，0，0）使创建的点云为以相机坐标系为原点。
     采用原图进行 1/4 采样的方式加速点云创建。
  2，鼠标选取点云创建区域与待匹配物体
     待创建区域的选取是在RGB图像上进行的，通过鼠标左键获取选取区域的对角坐标，存储在Pt1与pt2中，鼠标右键点选待匹配物体，坐标点存储在pt3中。
  3，点云滤波
     滤波主要采用了统计滤波器：pcl::StatisticalOutlierRemoval
                与半径滤波器：pcl::RadiusOutlierRemoval
  4, 点云超体聚类
     pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
     
     https://www.cnblogs.com/ironstark/p/5013968.html
     点云超体聚类对点云进行过分割。
     在点云聚类中，没有使用颜色信息，所以将color_importance设置为0。
     粒子距离、晶核距离 参数越小过分割越严重，对于最后的分割效果越精细。
  5, LCCP分割
     pcl::LCCPSegmentation<PointT> lccp;
     
     https://www.cnblogs.com/ironstark/p/5027269.html
     LCCP分割利用物体的凹凸关系在超体聚类的基础上对聚类结果再聚类。
     
     通过超体聚类与LCCP分割之后会得到不同label标注的单个物体点云块。
  6，点云降采样
     在分割出来的单物体点云与点云库匹配之前，对点云库中的点云进行降采样，以减少计算量。
     pcl::VoxelGrid<pcl::PointXYZ>  程序中采用0.004的网格进行采样。
  7，ICP匹配
     ICP匹配部分主要是将分割出来的单物体点云与点云库中物体进行匹配，以获得物体点云到云点云库物体匹配时的旋转矩阵。
     pcl::IterativeClosestPoint<PointT,PointT> 
     
     最优匹配结果的选取是根据比较单个点云与点云库中物体匹配的得分进行选择，将匹配得分最好的一个输出保存。
  8，点云显示
     点云显示中采用pcl::visualization::PCLVisualizer类，支持给点云添加颜色、显示点云中的法矢、在窗口中自己画图案、自定义视角的位置 等功能。
     
     在本程序中将点云显示封装为函数，支持两个视窗，与单个视窗显示，并显示出点云坐标系。
     pcl::visualization::PCLVisualizer  pcd_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr Final_best)   为两个视窗显示函数。
     
     pcl::visualization::PCLVisualizer  pcd_viewer_one(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr Final_best)  为单视窗显示函数。
     
     在显示函数中均支持单个视窗添加显示多个点云文件，显示匹配效果。
     
 
三，文件目录
    --CMakeLists.txt
    --supervoxel_clustering.cpp
    --readme.txt
    --obj   //待匹配点云库
      --box1.pcd
      ------
      --box(n).pcd 
    --data   
    ----rgb //待分割RGB图
    ----disp //待分割视差图
    --build //程序编译目录
    
四，编译运行
    1，依赖项
      Opencv2.4.9 or later
        https://opencv.org/releases.html
      PointCloud library 1.8
        https://github.com/PointCloudLibrary/pcl
     
      eigen3.0
        sudo apt-get install libeigen3.0-dev
    2,程序编译
      在工程目录下
      mkdir build
      cd build
      cmake ..
      make 
     得到可执行文件 supervoxel_clustering
     
    3，程序运行
     ./supervoxel_clustering disp rgb 17
     运行示例 ，disp 为视差图子路径（data目录下） ，rgb 为RGB图子路径  ，17 图片序号
