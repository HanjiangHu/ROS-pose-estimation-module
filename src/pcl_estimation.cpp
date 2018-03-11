#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include<pcl/filters/conditional_removal.h>
#include<pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/registration/icp_nl.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Core>
#include<iostream>
#include<ctime>
    #include<cmath>
    
/**
 * 计算fpfh特征
 * */
pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
        //法向量
        pcl::PointCloud<pcl::Normal>::Ptr point_normal (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> est_normal;
        est_normal.setInputCloud(input_cloud);
        est_normal.setSearchMethod(tree);
        est_normal.setKSearch(5);
        //est_normal.setRadiusSearch(0.005);
        est_normal.compute(*point_normal);
        //fpfh 估计
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);
        //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
        pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
        est_fpfh.setNumberOfThreads(4); //指定4核计算
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
        est_fpfh.setInputCloud(input_cloud);
        est_fpfh.setInputNormals(point_normal);
        est_fpfh.setSearchMethod(tree);
        est_fpfh.setKSearch(5);
        //est_fpfh.setRadiusSearch(0.01);
        est_fpfh.compute(*fpfh);
        return fpfh;
        
}
ros::Publisher pcl_pub;

void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dataset,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp){
    //  去噪
    /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    statFilter.setInputCloud(cloud_dataset -> makeShared());
    statFilter.setMeanK(10);
    statFilter.setStddevMulThresh(4);
    statFilter.filter(*cloud_dataset);*/

    //statFilter.setInputCloud(cloud_tmp -> makeShared());
    //statFilter.setMeanK(10);
    //statFilter.setStddevMulThresh(0.2);
    //statFilter.filter(*cloud_tmp);

    //对被旋转的进行降低点数
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud_tmp -> makeShared());
    voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelSampler.filter(*cloud_tmp);

    //voxelSampler.setInputCloud(cloud_dataset -> makeShared());
    //voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    //voxelSampler.filter(*cloud_dataset);
        
        
    //cloud_added = *cloud_dataset;//用于生成数据集点云
}

Eigen::Matrix4f fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dataset,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh =  compute_fpfh_feature(cloud_tmp,tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh =  compute_fpfh_feature(cloud_dataset,tree);
        
    //粗对齐
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(cloud_tmp);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(cloud_dataset);
    sac_ia.setTargetFeatures(target_fpfh);
    
    //sac_ia.setNumberOfSamples(6);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
    //sac_ia.setCorrespondenceRandomness(6); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
    sac_ia.align(*cloud_tmp); 
    std::cout << "粗对齐变换矩阵：" << std::endl << sac_ia.getFinalTransformation() << std::endl;
    //pcl::io::savePCDFileASCII ("fpfh.pcd", *cloud_tmp);
    return sac_ia.getFinalTransformation();
}

Eigen::Matrix4f icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dataset,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp){
    // 配准
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;   // 配准对象
    reg.setTransformationEpsilon (1e-17);   ///设置收敛判断条件，越小精度越大，收敛也越慢 
    reg.setMaxCorrespondenceDistance (0.5);  //大于此值的点对不考虑为对应点
    
    reg.setInputSource (cloud_tmp -> makeShared());   // 设置源点云
    reg.setInputTarget (cloud_dataset -> makeShared());    // 设置目标点云

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result = cloud_tmp;
    reg.setMaximumIterations (2000);//设置单次最大的迭代次数，停止内部迭代
    for (int i = 0; i < 2000; ++i)   //手动迭代，改变对应点的最大值
        {
            cloud_tmp = reg_result;
            reg.setInputSource (cloud_tmp -> makeShared());
            reg.align (*reg_result);

            Ti = reg.getFinalTransformation () * Ti;//Ti是积累的变换矩阵

            //如果变换矩阵改变量大于临界值，减小最大对应点距离重新匹配
            if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.0001);
            prev = reg.getLastIncrementalTransformation ();//上次的变换矩阵
        }
    //targetToSource = Ti.inverse();
    cloud_tmp = reg_result;
    std::cout << "icp变换矩阵：" << std::endl <<Ti << std::endl;
    return Ti;
}

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(input, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dataset(new pcl::PointCloud<pcl::PointXYZ>);//基准
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);//被旋转的
    //pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/huhanjiang/cup01234_filtered.pcd", *cloud_dataset);
    *cloud_tmp = cloud;

    clock_t start,finish;

    /**
     * 滤波
     * */
    start = clock();
    filter(cloud_dataset,cloud_tmp);
    finish = clock();
    std::cout <<"filter: "<< float (finish-start)/CLOCKS_PER_SEC<<std::endl;
		
	/**
     * fpfh粗对齐
     * */
    start = clock();
    Eigen::Matrix4f T1 = fpfh(cloud_dataset,cloud_tmp);
    finish = clock();
    std::cout <<"fpfh: "<< float (finish-start)/CLOCKS_PER_SEC<<std::endl;


    /**
     * 非线性icp
     * */
    start = clock();
    Eigen::Matrix4f T2 = icp(cloud_dataset,cloud_tmp);
    finish = clock();
    std::cout <<"icp: "<< float (finish-start)/CLOCKS_PER_SEC<<std::endl;   

    std::cout << "总变换矩阵：" << std::endl << T2 * T1 << std::endl;
    
        
    //对数据库里的点云进行滤波
    /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    statFilter.setInputCloud(cloud_added.makeShared());
    statFilter.setMeanK(10);
    statFilter.setStddevMulThresh(2);
    statFilter.filter(cloud_added);
    std::cout << cloud_added.points.size()<< std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud_added.makeShared());
    voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelSampler.filter(cloud_added);
    std::cout << cloud_added.points.size()<< std::endl;*/

    /*voxelSampler.setInputCloud(cloud_dataset -> makeShared());
    //voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelSampler.filter(*cloud_dataset);*/
    //pcl::toROSMsg(cloud_added, output);//用于生成数据集点云
    sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_tmp, output);
    //pcl_pub.publish(output);
    output.header.frame_id = "map";

    pcl_pub.publish(output);
}

main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_estimation");

    ros::NodeHandle nh;
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_matched", 1);//暂时发布icp后的点云，以便和目标点云进行对比
    ros::Subscriber bat_sub = nh.subscribe("pcl_target", 1, cloudCB);//接收实时点云
      
    ros::spin();
    return 0;
}



