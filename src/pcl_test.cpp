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


main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_test");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_matched", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud_in;
    //pcl::PointCloud<pcl::PointXYZ> cloud_out;

    pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
    //pcl::PointCloud<pcl::PointXYZ> cloud_added;//用于生成数据集点云
    sensor_msgs::PointCloud2 output;

    pcl::io::loadPCDFile<pcl::PointXYZ>("cup01234_filtered.pcd", *cloud_in);//基准
    pcl::io::loadPCDFile<pcl::PointXYZ>("target4.pcd", *cloud_out);//被旋转的
		
    clock_t start,finish;

    /**
     * 滤波
     * */
    start = clock();
    //  去噪
    /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    statFilter.setInputCloud(cloud_in -> makeShared());
    statFilter.setMeanK(10);
    statFilter.setStddevMulThresh(4);
    statFilter.filter(*cloud_in);*/

    //statFilter.setInputCloud(cloud_out -> makeShared());
    //statFilter.setMeanK(10);
    //statFilter.setStddevMulThresh(0.2);
    //statFilter.filter(*cloud_out);

    //对被旋转的进行降低点数
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud_out -> makeShared());
    voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelSampler.filter(*cloud_out);

    //voxelSampler.setInputCloud(cloud_in -> makeShared());
    //voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    //voxelSampler.filter(*cloud_in);
        
        
    //cloud_added = *cloud_in;//用于生成数据集点云
    finish = clock();
    std::cout <<"filter: "<< float (finish-start)/CLOCKS_PER_SEC<<std::endl;
		
	/**
     * fpfh粗对齐
     * */
    start = clock();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh =  compute_fpfh_feature(cloud_out,tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh =  compute_fpfh_feature(cloud_in,tree);
        
    //粗对齐
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(cloud_out);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(cloud_in);
    sac_ia.setTargetFeatures(target_fpfh);
    
    //sac_ia.setNumberOfSamples(6);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
    //sac_ia.setCorrespondenceRandomness(6); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
    sac_ia.align(*cloud_out); 
    std::cout << "粗对齐变换矩阵：" << std::endl << sac_ia.getFinalTransformation() << std::endl;
    //pcl::io::savePCDFileASCII ("fpfh.pcd", *cloud_out);
    finish = clock();
    std::cout <<"fpfh: "<< float (finish-start)/CLOCKS_PER_SEC<<std::endl;


    /**
     * 非线性icp
     * */
    start = clock();
    // 配准
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;   // 配准对象
    reg.setTransformationEpsilon (1e-17);   ///设置收敛判断条件，越小精度越大，收敛也越慢 
    reg.setMaxCorrespondenceDistance (0.5);  //大于此值的点对不考虑为对应点
    
    reg.setInputSource (cloud_out -> makeShared());   // 设置源点云
    reg.setInputTarget (cloud_in -> makeShared());    // 设置目标点云

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result = cloud_out;
    reg.setMaximumIterations (2000);//设置单次最大的迭代次数，停止内部迭代
    for (int i = 0; i < 2000; ++i)   //手动迭代，改变对应点的最大值
        {
            cloud_out = reg_result;
            reg.setInputSource (cloud_out -> makeShared());
            reg.align (*reg_result);

            Ti = reg.getFinalTransformation () * Ti;//Ti是积累的变换矩阵

            //如果变换矩阵改变量大于临界值，减小最大对应点距离重新匹配
            if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.0001);
            prev = reg.getLastIncrementalTransformation ();//上次的变换矩阵
        }
    //targetToSource = Ti.inverse();
    std::cout << "icp变换矩阵：" << std::endl <<Ti << std::endl;
    finish = clock();
    std::cout <<"icp: "<< float (finish-start)/CLOCKS_PER_SEC<<std::endl;   

    std::cout << "总变换矩阵：" << std::endl << Ti * sac_ia.getFinalTransformation() << std::endl;
    //cloud_added += *reg_result;//用于生成数据集点云
    //cloud_added += *cloud_out;

    /*一般icp*/
	/*pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_out -> makeShared());//变换的
    icp.setInputTarget(cloud_in -> makeShared());// 基准

    icp.setMaxCorrespondenceDistance(0.02);//5对应点对之间的最大距离
    icp.setMaximumIterations(2000000000);//100
    icp.setTransformationEpsilon (1e-17);//1e-12两次变化矩阵的插值
    icp.setEuclideanFitnessEpsilon(0.00001);//0.1均方差小于阈值，停止迭代

    icp.align(cloud_aligned);
    std::cout << icp.getFinalTransformation() << std::endl;*/
    //pcl::io::savePCDFileASCII ("target4_aligned2.pcd", cloud_aligned);
    //cloud_added += cloud_aligned;
	
    //std::cout << reg_result -> points.size()<< std::endl;
        
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

    /*voxelSampler.setInputCloud(cloud_in -> makeShared());
    //voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelSampler.filter(*cloud_in);*/
    //pcl::toROSMsg(cloud_added, output);//用于生成数据集点云
	pcl::toROSMsg(*reg_result, output);
    //pcl_pub.publish(output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


