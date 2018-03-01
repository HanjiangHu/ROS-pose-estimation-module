#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud,pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
        //法向量
        pointnormal::Ptr point_normal (new pointnormal);
        pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> est_normal;
        est_normal.setInputCloud(input_cloud);
        est_normal.setSearchMethod(tree);
        est_normal.setKSearch(10);
        est_normal.compute(*point_normal);
        //fpfh 估计
        fpfhFeature::Ptr fpfh (new fpfhFeature);
        //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
        pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
        est_fpfh.setNumberOfThreads(4); //指定4核计算
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
        est_fpfh.setInputCloud(input_cloud);
        est_fpfh.setInputNormals(point_normal);
        est_fpfh.setSearchMethod(tree);
        est_fpfh.setKSearch(10);
        est_fpfh.compute(*fpfh);

        return fpfh;
        
}


int main (int argc, char **argv)
{
        if (argc < 3)
        {
                cout<<"please input two pointcloud"<<endl;
                return -1;
        }
        clock_t start,end,time;
        start  = clock();
        pointcloud::Ptr source (new pointcloud);
        pointcloud::Ptr target (new pointcloud);
        pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *source);
        pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *target);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

        fpfhFeature::Ptr source_fpfh =  compute_fpfh_feature(source,tree);
        fpfhFeature::Ptr target_fpfh =  compute_fpfh_feature(target,tree);
        
         //对齐(占用了大部分运行时间)
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        sac_ia.setInputSource(source);
        sac_ia.setSourceFeatures(source_fpfh);
        sac_ia.setInputTarget(target);
        sac_ia.setTargetFeatures(target_fpfh);
        pointcloud::Ptr align (new pointcloud);
        pointcloud::Ptr final0 (new pointcloud);
        //  sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
        sac_ia.setCorrespondenceRandomness(6); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
        sac_ia.align(*align); 
        end = clock();
        cout <<"calculate time is: "<< float (end-start)/CLOCKS_PER_SEC<<endl;
        
        pcl::io::savePCDFile ("crou_output.pcd", *align);
         /*//可视化
        boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("fpfh test"));
        int v1;
        int v2;
        
        view->createViewPort(0,0.0,0.5,1.0,v1);
        view->createViewPort(0.5,0.0,1.0,1.0,v2);
        view->setBackgroundColor(0,0,0,v1);
        view->setBackgroundColor(0.05,0,0,v2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source,250,0,0);
        view->addPointCloud(source,sources_cloud_color,"sources_cloud_v1",v1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color (target,0,250,0);
        view->addPointCloud(target,target_cloud_color,"target_cloud_v1",v1);
        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sources_cloud_v1");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(final0,255,0,0);
        view->addPointCloud(align,aligend_cloud_color,"aligend_cloud_v2",v2);
        view->addPointCloud(target,target_cloud_color,"target_cloud_v2",v2);
        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4,"aligend_cloud_v2");
        view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v2");

        //   view->addCorrespondences<pcl::PointXYZ>(source,target,*cru_correspondences,"correspondence",v1);//添加显示对应点对
        while (!view->wasStopped())
        {
                // view->spin();
                view->spinOnce(100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                  

        }
         pcl::io::savePCDFile ("crou_output.pcd", *align);
         //  pcl::io::savePCDFile ("final_align.pcd", *final);
        */
        return 0;
}
