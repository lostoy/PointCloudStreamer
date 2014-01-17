#ifndef GPU_UTILITY
#define GPU_UTILITY
#ifdef CUDA_DLL_EXPORTS
#define CUDA_DLL_API __declspec(dllexport)
#else
#define CUDA_DLL_API __declspec(dllimport)
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


CUDA_DLL_API void transformCloudGPU(const pcl::PointCloud<pcl::PointXYZRGBA> &src ,pcl::PointCloud<pcl::PointXYZRGBA> &tgt,Eigen::Matrix4f transform);

#endif