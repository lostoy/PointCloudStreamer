#include "gpu_utility.h"
void transformCloudGPU_CU(float *,float*,float *,float *,int);
void transformCloudGPU(const pcl::PointCloud<pcl::PointXYZRGBA> &src ,pcl::PointCloud<pcl::PointXYZRGBA> &tgt, Eigen::Matrix4f transform)
{
	float *x=new float[src.size()];
	float *y=new float[src.size()];
	float *z=new float[src.size()];

	for (int i=0;i<src.size();i++)
	{
		x[i]=src.points[i].x;
		y[i]=src.points[i].y;
		z[i]=src.points[i].z;

	}

	float transform_[16];
	for (int i=0;i<16;i++)
		transform_[i]=transform.coeff(i);
	transformCloudGPU_CU(x,y,z,transform_,src.size());
	for (int i=0;i<src.size();i++)
	{
		tgt.points[i].x=x[i];
		tgt.points[i].y=y[i];
		tgt.points[i].z=z[i];
	}

	delete []x;
	delete []y;
	delete []z;


}
