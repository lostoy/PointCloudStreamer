

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>
#include <float.h>

__global__ void transformKernel(float *x ,float *y,float *z,float *transform)
{
	int i=blockIdx.x*blockDim.x+threadIdx.x;
	
	/*if (_finite(x[i])||
		_finite(y[i])||
		_finite(z[i]))
		return;*/
	 
	 
	 float x_,y_,z_;
	 x_ = static_cast<float> (transform [0] * x[i] + transform [1] * y[i] + transform [2] * z[i] + transform [3]);
	 y_ = static_cast<float> (transform [4] * x[i] + transform [5] * y[i] + transform [6] * z[i] + transform [7]);
     z_ = static_cast<float> (transform [8] * x[i] + transform [9] * y[i] + transform [10] * z[i] + transform [11]);
	 x[i]=x_;
	 y[i]=y_;
	 z[i]=z_;


}

void transformCloudGPU_CU(float *x ,float*y ,float *z,float *transform,int size)
{

	float *dev_x,*dev_y,*dev_z,*dev_transform;
	cudaError_t cudaStatus;
	cudaStatus = cudaSetDevice(0);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
        return;
    }

	cudaStatus = cudaMalloc((void**)&dev_x, size * sizeof(float));

	if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        return;
    }

    cudaStatus = cudaMalloc((void**)&dev_y, size * sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        return;
    }

    cudaStatus = cudaMalloc((void**)&dev_z, size * sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        return;
    }

	cudaStatus = cudaMalloc((void**)&dev_transform, 16 * sizeof(float));
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMalloc failed!");
        return;
    }


	cudaStatus = cudaMemcpy(dev_x, x, size * sizeof(float), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

    cudaStatus = cudaMemcpy(dev_y, y, size * sizeof(float), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

	cudaStatus = cudaMemcpy(dev_z, z, size * sizeof(float), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

	cudaStatus = cudaMemcpy(dev_transform, transform, 16 * sizeof(float), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

	transformKernel<<<640,480>>>(dev_x,dev_y,dev_z,dev_transform);
	
	cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
		return;
    }

	cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
		return;
    }

	cudaStatus = cudaMemcpy(x,dev_x, size * sizeof(float), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

    cudaStatus = cudaMemcpy( y,dev_y, size * sizeof(float), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

	cudaStatus = cudaMemcpy( z, dev_z,size * sizeof(float), cudaMemcpyDeviceToHost);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

	/*cudaStatus = cudaDeviceReset();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaDeviceReset failed!");
        return ;
    }*/



}
