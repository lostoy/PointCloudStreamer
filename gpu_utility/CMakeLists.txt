cmake_minimum_required(VERSION 2.8)
project(gpu_utility)

set(Boost_DIR C:/local/boost_1_55_0_vs11_64/include)
set(Boost_LIBRARY_DIR C:/local/boost_1_55_0_vs11_64/lib)

find_package(Boost)
find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(CUDA)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_10,code=sm_10)

#cuda_compile(gpu_utility.cu)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

cuda_add_library(gpu_utility SHARED gpu_utility.cpp gpu_utility.h gpu_utility.cu)



