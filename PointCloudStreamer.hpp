#ifndef POINTCLOUDSTREAMER_H
#define POINTCLOUDSTREAMER_H
#include "SocketServer.hpp"


#include <boost/asio.hpp>
#include <boost/timer.hpp>
#include <boost/chrono.hpp>
#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>

#include "gpu_utility/gpu_utility.h"

#include <iostream>
#include <string>
#include <conio.h>  


class PointCloudStreamer
{
public:
	PointCloudStreamer(pcl::Grabber &capture, bool enableServer = false, bool enableClient = false, bool enableVis = true) :capture_(capture), enableClient_(enableClient), enableServer_(enableServer), enableVis_(enableVis), frameid(0), stepFrame_(false), saveFrame_(true)
	{
		exit_ = false;

		if (enableVis_)
			initVis();
		if (enableServer_)
			initServer();

	}

	~PointCloudStreamer()
	{

		if (enableServer_)
		{
			io_service_.stop();
			socketThread_->join();

		}

	}

	void initVis();

	void initServer();

	void grabRGBAframe(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);


	void mainLoop();
private:

	size_t DispatchData();

	void gicp(pcl::PointCloud<pcl::PointXYZRGBA> & cloud_plus)
	{

		//pcl::GeneralizedIterativeClosestPoint< pcl::PointXYZRGBA, pcl::PointXYZRGBA > gicp_;
		pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> gicp_;
		std::vector<int> ind;
		pcl::removeNaNFromPointCloud(pre_cloud_, pre_cloud_, ind);
		pcl::removeNaNFromPointCloud(cloud_, cloud_plus, ind);
		gridFilter(pre_cloud_, 0.05f);
		gridFilter(cloud_plus, 0.05f);
		gicp_.setInputSource(cloud_plus.makeShared());
		gicp_.setInputTarget(pre_cloud_.makeShared());

		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
		gicp_.align(cloud_plus, transform);
		pcl::transformPointCloud(cloud_, cloud_, transform);

	}
	bool ransacFusion();

	bool fusion2World()
	{
		if (pre_cloud_.empty())
			return false;
		boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());

		cv::Mat pre_img_mat(height, width, CV_8UC3, pre_image_);//not transform from rgb2bgr
		cv::Mat img_mat(height, width, CV_8UC3, image_);
		


		std::vector<cv::KeyPoint> pre_keypoints, keypoints;
		/*cv::Ptr<cv::FeatureDetector> fdect = cv::FeatureDetector::create("FAST");
		fdect->detect(pre_img_mat, pre_keypoints);
		fdect->detect(img_mat, keypoints);*/

		cv::SurfFeatureDetector detc(400);

		detc.detect(img_mat, keypoints);
		detc.detect(pre_img_mat, pre_keypoints);


		cv::Mat pre_img_desc, img_desc;

		/*cv::drawKeypoints(img_mat, keypoints, img_mat);
		cv::drawKeypoints(pre_img_mat, pre_keypoints, pre_img_mat);
		imshow("pre_img", pre_img_mat);
		imshow("img",img_mat);
		cvWaitKey(5);
		*/

		cv::SurfDescriptorExtractor extractor;
		extractor.compute(pre_img_mat, pre_keypoints, pre_img_desc);
		extractor.compute(img_mat, keypoints, img_desc);

		boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
		boost::posix_time::time_duration dt = t2 - t1;
		std::cout << "extract a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;

		std::vector<cv::DMatch> matches;

		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
		matcher->match(pre_img_desc, img_desc, matches);

		boost::posix_time::ptime t3(boost::posix_time::microsec_clock::local_time());
		dt = t3 - t2;
		std::cout << "match a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;

		std::sort(matches.begin(), matches.end());
		pcl::Correspondences corr;
		pcl::PointCloud<pcl::PointXYZRGBA> pcl_keypoints, pcl_keypoints_pre;
		int j = 0;
		for (size_t i = 0; i < 32; i++)
		{
			int sid = matches[i].queryIdx;
			int tid = matches[i].trainIdx;

			int sind = int(pre_keypoints[sid].pt.y)*width + int(pre_keypoints[sid].pt.x);
			int tind = int(keypoints[tid].pt.y)*width + int(keypoints[tid].pt.x);


			if (pcl_isfinite(cloud_.points[tind].x) && pcl_isfinite(pre_cloud_.points[sind].x))
			{
				pcl_keypoints.push_back(cloud_.points[tind]);
				pcl_keypoints_pre.push_back(pre_cloud_.points[sind]);

				pcl::Correspondence cori;
				cori.index_query = j;
				cori.index_match = j;
				j++;
				corr.push_back(cori);
			}
		}

		Eigen::Matrix4f transform;

		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> est;



		est.estimateRigidTransformation(pcl_keypoints, pcl_keypoints_pre, corr, transform);

		pcl::transformPointCloud(cloud_, cloud_, transform);

		boost::posix_time::ptime t4(boost::posix_time::microsec_clock::local_time());
		dt = t4 - t3;
		std::cout << "fusion a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;
	}

	bool fusion2World_GPU()
	{
		if (pre_cloud_.empty())
			return false;

		cv::Mat pre_img_mat(height, width, CV_8UC3, pre_image_);//not transform from rgb2bgr
		cv::Mat img_mat(height, width, CV_8UC3, image_);


		boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
		cv::gpu::GpuMat img_gpu_mat, img_gpu_mat_pre;
		pre_img_mat = pre_img_mat(cv::Rect(0, 0, width, height));
		img_mat = img_mat(cv::Rect(0, 0, width, height));

		cv::cvtColor(pre_img_mat, pre_img_mat, CV_RGB2GRAY);
		cv::cvtColor(img_mat, img_mat, CV_RGB2GRAY);

		img_gpu_mat.upload(img_mat);
		img_gpu_mat_pre.upload(pre_img_mat);

		//cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

		cv::gpu::SURF_GPU surf;

		cv::gpu::GpuMat keypoints_gpu, keypoints_gpu_pre;
		cv::gpu::GpuMat img_desc_gpu, img_desc_gpu_pre;

		surf(img_gpu_mat, cv::gpu::GpuMat(), keypoints_gpu, img_desc_gpu);
		surf(img_gpu_mat_pre, cv::gpu::GpuMat(), keypoints_gpu_pre, img_desc_gpu_pre);

		cv::gpu::BFMatcher_GPU matcher_gpu(cv::NORM_L2);

		cv::gpu::GpuMat trainIdx, distance;
		matcher_gpu.matchSingle(img_desc_gpu_pre, img_desc_gpu, trainIdx, distance);

		std::vector<cv::KeyPoint> keypoints, keypoints_pre;
		std::vector<float>img_desc, img_desc_pre;

		std::vector<cv::DMatch> matches;

		surf.downloadKeypoints(keypoints_gpu, keypoints);
		surf.downloadKeypoints(keypoints_gpu_pre, keypoints_pre);

		surf.downloadDescriptors(img_desc_gpu, img_desc);
		surf.downloadDescriptors(img_desc_gpu_pre, img_desc_pre);

		cv::gpu::BFMatcher_GPU::matchDownload(trainIdx, distance, matches);

		boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
		boost::posix_time::time_duration dt = t2 - t1;
		std::cout << "[1]match a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;

		std::sort(matches.begin(), matches.end());

		std::vector<cv::DMatch> good_matches;

		pcl::Correspondences corr;
		pcl::PointCloud<pcl::PointXYZRGBA> pcl_keypoints, pcl_keypoints_pre;
		int j = 0;
		for (size_t i = 0; i < matches.size() && j < 5; i++)
		{
			int sid = matches[i].queryIdx;
			int tid = matches[i].trainIdx;

			int sind = int(keypoints_pre[sid].pt.y)*width + int(keypoints_pre[sid].pt.x);
			int tind = int(keypoints[tid].pt.y)*width + int(keypoints[tid].pt.x);


			if (pcl_isfinite(cloud_.points[tind].x) && pcl_isfinite(pre_cloud_.points[sind].x))
			{
				good_matches.push_back(matches[i]);
				pcl_keypoints.push_back(cloud_.points[tind]);
				pcl_keypoints_pre.push_back(pre_cloud_.points[sind]);

				pcl::Correspondence cori;
				cori.index_query = j;
				cori.index_match = j;
				j++;
				corr.push_back(cori);
			}
		}
		std::cout << "[3.5]number of correspondence: " << j << std::endl;
		drawMatches(pre_img_mat, keypoints_pre, img_mat, keypoints, good_matches, img_matches);
		imshow("matches", img_matches);
		std::stringstream ss;
		ss << frameid;
		cv::imwrite("data/" + ss.str() + ".jpg", img_matches);
		cvWaitKey(2);
		Eigen::Matrix4f transform;

		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> est;


		boost::posix_time::ptime t3(boost::posix_time::microsec_clock::local_time());
		dt = t3 - t2;
		std::cout << "[3]get correspondence in :" << dt.total_milliseconds() / 1000.0 << std::endl;

		est.estimateRigidTransformation(pcl_keypoints, pcl_keypoints_pre, corr, transform);

		boost::posix_time::ptime t4(boost::posix_time::microsec_clock::local_time());
		dt = t4 - t3;
		std::cout << "[4]estimate transfrom in: " << dt.total_milliseconds() / 1000.0 << std::endl;


		//!!!!!cuda code needs refinement!!!!

		//pcl::transformPointCloud(cloud_, cloud_, transform);
		/*float transform_[16];
		for (int i=0;i<16;i++)
		transform_[i]=transform.coeff(i);*/
		/*pcl::copyPointCloud(cloud_,world_);*/
		//transformCloudGPU(cloud_,cloud_,transform);
		//pcl::PointCloud<pcl::PointXYZRGBA> cloud_after;

		//slow version of transformpointcloud
		pcl::transformPointCloud(cloud_, cloud_, transform);


		boost::posix_time::ptime t5(boost::posix_time::microsec_clock::local_time());
		dt = t5 - t4;

		std::cout << "[5]transform cloud: " << dt.total_milliseconds() / 1000.0 << std::endl;
		return true;
	}
	void onlyicp()
	{
		if (pre_cloud_.empty())
			return;

		gicp(cloud_plus);
		std::cout << "fusion finished!!" << std::endl;
	}
	void gridFilter(pcl::PointCloud<pcl::PointXYZRGBA> &cloud, float scale)
	{
		pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> sog;
		sog.setInputCloud(cloud.makeShared());
		sog.setLeafSize(scale, scale, scale);
		sog.filter(cloud);

	}
	void saveCloud()
	{
		if (cloud_.empty())
			return;
		std::stringstream ss;
		ss << frameid;
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_f;
		std::vector<int> ind;
		pcl::removeNaNFromPointCloud(cloud_, cloud_f, ind);
		pcl::io::savePLYFileBinary("data/" + ss.str() + "_cloud.ply", cloud_f);

	}
	void keyboard_callback(const pcl::visualization::KeyboardEvent &e)
	{
		if (e.keyUp())
		{
			int key = e.getKeyCode();
			if (key == (int)'q')
				exit_ = true;
			if (key == (int)'s')
			{

				saveCloud();
			}

		}
	}


	static const int width = 640, height = 480;
	boost::asio::io_service io_service_;
	SocketServerPtr server_;
	SocketClient::SocketClientPtr client_;

	pcl::Grabber& capture_;

	cv::Mat img_matches;

	pcl::visualization::PCLVisualizer cloud_viewer_;
	//pcl::visualization::ImageViewer image_viewer_;

	pcl::PointCloud<pcl::PointXYZRGBA> cloud_;
	pcl::PointCloud<pcl::PointXYZRGBA> pre_cloud_;
	pcl::PointCloud<pcl::PointXYZRGBA> world_;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud_plus;

	unsigned char image_[width*height * 3];
	unsigned char pre_image_[width*height * 3];

	bool enableServer_;
	bool enableClient_;
	bool enableVis_;
	bool exit_;
	bool stepFrame_;
	bool saveFrame_;

	boost::mutex data_ready;
	boost::condition_variable data_ready_cond_;

	boost::shared_ptr<boost::thread> socketThread_;

	size_t frameid;
};

#endif