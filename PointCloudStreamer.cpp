#include "SocketServer.hpp"
#include "PointCloudStreamer.hpp"





void PointCloudStreamer::initVis()
{
	cloud_viewer_.setBackgroundColor(0, 0, 0);
	cloud_viewer_.registerKeyboardCallback(boost::bind(&PointCloudStreamer::keyboard_callback, this, _1));
}

void PointCloudStreamer::initServer()
{

	server_ = boost::shared_ptr<SocketServer>(new SocketServer(io_service_));
	server_->start();
	socketThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_)));

}

void PointCloudStreamer::grabRGBAframe(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	boost::mutex::scoped_try_lock lock(data_ready);
	if (!lock || exit_)
		return;
	boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());

	pcl::copyPointCloud(*cloud, cloud_);
	size_t j = 0;
	for (size_t i = 0; i < cloud->size(); i++)
	{

		image_[j++] = cloud->points[i].r;
		image_[j++] = cloud->points[i].g;
		image_[j++] = cloud->points[i].b;
	}
	boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration dt = t2 - t1;
	std::cout << "grab a frame in: " << dt.total_milliseconds() / 1000.0 << std::endl;

	data_ready_cond_.notify_one();
}

void PointCloudStreamer::mainLoop()
{
	using namespace openni_wrapper;
	typedef boost::shared_ptr<Image> ImagePtr;
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f_cloud = boost::bind(&PointCloudStreamer::grabRGBAframe, this, _1);

	capture_.registerCallback(f_cloud);

	capture_.start();
	boost::unique_lock<boost::mutex> lock(data_ready);

	while (!exit_&&!cloud_viewer_.wasStopped()/*&&!image_viewer_.wasStopped()*/)
	{
		if (stepFrame_)
		{
			char ch;
			ch = getch();
			if (ch == 'q')
				exit_ = true;
		}

		bool has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(500));

		if (has_data)
		{
			std::cout << "[0]START to fusion!!" << std::endl;

			//fusion2world: gpuSurf+umeyama point-point error registration
			//fusion2World();
			//ransacFusion: ransac{gpuSurf+ymeyama}
			ransacFusion();
			if (!cloud_.empty())
			{
				pcl::copyPointCloud(cloud_, pre_cloud_);
				memcpy(pre_image_, image_, width*height * 3);
			}


			if (enableVis_)
			{


				//image_viewer_.showRGBImage(image_, width, height);
				cloud_viewer_.removeAllPointClouds();
				cloud_viewer_.addPointCloud(cloud_.makeShared(), "world_");
				cloud_viewer_.spinOnce(10);

			}

			if (saveFrame_)
			{

				saveCloud();
			}
			if (enableServer_)
			{
				DispatchData();
			}
			frameid++;
		}


	}
	capture_.stop();

}


size_t PointCloudStreamer::DispatchData()
{
	for (std::vector<SocketStreamer::SocketStreamerPtr>::iterator it = server_->socketList_.begin(); it != server_->socketList_.end(); it++)
	{


		SocketStreamer & itref = *(*it);
		itref.sendData();
	}
	return server_->socketList_.size();

}

bool PointCloudStreamer:: ransacFusion()
{
	if (pre_cloud_.empty())
		return false;

	cv::Mat pre_img_mat(height, width, CV_8UC3, pre_image_);//not transform from rgb2bgr
	cv::Mat img_mat(height, width, CV_8UC3, image_);

	boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
	cv::gpu::GpuMat img_gpu_mat, img_gpu_mat_pre;
	pre_img_mat = pre_img_mat(cv::Rect(0, 0, width / 2, height));
	img_mat = img_mat(cv::Rect(0, 0, width / 2, height));
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


	int iter = 0; float k = 1.0;


	int best_inlier_n = -1;
	Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
	std::vector<cv::DMatch> good_matches;
	pcl::Correspondences best_corr;

	int valid_count = 0;
	while ((iter < k || best_inlier_n < 6) && iter < 5000)
	{
		//std::cout<<"iter: "<<iter<<"start!"<<std::endl;
		pcl::Correspondences corr;
		pcl::PointCloud<pcl::PointXYZRGBA> pcl_keypoints, pcl_keypoints_pre;

		int inlier_ = 0;
		int sampleId[5];
		int i = 0;
		std::vector<cv::DMatch> now_matches;
		int N = (std::min(200, int(matches.size())));
		while (i < 3)
		{
			int sind, tind;
			bool unique_;
			int tmpid;
			do
			{
				tmpid = rand() % N;
				int sid = matches[tmpid].queryIdx;
				int tid = matches[tmpid].trainIdx;

				sind = int(keypoints_pre[sid].pt.y)*width + int(keypoints_pre[sid].pt.x);
				tind = int(keypoints[tid].pt.y)*width + int(keypoints[tid].pt.x);
				unique_ = true;
				for (int j = 0; j < i; j++)
				if (sampleId[j] == tmpid)
					unique_ = false;

			} while (!unique_ || !pcl_isfinite(cloud_.points[tind].x) || !pcl_isfinite(pre_cloud_.points[sind].x));
			sampleId[i] = tmpid;
			pcl_keypoints.push_back(cloud_.points[tind]);
			pcl_keypoints_pre.push_back(pre_cloud_.points[sind]);



			pcl::Correspondence cori;
			cori.index_query = i;
			cori.index_match = i;

			corr.push_back(cori);
			i++;
		}

		Eigen::Matrix4f transform;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> est;
		est.estimateRigidTransformation(pcl_keypoints, pcl_keypoints_pre, corr, transform);


		valid_count = 0;
		pcl::Correspondences now_corr;
		for (int i = 0; i < N; i++)
		{
			pcl::PointXYZRGBA p1, p2;
			int sid = matches[i].queryIdx;
			int tid = matches[i].trainIdx;

			int sind = int(keypoints_pre[sid].pt.y)*width + int(keypoints_pre[sid].pt.x);
			int tind = int(keypoints[tid].pt.y)*width + int(keypoints[tid].pt.x);

			p1 = cloud_.points[tind];
			p2 = pre_cloud_.points[sind];

			p1 = pcl::transformPoint<pcl::PointXYZRGBA>(p1, Eigen::Affine3f(transform));



			if (pcl_isfinite(cloud_.points[tind].x) && pcl_isfinite(pre_cloud_.points[sind].x))
			{

				if ((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z) < 0.03*0.03)
				{
					inlier_++;
					pcl::Correspondence cori;
					cori.index_query = tind;
					cori.index_match = sind;
					now_corr.push_back(cori);
					now_matches.push_back(matches[i]);
				}
				valid_count++;
			}
		}
		if (inlier_ > best_inlier_n)
		{
			best_inlier_n = inlier_;
			best_transform = transform;
			good_matches = now_matches;
			best_corr = now_corr;
		}
		double w = static_cast<double> (best_inlier_n)*1.0 / valid_count;
		double p_no_outliers = 1.0 - pow(w, static_cast<double> (3));
		p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
		p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
		k = log(1 - 0.9f) / log(p_no_outliers);
		iter++;
		//std::cout<<"iter: "<<iter<<"done!"<<std::endl;
	}
	//draw keypoints and correspondence for visualization purpose
	drawMatches(pre_img_mat, keypoints_pre, img_mat, keypoints, good_matches, img_matches);
	imshow("matches", img_matches);
	std::stringstream ss;
	ss << frameid;
	cv::imwrite("data/_img/" + ss.str() + ".jpg", img_matches);
	cvWaitKey(2);

	std::cout << "[2] ransac finished iter: " << iter << "with inlier#: " << best_inlier_n << " of feature# " << valid_count << " finish!!" << std::endl;






	//estimate transform with refined inliers		
	Eigen::Matrix4f transform_refined;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> est;
	est.estimateRigidTransformation(cloud_, pre_cloud_, best_corr, transform_refined);

	//slow version of transformPointCloud
	pcl::transformPointCloud(cloud_, cloud_, transform_refined);



	return true;

}
