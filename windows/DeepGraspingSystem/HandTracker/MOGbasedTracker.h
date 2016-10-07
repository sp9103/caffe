#pragma once
#include <opencv2\opencv.hpp>

#include "BlobLabeling.h"

#define  NMIXTURES 2
#define VAR_THRESHOLD 300

class MOGbasedTracker
{
public:
	MOGbasedTracker();
	~MOGbasedTracker();

	void InsertBackGround(cv::Mat Color, cv::Mat Depth);

	cv::Mat calcImage(cv::Mat src, cv::Mat depth, std::vector<cv::Rect> *roi);
	cv::Mat calcImage(cv::Mat src, cv::Mat depth, cv::Mat *dstColor, cv::Mat *dstDepth);

private:
	cv::BackgroundSubtractorMOG2 mog2;
	BlobLabeling _blobLabeling;

	cv::Mat ColorBackGround;
	cv::Mat DepthBackGround;

	int imgWidth;
	int imgHeight;

	cv::Mat DetectColorMap(cv::Mat rgb, cv::Mat subMap);
	cv::Mat DeleteSub(cv::Mat map, cv::Mat src);
	cv::Mat subBackground(cv::Mat srcColor, cv::Mat srcDepth);
	cv::Mat drawBlobMap(cv::Mat src, std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> blobInfo);
	cv::Mat colorCompose(cv::Mat src, cv::Mat bin);
	cv::Mat depthCompose(cv::Mat src, cv::Mat bin);
	cv::Mat createBinMap(std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> src);
	void FindBlob(std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> *dst);
};

