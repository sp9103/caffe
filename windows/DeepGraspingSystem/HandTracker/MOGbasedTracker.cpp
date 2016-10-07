#include "MOGbasedTracker.h"


MOGbasedTracker::MOGbasedTracker()
{
}


MOGbasedTracker::~MOGbasedTracker()
{
}


void MOGbasedTracker::InsertBackGround(cv::Mat Color, cv::Mat Depth){
	imgWidth = Color.cols;
	imgHeight = Color.rows;

	if (Color.channels() == 3)		ColorBackGround = Color.clone();
	else						cv::cvtColor(Color, ColorBackGround, CV_BGRA2BGR);
	DepthBackGround = Depth.clone();

	cv::Mat tempFore;
	mog2.set("nmixtures", 2);
	mog2.setDouble("varThreshold", 300.f);
	mog2.operator()(ColorBackGround, tempFore);
}

cv::Mat MOGbasedTracker::calcImage(cv::Mat src, cv::Mat depth, std::vector<cv::Rect> *roi){
	//if (src.channels() == 4)	cv::cvtColor(src, src, CV_BGRA2BGR);

	//double duration;                                                         //변수 설정
	//duration = static_cast<double>(cv::getTickCount());       //초기 시작 시간 설정
	cv::Mat output;
	//cv::Mat backsub = subBackground(src, depth);
	//cv::Mat redMap = DetectColorMap(src, backsub);
	//cv::Mat MapSub = DeleteSub(redMap, backsub);
	//_blobLabeling.Labeling(100, MapSub);

	////Calculate Object Image
	//std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> BOI;		//blob of interest
	//FindBlob(&BOI);
	//cv::Mat src2 = drawBlobMap(src, BOI);
	//output = colorCompose(src, BOI);

	////cv::imshow("src2", src2);
	////cv::imshow("backsub", backsub);
	////cv::imshow("redmap", redMap);
	////cv::imshow("Mapsub", MapSub);

	return output;
}

cv::Mat MOGbasedTracker::calcImage(cv::Mat src, cv::Mat depth, cv::Mat *dstColor, cv::Mat *dstDepth){
	if (src.channels() == 4)	cv::cvtColor(src, src, CV_BGRA2BGR);

	double duration;                                                         //변수 설정
	duration = static_cast<double>(cv::getTickCount());       //초기 시작 시간 설정
	cv::Mat output;
	cv::Mat backsub = subBackground(src, depth);
	cv::Mat redMap = DetectColorMap(src, backsub);
	cv::Mat MapSub = DeleteSub(redMap, backsub);
	_blobLabeling.Labeling(50, MapSub);

	//Calculate Object Image
	std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> BOI;		//blob of interest
	FindBlob(&BOI);
	cv::Mat bin = createBinMap(BOI);
	cv::Mat src2 = drawBlobMap(src, BOI);
	output = colorCompose(src, bin);
	cv::Mat depthOutput = depthCompose(depth, bin);

	//cv::imshow("src2", src2);
	//cv::imshow("backsub", backsub);
	//cv::imshow("redmap", redMap);
	//cv::imshow("Mapsub", MapSub);
	//cv::imshow("bin", bin);
	//cv::waitKey(0);
	
	if (dstColor != NULL)
		*dstColor = output.clone();
	if (dstDepth != NULL)
		*dstDepth = depthOutput.clone();

	return output;
}

cv::Mat MOGbasedTracker::createBinMap(std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> src){
	cv::Mat output = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);

	for (int i = 0; i < src.size(); i++){
		std::vector<cv::Point2i> blobPixelVec = src.at(i).first;

		for (int j = 0; j < blobPixelVec.size(); j++){
			cv::Point2i blobPoint = blobPixelVec.at(j);
			output.at<uchar>(blobPoint.x, blobPoint.y) = (uchar)255;
		}
	}
	cv::dilate(output, output, cv::Mat(), cv::Point(-1, -1), 4);
	return output;
}

cv::Mat MOGbasedTracker::DetectColorMap(cv::Mat rgb, cv::Mat subMap){
	cv::Mat output(rgb.rows, rgb.cols, CV_8UC1);
	cv::Mat HSV_Input(rgb.rows, rgb.cols, rgb.type());
	cv::cvtColor(rgb, HSV_Input, CV_BGR2HSV);

	for (int h = 0; h < rgb.rows; h++){
		for (int w = 0; w < rgb.cols; w++){
			uchar R = rgb.at<cv::Vec3b>(h, w)[2];
			uchar B = rgb.at<cv::Vec3b>(h, w)[0];
			uchar G = rgb.at<cv::Vec3b>(h, w)[1];

			uchar H = HSV_Input.at<cv::Vec3b>(h, w)[0];
			uchar S = HSV_Input.at<cv::Vec3b>(h, w)[1];
			uchar V = HSV_Input.at<cv::Vec3b>(h, w)[2];

			uchar subMapPixel = subMap.at<uchar>(h, w);

			if (subMapPixel > 0){
				if ((10 > H || H > 155) && S > 20){
					output.at<uchar>(h, w) = (uchar)255;
				}
				else
					output.at<uchar>(h, w) = (uchar)0;

			}
			else{
				output.at<uchar>(h, w) = 0;
			}
		}
	}

	cv::erode(output, output, cv::Mat(), cv::Point(-1, -1), 2);
	cv::dilate(output, output, cv::Mat(), cv::Point(-1, -1), 9);
	//cv::erode(output, output, cv::Mat(), cv::Point(-1, -1), 3);

	return output;
}

cv::Mat MOGbasedTracker::DeleteSub(cv::Mat map, cv::Mat src){
	cv::Mat output(map.rows, map.cols, src.type());

	for (int h = 0; h < map.rows; h++){
		for (int w = 0; w < map.cols; w++){
			uchar srcPixel = src.at<uchar>(h, w);
			uchar mapPixel = map.at<uchar>(h, w);

			if (srcPixel == 0 || mapPixel > 0)
				output.at<uchar>(h, w) = (uchar)0;
			else
				output.at<uchar>(h, w) = (uchar)255;
		}
	}
	//cv::dilate(output, output, cv::Mat(), cv::Point(-1, -1), 2);

	return output;
}

cv::Mat MOGbasedTracker::subBackground(cv::Mat srcColor, cv::Mat srcDepth){
	cv::Mat fore;
	mog2.operator()(srcColor, fore, 0.0f);
	//cv::imshow("fore", fore);

	cv::Mat binMask(160, 160, CV_8UC1);
	for (int h = 0; h < 160; h++){
		for (int w = 0; w < 160; w++){
			if (fore.at<uchar>(h, w) == 255)
				binMask.at<uchar>(h, w) = 255;
			else
				binMask.at<uchar>(h, w) = 0;
		}
	}

	for (int h = 0; h < 160; h++){
		for (int w = 0; w < 160; w++){
			uchar binVal = binMask.at<uchar>(h, w);
			if (binVal != 0){
				float val = abs(srcDepth.at<float>(h, w) - DepthBackGround.at<float>(h, w));
				if (val < 10)
					binMask.at<uchar>(h, w) = 0;
			}
		}
	}

	cv::erode(binMask, binMask, cv::Mat(), cv::Point(-1, -1), 1);
	cv::dilate(binMask, binMask, cv::Mat(), cv::Point(-1, -1), 3);

	return binMask;
}

void MOGbasedTracker::FindBlob(std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> *dst){
	int blobCount = _blobLabeling.getBlobCount();
	dst->clear();

	for (int i = 0; i < blobCount; i++){
		cv::Rect BoundingBox;
		cv::Point2i MinPoint = cv::Point2i(INT_MAX, INT_MAX);
		cv::Point2i MaxPoint = cv::Point2i(-1, -1);
		std::vector<cv::Point2i> blobVec = _blobLabeling.getBlob(i);
		for (int j = 0; j < blobVec.size(); j++){
			cv::Point2i pixelPoint = blobVec.at(j);
			if (MinPoint.x > pixelPoint.y)		MinPoint.x = pixelPoint.y;
			if (MinPoint.y > pixelPoint.x)		MinPoint.y = pixelPoint.x;
			if (MaxPoint.x < pixelPoint.y)		MaxPoint.x = pixelPoint.y;
			if (MaxPoint.y < pixelPoint.x)		MaxPoint.y = pixelPoint.x;
		}
		BoundingBox = cv::Rect(MinPoint, MaxPoint);
		if (BoundingBox.y <= 0 || BoundingBox.x <= 0 || (BoundingBox.x + BoundingBox.width) >= imgWidth - 1 || (BoundingBox.y + BoundingBox.height) >= imgHeight - 1)
			continue;

		dst->push_back(std::make_pair(blobVec, BoundingBox));
	}
}

cv::Mat MOGbasedTracker::drawBlobMap(cv::Mat src, std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> blobInfo){
	cv::Mat retMap = src.clone();

	for (int i = 0; i < blobInfo.size(); i++){
		std::vector<cv::Point2i> blobPixelVec = blobInfo.at(i).first;

		for (int j = 0; j < blobPixelVec.size(); j++){
			cv::Point2i blobPoint = blobPixelVec.at(j);
			retMap.at<cv::Vec3b>(blobPoint.x, blobPoint.y) = cv::Vec3b(((i + 1) * 137) % 255, ((i + 1) * 79) % 255, ((i + 1) * 101) % 255);
		}
	}

	return retMap;
}

cv::Mat MOGbasedTracker::colorCompose(cv::Mat src, cv::Mat bin){
	cv::Mat retMap = ColorBackGround.clone();

	for (int h = 0; h < bin.rows; h++){
		for (int w = 0; w < bin.cols; w++){
			uchar val = bin.at<uchar>(h, w);
			if (val != 0)	retMap.at<cv::Vec3b>(h, w) = src.at<cv::Vec3b>(h, w);
		}
	}

	return retMap;
}

cv::Mat MOGbasedTracker::depthCompose(cv::Mat src, cv::Mat bin){
	cv::Mat retMap = DepthBackGround.clone();

	for (int h = 0; h < bin.rows; h++){
		for (int w = 0; w < bin.cols; w++){
			uchar val = bin.at<uchar>(h, w);
			if (val != 0)	retMap.at<float>(h, w) = src.at<float>(h, w);
		}
	}

	return retMap;
}