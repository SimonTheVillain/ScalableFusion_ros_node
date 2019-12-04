//
// Created by simon on 29.11.19.
//

#ifndef CATKIN_WS_CAM_NODE_LISTENER_H
#define CATKIN_WS_CAM_NODE_LISTENER_H

class CamNodeListener : public video::Camera{
public:
	std::mutex mutex;
	bool new_rgb = false;
	bool new_depth = false;


public:

	//! Dataset-specific extension of Source::readFrame()
	bool readFrame(){
		//cout << " is this even called" << endl;

		if(new_rgb && new_depth){
			//cout << "got new frame" << endl;
			mutex.lock();
			frame.rgb = rgb_in;
			frame.depth = depth_in;
			new_rgb = new_depth  = false;
			mutex.unlock();
			return true;
		}
		return false;
	}


	CamNodeListener() {
		intrinsics_rgb_ = Vector4f(530, 530, 320, 240);
		intrinsics_depth_ = Vector4f(570, 570, 320, 240);

		intrinsics_rgb_ = Vector4f(538.39, 538.085, 315, 233);
		intrinsics_depth_ = Vector4f(538.39, 538.085, 315, 233);
		is_running_ = true;
	}

	//! Implementation of Source::readRgb_()
	bool readRgb_() {
		return new_rgb;
	}
	//! Implementation of Source::readDepth_()
	bool readDepth_() {
		return new_depth;
	}
	//! Implementation of Source::readExposure_()
	bool readExposure_() {
		return false;
	}
	//! Implementation of Source::readOdom_()
	bool readOdom_() {
		return false;
	}

	bool setExposure(float exposure){
		//not supported
		return false;
	}

	cv::Mat rgb_in;
	cv::Mat depth_in;





};

#endif //CATKIN_WS_CAM_NODE_LISTENER_H
