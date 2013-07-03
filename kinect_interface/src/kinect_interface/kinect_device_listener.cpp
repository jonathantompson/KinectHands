#include <string>
#include <sstream>
#include <iostream>
#include "kinect_interface/kinect_device_listener.h"
#include "jtil/exceptions/wruntime_error.h"
#include "kinect_interface/kinect_interface.h"

namespace kinect_interface {
  void KinectDeviceListener::onDeviceStateChanged(
    const openni::DeviceInfo* pInfo, openni::DeviceState errorState) {
    std::string uri = pInfo->getUri();
    std::cout << "Device " << uri << " state change:" << errorState << std::endl;
    // Find an open KinectInterface that shares this URI
    for (uint32_t i = 0; i < KinectInterface::open_kinects_.size(); i++) {
      if (KinectInterface::open_kinects_[i]->device_uri_ == uri) {
        if (errorState != 0) {
          std::stringstream ss;
          ss << "Device " << i << " is in error state: " << errorState;
          throw std::wruntime_error(ss.str());
        } else {
          // Nothing, everything is OK
        }
      }
    }
  }

  void KinectDeviceListener::onDeviceDisconnected(const openni::DeviceInfo* pInfo) {
    std::string uri = pInfo->getUri();
    std::cout << "Device " << uri << " disconnected" << std::endl;
    // Find an open KinectInterface that shares this URI
    for (uint32_t i = 0; i < KinectInterface::open_kinects_.size(); i++) {
      if (KinectInterface::open_kinects_[i]->device_uri_ == uri) {
        std::stringstream ss;
        ss << "Device " << i << " disconnected!";
        throw std::wruntime_error(ss.str());
      }
    }
  }

  KinectDeviceListener::KinectDeviceListener() {

  }

  KinectDeviceListener::~KinectDeviceListener() {

  }

}  // namespace kinect


