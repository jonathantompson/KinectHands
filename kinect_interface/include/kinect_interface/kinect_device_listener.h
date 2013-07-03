//
//  kinect_device_listener.h
// 

#ifndef KINECT_INTERFACE_KINECT_DEVICE_LISTENER_HEADER
#define KINECT_INTERFACE_KINECT_DEVICE_LISTENER_HEADER

#include "OpenNI.h"
		
namespace kinect_interface {
  class KinectDeviceListener : public openni::OpenNI::DeviceStateChangedListener,
    public openni::OpenNI::DeviceDisconnectedListener {
  public:
    KinectDeviceListener();
    ~KinectDeviceListener();

    virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, 
      openni::DeviceState errorState);
	  virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo);

  private:
  };

};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_KINECT_DEVICE_LISTENER_HEADER
