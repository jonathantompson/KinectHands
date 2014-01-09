#include <mutex>
#include <thread>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <limits>
#include "jtil/jtil.h"
#include "jtil/clk/clk.h"
#include "jtil/image_util/image_util.h"
#include "jtil/threading/callback.h"
#include "jtil/threading/thread.h"
#include "jtil/settings/settings_manager.h"
#include "jtil/exceptions/wruntime_error.h"
#include "kinect_interface/kinect_interface.h"
#include "jtil/threading/thread_pool.h"
#include "Kinect.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease) {
    if (pInterfaceToRelease != NULL) {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

#define CALL_SAFE(expression, error_string) { \
    HRESULT hr = expression; \
    if (!SUCCEEDED(hr)) { \
      throw std::wruntime_error(error_string); \
    } \
  };

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf_s
#endif
#endif

using std::cout;
using std::endl;
using std::thread;
using jtil::math::Float3;
using jtil::clk::Clk;
using namespace jtil::data_str;
using namespace jtil::threading;
using namespace jtil::file_io;

namespace kinect_interface {

  jtil::clk::Clk KinectInterface::shared_clock_;
  Vector<KinectInterface*> KinectInterface::open_kinects_;
  std::recursive_mutex KinectInterface::sdk_static_lock_;

  KinectInterface::KinectInterface(const std::string& device_id) {
    sdk_static_lock_.lock();
    device_initialized_ = false;
    kinect_sensor_ = NULL;
    frame_reader_ = NULL;
    depth_ = new RGBQUAD[depth_w * depth_h];
    depth_frame_number_ = 0;

    init(device_id);

    //  Now spawn the Kinect Update Thread
    kinect_running_ = true;
    Callback<void>* threadBody = MakeCallableOnce(
      &KinectInterface::kinectUpdateThread, this);
    kinect_thread_ = MakeThread(threadBody);
    
    sdk_static_lock_.unlock();
  }

  KinectInterface::~KinectInterface() {
    SAFE_DELETE_ARR(depth_);
    if (kinect_sensor_) {
      kinect_sensor_->Close();
      SafeRelease(frame_reader_);
      SafeRelease(kinect_sensor_);
    }
  }

  // ************************************************************
  // Initialization
  // ************************************************************

  void KinectInterface::init(const std::string& device_id) {
     HRESULT hr;

    // Make sure that we haven't already opened this device
    // TODO: Store a hashmap of ids to make this faster.
    for (uint32_t i = 0; i < open_kinects_.size(); i++) {
      if (open_kinects_[i]->device_id_ == device_id) {
        throw std::wruntime_error("KinectInterface::init() - ERROR: Device is"
          " already open!");
      }
    }

    kinect_sensor_ = getDeviceByID(device_id);
    if (kinect_sensor_ == NULL) {
      throw std::wruntime_error("KinectInterface::init() - ERROR: Couldn't"
        " find sensor with id: " + device_id);
    }

    // Initialize the Kinect and get the depth reader
    CALL_SAFE(kinect_sensor_->Open(), "KinectInterface::init() - "
      "Could not open device");
    long flags = 0;
    flags |= FrameSourceTypes::FrameSourceTypes_Depth;
    hr = kinect_sensor_->OpenMultiSourceFrameReader(flags, &frame_reader_);
    if (!SUCCEEDED(hr)) {
      throw std::wruntime_error("KinectInterface::init() - ERROR: Could not"
          " get frame reader from device: " + device_id);
    }

    device_initialized_ = true;
    open_kinects_.pushBack(this);

    cout << "Finished initializing device " << device_id << "..." << endl;
  }

  IKinectSensor* KinectInterface::getDeviceByID(const std::string& id) {
    sdk_static_lock_.lock();

    IKinectSensorCollection* sensor_collection = NULL;
    long hr = GetKinectSensorCollection(&sensor_collection);
    if (SUCCEEDED(hr) && sensor_collection != 0) {
      IEnumKinectSensor* sensor_enum = 0;
      hr = sensor_collection->get_Enumerator(&sensor_enum);
      if (SUCCEEDED(hr) || sensor_enum != 0) {
        while (SUCCEEDED(hr)) {
          IKinectSensor* sensor = 0;
          hr = sensor_enum->GetNext( &sensor );
          if ( sensor != 0 ) {
            wchar_t wid[ 48 ];
            if (SUCCEEDED(sensor->get_UniqueKinectId(48, wid))) {
              std::string cur_id = jtil::string_util::ToNarrowString(wid);
              if (id == cur_id) {
                sdk_static_lock_.unlock();
                return sensor;
              }
            }
          }
        }
      }
    }

    sdk_static_lock_.unlock();
    return NULL;
  }

  void KinectInterface::getDeviceIDs(Vector<std::string>& ids) {
    sdk_static_lock_.lock();

    IKinectSensorCollection* sensor_collection = NULL;
    long hr = GetKinectSensorCollection(&sensor_collection);
    if (SUCCEEDED(hr) && sensor_collection != 0) {
      IEnumKinectSensor* sensor_enum = 0;
      hr = sensor_collection->get_Enumerator(&sensor_enum);
      if (SUCCEEDED(hr) || sensor_enum != 0) {
        while (SUCCEEDED(hr)) {
          IKinectSensor* sensor = 0;
          hr = sensor_enum->GetNext( &sensor );
          if ( sensor != 0 ) {
            wchar_t wid[ 48 ];
            if (SUCCEEDED(sensor->get_UniqueKinectId(48, wid))) {
              std::string id = jtil::string_util::ToNarrowString(wid);
              ids.pushBack(id);
            }
          }
        }
      }
    }

    sdk_static_lock_.unlock();
  }

  void KinectInterface::kinectUpdateThread() {
    SetThreadName("KinectInterface::kinectUpdateThread()");

    IMultiSourceFrame* frame;
    // IBodyFrame* body_frame;
    // IColorFrame* color_frame;
    // IInfraredFrame* infrared_frame;
    IDepthFrame* depth_frame;

    while (kinect_running_) {
      frame = NULL;
      HRESULT hr = frame_reader_->AcquireLatestFrame(&frame);

      std::cout << "hr = " << hr << std::endl;
      // TODO: Is the above blocking?????

      // Check if we're running again.
      if (!kinect_running_) {
        break;
      }

      if (SUCCEEDED(hr) && frame != NULL) {
        data_lock_.lock();

        // Get the depth frame
        IDepthFrameReference* frame_ref = NULL;
        CALL_SAFE(frame->get_DepthFrameReference(&frame_ref), 
          "could not get depth frame reference");
        CALL_SAFE(frame_ref->AcquireFrame(&depth_frame), 
          "could not aquire depth frame");
        //CALL_SAFE(depth_frame->AccessUnderlyingBuffer( &depthBufferSize, 
        //  &depthBuffer ), "could not access underlying depth frame buffer");
        
        depth_frame_number_++;

        if ( frame_ref != 0 ) {
          frame_ref->Release();
          frame_ref = NULL;
        }
      
        data_lock_.unlock();
      }

      std::this_thread::yield();

    }  // end while (kinect_running_)
    cout << "kinectUpdateThread shutting down..." << endl;
  }

  void KinectInterface::shutdownKinect() {
    // Technically we don't need to grab the lock to set the shutdown flag,
    // but this way we can control shutdown order.
    data_lock_.lock();
    kinect_running_ = false;
    data_lock_.unlock();
    cout << "kinectUpdateThread shutdown requested..." << std::endl;
    kinect_thread_.join();
  }

}  // namespace kinect


