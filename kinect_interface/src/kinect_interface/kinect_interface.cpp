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
    depth_ = new uint16_t[depth_w * depth_h];
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
    int64_t last_frame_time = 0;
    int64_t frame_accum = 0;
    int64_t frame_counter = 0;

    while (kinect_running_) {
      frame = NULL;
      HRESULT hr = frame_reader_->AcquireLatestFrame(&frame);

      // Check if we're running again.
      if (!kinect_running_) {
        break;
      }

      if (SUCCEEDED(hr) && frame != NULL) {
        data_lock_.lock();

        // Get a reference to the depth frame
        IDepthFrameReference* frame_ref = NULL;
        CALL_SAFE(frame->get_DepthFrameReference(&frame_ref), 
          "could not get depth frame reference");
        CALL_SAFE(frame_ref->AcquireFrame(&depth_frame), 
          "could not aquire depth frame");
        uint32_t depth_buffer_size;
        uint16_t* internal_depth_buffer;
        CALL_SAFE(depth_frame->AccessUnderlyingBuffer(&depth_buffer_size, 
          &internal_depth_buffer ), 
          "could not access underlying depth frame buffer");

        // Now copy the internal data to our data structure
        if (depth_buffer_size != depth_dim) {
          throw std::wruntime_error("KinectInterface::kinectUpdateThread() - "
            "ERROR: Depth buffer size does not match!");
        }
        memcpy(depth_, internal_depth_buffer, sizeof(depth_[0]) * depth_dim);
        depth_frame_number_++;
        depth_frame->get_RelativeTime(&depth_frame_time_);

        // Update the fps string
        frame_accum += (depth_frame_time_ - last_frame_time);
        last_frame_time = depth_frame_time_;
        frame_counter++;
        if (frame_accum > 10000000) {
          std::cout << "frame_accum = " << frame_accum << std::endl;
          std::cout << "frame_counter = " << frame_counter << std::endl;
          // Update every 1 second
#if defined(WIN32) || defined(_WIN32)
#pragma warning(push)
#pragma warning(disable:4996)
#endif
          float fps = 1e7f * (float)frame_counter / (float)frame_accum;
          snprintf(kinect_fps_str_, 255, "%.2f", fps);
          frame_counter = 0;
          frame_accum = 0;
#if defined(WIN32) || defined(_WIN32)
#pragma warning(pop)
#endif
        }


        if (depth_frame != NULL) {
          depth_frame->Release();
          depth_frame = NULL;
        }
        if (frame_ref != NULL) {
          frame_ref->Release();
          frame_ref = NULL;
        }
        if (frame != NULL) {
          frame->Release();
          frame = NULL;
        }
        data_lock_.unlock();

        // New data, yield to let someone else do work
        std::this_thread::yield();
      } else {
        // No new data
        // Sleep rather than yield (much less aggressive)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

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

  const uint16_t* KinectInterface::depth() const {
    return depth_;
  }

}  // namespace kinect


