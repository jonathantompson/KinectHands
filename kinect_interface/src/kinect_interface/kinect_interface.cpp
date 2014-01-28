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
#include "jtil/renderer/camera/camera.h"
#include "Kinect.h"
#include <comdef.h>

#define SAFE_FREE(x) do { if (x != NULL) { free(x); x = NULL; } } while (0);
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
      _com_error error(hr); \
      std::string err_str = \
        jtil::string_util::ToNarrowString(error.ErrorMessage()); \
      std::stringstream ss; \
      ss << "ERROR condition returned: " << error_string << ". code=" << hr; \
      ss << ". string=" << err_str; \
      throw std::wruntime_error(ss.str()); \
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
using namespace jtil::math;
using namespace jtil::renderer;

namespace kinect_interface {

  jtil::clk::Clk KinectInterface::shared_clock_;
  Vector<KinectInterface*> KinectInterface::open_kinects_;
  std::recursive_mutex KinectInterface::sdk_static_lock_;

  KinectInterface::KinectInterface(const char* device_id) {
    sdk_static_lock_.lock();
    device_initialized_ = false;
    kinect_sensor_ = NULL;
    depth_frame_reader_ = NULL;
    rgb_frame_reader_ = NULL;
    coord_mapper_ = NULL;
    body_frame_reader_ = NULL;

    // NOTE: depth_ and depth_colored_ are actually one contiguous array:
    // depth_colored_ just indexes into depth_
    depth_colored_ = (uint8_t*)&depth_[depth_dim];

    depth_first_frame_time_ = -1;
    depth_frame_number_ = 0;
    rgb_frame_number_ = 0;
    body_frame_number_ = 0;
    sync_rgb_ = true;
    sync_depth_ = true;
    sync_depth_colored_ = true;
    sync_xyz_ = true;
    sync_body_ = true;

    init(device_id);

    //  Now spawn the Kinect Update Thread
    kinect_running_ = true;
    Callback<void>* threadBody = MakeCallableOnce(
      &KinectInterface::kinectUpdateThread, this);
    kinect_thread_ = MakeThread(threadBody);
    
    sdk_static_lock_.unlock();
  }

  KinectInterface::~KinectInterface() {
    // depth_colored_ is part of depth_ so it doesn't get deleted
    if (kinect_sensor_) {
      kinect_sensor_->Close();
      SafeRelease(coord_mapper_);
      SafeRelease(depth_frame_reader_);
      SafeRelease(rgb_frame_reader_);
      SafeRelease(body_frame_reader_);
      SafeRelease(kinect_sensor_);
    }
  }

  // ************************************************************
  // Initialization
  // ************************************************************

  void KinectInterface::init(const std::string& device_id) {
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

    // Initialize the Kinect
    CALL_SAFE(kinect_sensor_->Open(), "KinectInterface::init() - "
      "Could not open device");

    // Open the depth frame reader
    IDepthFrameSource* depth_frame_source = NULL;
    CALL_SAFE(kinect_sensor_->get_DepthFrameSource(&depth_frame_source),
      "could not open depth frame source");
    depth_frame_source->OpenReader(&depth_frame_reader_);

    // Get the depth frame width, height, etc
    IFrameDescription* depth_frame_description;
    CALL_SAFE(depth_frame_source->get_FrameDescription(&depth_frame_description),
      "could not get depth frame description");
    int w, h;
    CALL_SAFE(depth_frame_description->get_Width(&w), 
      "could not get depth width");
    CALL_SAFE(depth_frame_description->get_Height(&h), 
      "could not get depth height");
    CALL_SAFE(depth_frame_source->get_DepthMinReliableDistance(&min_depth_), 
      "could not get depth min reliable distance");
    CALL_SAFE(depth_frame_source->get_DepthMaxReliableDistance(&max_depth_), 
      "could not get depth max reliable distance");
    CALL_SAFE(depth_frame_description->get_HorizontalFieldOfView(&cur_depth_hfov_),
      "could not get horizontal field of view");
    CALL_SAFE(depth_frame_description->get_VerticalFieldOfView(&cur_depth_vfov_),
      "could not get vertical field of view");

    SafeRelease(depth_frame_source);

    if (w != depth_w || h != depth_h || 
      fabsf(depth_hfov - cur_depth_hfov_) > EPSILON ||
      fabsf(depth_vfov - cur_depth_vfov_) > EPSILON) {
      throw std::wruntime_error("KinectInterface::init() - ERROR: depth_w, "
        " depth_h, depth_hfov or depth_vfov do not match the device!");
    }

    // Open the rgb frame reader
    IColorFrameSource* rgb_frame_source = NULL;
    CALL_SAFE(kinect_sensor_->get_ColorFrameSource(&rgb_frame_source),
      "could not open rgb frame source");
    rgb_frame_source->OpenReader(&rgb_frame_reader_);

    // Get the color frame width, height, etc
    IFrameDescription* rgb_frame_description;
    CALL_SAFE(rgb_frame_source->get_FrameDescription(&rgb_frame_description),
      "could not get depth frame description");
    CALL_SAFE(rgb_frame_description->get_Width(&w), 
      "could not get depth width");
    CALL_SAFE(rgb_frame_description->get_Height(&h), 
      "could not get depth height");
    CALL_SAFE(rgb_frame_description->get_HorizontalFieldOfView(&cur_rgb_hfov_),
      "could not get horizontal field of view");
    CALL_SAFE(rgb_frame_description->get_VerticalFieldOfView(&cur_rgb_vfov_),
      "could not get vertical field of view");

    SafeRelease(depth_frame_source);

    if (w != rgb_w || h != rgb_h || fabsf(rgb_hfov - cur_rgb_hfov_) > EPSILON 
      || fabsf(rgb_vfov - cur_rgb_vfov_) > EPSILON) {
      throw std::wruntime_error("KinectInterface::init() - ERROR: rgb_w, "
        " rgb_h, rgb_hfov or rgb_vfov do not match the device!");
    }

    // Open the mapper between coordinate systems
    CALL_SAFE(kinect_sensor_->get_CoordinateMapper(&coord_mapper_),
      "could not get the coordinate mapper");

    // Parainoid check to make sure any padding of XYZPoint is the same as 
    // CameraSpacePoint (otherwise we might go over the memory bounds).
    CameraSpacePoint dummy_3pt;
    static_cast<void>(dummy_3pt);
    if (sizeof(dummy_3pt) != sizeof(xyz_[0])) {
      throw std::wruntime_error("KinectInterface::KinectInterface() - "
        "ERROR: CameraSpacePoint and XYZPoint sizes don't match!");
    }
    // Do the same for DepthSpacePoints and ColorSpacePoints
    DepthSpacePoint dummy_2pt_d;
    ColorSpacePoint dummy_2pt_c;
    static_cast<void>(dummy_2pt_d);
    static_cast<void>(dummy_2pt_c);
    if (sizeof(dummy_2pt_d) != sizeof(uv_tmp_[0])) {
      throw std::wruntime_error("KinectInterface::KinectInterface() - "
        "ERROR: DepthSpacePoint and XYPoint sizes don't match!");
    }
    if (sizeof(dummy_2pt_c) != sizeof(uv_depth_2_rgb_[0])) {
      throw std::wruntime_error("KinectInterface::KinectInterface() - "
        "ERROR: CameraSpacePoint and XYPoint sizes don't match!");
    }

    // Open the body frame reader
    IBodyFrameSource* body_frame_source = NULL;
    CALL_SAFE(kinect_sensor_->get_BodyFrameSource(&body_frame_source),
      "ERROR: Could not get body frame source.");
      
    CALL_SAFE(body_frame_source->OpenReader(&body_frame_reader_), 
      "ERROR: Could not open the body frame reader.");

    SafeRelease(body_frame_source);

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

  void KinectInterface::getDeviceIDs(VectorManaged<const char*>& ids) {
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
              char* id_cstr = new char[id.length() + 1];
              strncpy(id_cstr, id.c_str(), id.length() + 1);
              ids.pushBack(id_cstr);
            }
          }
        }
      }
    }

    sdk_static_lock_.unlock();
  }

  void KinectInterface::kinectUpdateThread() {
    SetThreadName("KinectInterface::kinectUpdateThread()");

    int64_t last_frame_time = 0;
    int64_t frame_accum = 0;
    int64_t frame_counter = 0;

    while (kinect_running_) {
      waitForDepthFrame(2000);  // Blocking until ready with 2s timeout

      // Check if we're running again (early out opportunity)
      if (!kinect_running_) {
        break;
      }
      
      // ***** Aquire the Depth frame *****
      HRESULT hr = -1;
      IDepthFrame* depth_frame = NULL;
      bool new_depth = false;
      if (sync_depth_ || sync_depth_colored_) { 
        hr = depth_frame_reader_->AcquireLatestFrame(&depth_frame);
      }

      data_lock_.lock();

      if (SUCCEEDED(hr) && depth_frame != NULL) {
        new_depth = true;

        // Copy over the underlying data
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
        // Kinect time stamp is in units of 100ns (or 0.1us)
        depth_frame->get_RelativeTime(&depth_frame_time_);
        if (depth_first_frame_time_ == -1) {
          depth_first_frame_time_ = depth_frame_time_;
        }

        // Update the fps string
        frame_accum += (depth_frame_time_ - last_frame_time);
        last_frame_time = depth_frame_time_;
        frame_counter++;
        if (frame_accum > 10000000) {  // 1 sec in units of 0.1us
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
      }

      // ***** Aquire the RGB frame *****
      IColorFrame* rgb_frame = NULL;
      hr = -1;
      bool new_rgb = false;
      if (sync_rgb_) {
        hr = rgb_frame_reader_->AcquireLatestFrame(&rgb_frame);
      }

      if (SUCCEEDED(hr) && rgb_frame != NULL) {
        new_rgb = true;

        // Copy over the underlying data
        uint32_t rgb_buffer_size;
        RGBQUAD* internal_rgb_buffer;

        ColorImageFormat image_format = ColorImageFormat_None;
        CALL_SAFE(rgb_frame->get_RawColorImageFormat(&image_format),
          "could not get rgb image format");

        switch (image_format) {
        case ColorImageFormat_Rgba:
          {
            CALL_SAFE(rgb_frame->AccessRawUnderlyingBuffer(&rgb_buffer_size, 
              reinterpret_cast<BYTE**>(&internal_rgb_buffer)), 
              "could not get raw rgb buffer");
            for (uint32_t i = 0; i < rgb_dim; i++) {
              rgb_[i * 3] = internal_rgb_buffer[i].rgbRed;
              rgb_[i * 3 + 1] = internal_rgb_buffer[i].rgbGreen;
              rgb_[i * 3 + 2] = internal_rgb_buffer[i].rgbBlue;
            }
          }
          break;
        default:
          {
            // TODO: The default is Luv.  Do we really need to convert all the 
            // time.  How do you set the default source format?
            uint32_t buffer_size = rgb_w * rgb_h * sizeof(RGBQUAD);
            hr = rgb_frame->CopyConvertedFrameDataToArray(buffer_size, 
              reinterpret_cast<BYTE*>(rgb_), ColorImageFormat_Rgba);  
            // Now convert in place from RGBA to rgb
            for (uint32_t i = 0; i < rgb_dim; i++) {
              rgb_[i * 3] = rgb_[i * 4];
              rgb_[i * 3 + 1] = rgb_[i * 4 + 1];
              rgb_[i * 3 + 2] = rgb_[i * 4 + 2];
            }
          }
          break;
        }
        
        rgb_frame_number_++;
        rgb_frame->get_RelativeTime(&rgb_frame_time_);
        rgb_frame->Release();
      }

      // ***** Aquire the colored depth frame *****
      if (new_depth && sync_depth_colored_) {
        CALL_SAFE(coord_mapper_->MapDepthFrameToColorSpace(depth_dim, 
          (UINT16*)depth_, depth_dim, (ColorSpacePoint*)uv_depth_2_rgb_),
          "could not map depth frame to rgb space");
        for (uint32_t i = 0; i < depth_dim; i++) {
          XYPoint* uv = &uv_depth_2_rgb_[i];
          // make sure the depth pixel maps to a valid point in color space
          int32_t rgb_u = (int32_t)(floor(uv->x + 0.5));
          int32_t rgb_v = (int32_t)(floor(uv->y + 0.5));
          if ((rgb_u >= 0) && (rgb_u < (int32_t)rgb_w) && 
            (rgb_v >= 0) && (rgb_v < (int32_t)rgb_h)) {
            uint32_t isrc = rgb_v * rgb_w + rgb_u;
            depth_colored_[i * 3] = rgb_[isrc * 3];
            depth_colored_[i * 3 + 1] = rgb_[isrc * 3 + 1];
            depth_colored_[i * 3 + 2] = rgb_[isrc * 3 + 2];
          } else {
            depth_colored_[i * 3] = 0;
            depth_colored_[i * 3 + 1] = 0;
            depth_colored_[i * 3 + 2] = 0;
          }
        }
      }

      // ***** Aquire the XYZ frame *****
      if (new_depth && sync_xyz_) {
        convertDepthFrameToApproxXYZ(depth_dim, depth_, xyz_);
      }

      // ***** Aquire the body frame *****
      IBodyFrame* body_frame = NULL;
      hr = -1;
      if (sync_body_) {
        hr = body_frame_reader_->AcquireLatestFrame(&body_frame);
      }

      if (SUCCEEDED(hr) && body_frame != NULL) {
        // Get the body frame data
        CALL_SAFE(hr = body_frame->get_RelativeTime(&body_frame_time_),
          "Could not get body frame time");

#if defined(DEBUG) || defined(_DEBUG)
            if (num_users != BODY_COUNT) {
              throw std::wruntime_error("hard-coded user count constant is "
                "incorrect");
            }
#endif

        IBody* bodies[num_users] = {0};
        CALL_SAFE(hr = body_frame->GetAndRefreshBodyData(_countof(bodies), 
          bodies), "Could not get body data");

        for (uint32_t i = 0; i < num_users; i++) {
          IBody* body = bodies[i];
          BOOLEAN tracked = false;
          if (body) {
            CALL_SAFE(body->get_IsTracked(&tracked), "IsTracked query failed");
          }
          if (tracked) {
            user_tracked_[i] = true;
            Joint joints[JointType_Count]; 
            HandState leftHandState = HandState_Unknown;
            HandState rightHandState = HandState_Unknown;

            body->get_HandLeftState(&leftHandState);
            body->get_HandRightState(&rightHandState);

            CALL_SAFE(body->GetJoints(_countof(joints), joints), 
              "Could not get the joints");

#if defined(DEBUG) || defined(_DEBUG)
            if (num_user_joints != JointType_Count) {
              throw std::wruntime_error("hard-coded joint count constant is "
                "incorrect");
            }
#endif

            for (int j = 0; j < _countof(joints); ++j) {
              user_joints_[i][j][0] = joints[j].Position.X;
              user_joints_[i][j][1] = joints[j].Position.Y;
              user_joints_[i][j][2] = joints[j].Position.Z;
            }
          } else {
            user_tracked_[i] = false;
          }
        }

        for (int i = 0; i < _countof(bodies); ++i) {
          SafeRelease(bodies[i]);
        }
        
        body_frame_number_++;
        body_frame->Release();
      }

      data_lock_.unlock();

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

  const uint16_t* KinectInterface::depth() const {
    return depth_;
  }

  const uint8_t* KinectInterface::rgb() const {
    return rgb_;
  }

  const uint8_t* KinectInterface::depth_colored() const {
    return depth_colored_;
  }

  const Float3* KinectInterface::user_joints(const uint32_t user) const {
    return user_joints_[user];
  }

  const bool* KinectInterface::user_tracked() const {
    return user_tracked_;
  }

  const XYZPoint* KinectInterface::xyz() const {
    return xyz_;
  }

  void KinectInterface::waitForDepthFrame(const uint64_t timeout_ms) {
    HANDLE hEvent1 = CreateEvent(NULL,TRUE,FALSE, L"FrameReady");
    depth_frame_reader_->SubscribeFrameArrived((WAITABLE_HANDLE*)&hEvent1);
    if (timeout_ms == -1) {
      WaitForSingleObject(hEvent1, INFINITE);
    } else {
      WaitForSingleObject(hEvent1, (DWORD)timeout_ms);
    }
    ResetEvent(hEvent1);
  }

  void KinectInterface::waitForRGBFrame(const uint64_t timeout_ms) {
    HANDLE hEvent1 = CreateEvent(NULL,TRUE,FALSE, L"FrameReady");
    rgb_frame_reader_->SubscribeFrameArrived((WAITABLE_HANDLE*)&hEvent1);
    if (timeout_ms == -1) {
      WaitForSingleObject(hEvent1, INFINITE);
    } else {
      WaitForSingleObject(hEvent1, (DWORD)timeout_ms);
    }
    ResetEvent(hEvent1);
  }

  void KinectInterface::setSyncRGB(const bool sync_rgb) {
    sync_rgb_ = sync_rgb;
  }

  void KinectInterface::setSyncDepth(const bool sync_depth) {
    sync_depth_ = sync_depth;
  }

  void KinectInterface::setSyncDepthColored(const bool sync_depth_colored) {
    sync_depth_colored_ = sync_depth_colored;
  }

  void KinectInterface::setSyncXYZ(const bool sync_xyz) {
    sync_xyz_ = sync_xyz;
  }

  void KinectInterface::setSyncBody(const bool sync_body) {
    sync_body_ = sync_body;
  }

  void KinectInterface::convertDepthFrameToCamera(const uint32_t n_pts,
    const uint16_t* depth, XYZPoint* xyz) {
    CALL_SAFE(coord_mapper_->MapDepthFrameToCameraSpace(n_pts, 
      (UINT16*)depth, n_pts, (CameraSpacePoint*)xyz),
      "could not map depth frame to xyz (camera) space");
  }

  void KinectInterface::convertCameraToDepthSpace(const uint32_t n_pts, 
    const XYZPoint* xyz, XYPoint* uv_pos) {
    CALL_SAFE(coord_mapper_->MapCameraPointsToDepthSpace(n_pts, 
      (CameraSpacePoint*)xyz, n_pts, (DepthSpacePoint*)uv_pos),
      "could not map xyz (camera) to depth space");
  }

 void KinectInterface::convertUVDToCamera(const uint32_t n_pts, const float* uvd, 
   float* xyz) {
   // Copy the input data into Microsoft's structure
   for (uint32_t i = 0; i < n_pts; i++) {
     uv_tmp_[i].x = uvd[i * 3];
     uv_tmp_[i].y = uvd[i * 3 + 1];
     depth_tmp_[i] = (uint16_t)uvd[i * 3 + 2];
   }
   CALL_SAFE(coord_mapper_->MapDepthPointsToCameraSpace(n_pts, 
     (DepthSpacePoint*)uv_tmp_, n_pts, (UINT16*)depth_tmp_, n_pts,
     (CameraSpacePoint*)xyz_tmp_),
     "could not map xyz (camera) to depth space");
   // Copy the output data out of Microsoft's structure
   for (uint32_t i = 0; i < n_pts; i++) {
     xyz[i * 3] = xyz_tmp_[i].x;
     xyz[i * 3 + 1] = xyz_tmp_[i].y;
     xyz[i * 3 + 2] = xyz_tmp_[i].z;
   }
 }
 void KinectInterface::convertCameraToUVD(const uint32_t n_pts, const float* xyz,
   float* uvd) {
   // Copy the input data into Microsoft's structure
   for (uint32_t i = 0; i < n_pts; i++) {
     xyz_tmp_[i].x = xyz[i * 3];
     xyz_tmp_[i].y = xyz[i * 3 + 1];
     xyz_tmp_[i].z = xyz[i * 3 + 2];
   }
   convertCameraToDepthSpace(n_pts, xyz_tmp_, uv_tmp_);
   // Copy the output data out of Microsoft's structure
   for (uint32_t i = 0; i < n_pts; i++) {
     uvd[i*3] = uv_tmp_[i].x;
     uvd[i*3 + 1] = uv_tmp_[i].y;
     uvd[i*3 + 2] = xyz_tmp_[i].z * 1000.0f;
   }
 }

  void KinectInterface::convertDepthFrameToCamera(const uint32_t n_pts, 
    const uint16_t* depth, float* xyz) {
    convertDepthFrameToCamera(n_pts, depth, xyz_tmp_);
    // Copy the output data out of Microsoft's structure
    for (uint32_t i = 0; i < n_pts; i++) {
      xyz[i * 3] = xyz_tmp_[i].x;
      xyz[i * 3 + 1] = xyz_tmp_[i].y;
      xyz[i * 3 + 2] = xyz_tmp_[i].z;
    }
  }

  // FROM: XnOpenNI.cpp (and slightly edited)
  void KinectInterface::convertUVDToApproxXYZ(const uint32_t n_pts, 
    const float* uvd, float* xyz) {
    float depth_vfov_rad = 2.0f * (float)M_PI * (depth_vfov / 360.0f);
    float depth_hfov_rad = 2.0f * (float)M_PI * (depth_hfov / 360.0f);
    float fXToZ = tanf(depth_hfov_rad/2)*2;
    float fYToZ = tanf(depth_vfov_rad/2)*2;
    
    for (uint32_t i = 0; i < n_pts; ++i) {
      float fNormalizedX = (uvd[i*3] / depth_w - 0.5f);
      xyz[i*3] = (float)(fNormalizedX * uvd[i*3+2] * fXToZ);
      
      float fNormalizedY = (0.5f - uvd[i*3+1] / depth_h);
      xyz[i*3+1] = (float)(fNormalizedY * uvd[i*3+2] * fYToZ);
      
      xyz[i*3+2] = uvd[i*3+2];

      // The new kinect has units in meters
      xyz[i*3] *= (1.0f / 1000.0f);
      xyz[i*3+1] *= (1.0f / 1000.0f);
      xyz[i*3+2] *= (1.0f / 1000.0f);
    }
  }

  // FROM: XnOpenNI.cpp (and slightly edited)
  void KinectInterface::convertDepthFrameToApproxXYZ(const uint32_t n_pts, 
    const uint16_t* depth, XYZPoint* xyz) {
    float depth_vfov_rad = 2.0f * (float)M_PI * (depth_vfov / 360.0f);
    float depth_hfov_rad = 2.0f * (float)M_PI * (depth_hfov / 360.0f);
    float fXToZ = tanf(depth_hfov_rad / 2) * 2;
    float fYToZ = tanf(depth_vfov_rad / 2) * 2;
    
    for (uint32_t i = 0; i < n_pts; i++) {
      float uvd[3] = {(float)(i % depth_w), (float)(i / depth_w), (float)depth[i]};
      float fNormalizedX = (uvd[0] / depth_w - 0.5f);
      xyz[i].x = (float)(fNormalizedX * uvd[2] * fXToZ);
      
      float fNormalizedY = (0.5f - uvd[1] / depth_h);
      xyz[i].y = (float)(fNormalizedY * uvd[2] * fYToZ);
      
      xyz[i].z = uvd[2];

      // The new kinect has units in meters
      xyz[i].x *= (1.0f / 1000.0f);
      xyz[i].y *= (1.0f / 1000.0f);
      xyz[i].z *= (1.0f / 1000.0f);
    }
  }

  // FROM: XnOpenNI.cpp (and slightly edited)
  void KinectInterface::convertDepthFrameToApproxXYZ(const uint32_t n_pts, 
    const uint16_t* depth, float* xyz) {
    float depth_vfov_rad = 2.0f * (float)M_PI * (depth_vfov / 360.0f);
    float depth_hfov_rad = 2.0f * (float)M_PI * (depth_hfov / 360.0f);
    float fXToZ = tanf(depth_hfov_rad / 2) * 2;
    float fYToZ = tanf(depth_vfov_rad / 2) * 2;
    
    for (uint32_t i = 0; i < n_pts; i++) {
      float uvd[3] = {(float)(i % depth_w), (float)(i / depth_w), (float)depth[i]};
      float fNormalizedX = (uvd[0] / depth_w - 0.5f);
      xyz[i*3] = (float)(fNormalizedX * uvd[2] * fXToZ);
      
      float fNormalizedY = (0.5f - uvd[1] / depth_h);
      xyz[i*3+1] = (float)(fNormalizedY * uvd[2] * fYToZ);
      
      xyz[i*3+2] = uvd[2];

      // The new kinect has units in meters
      xyz[i*3] *= (1.0f / 1000.0f);
      xyz[i*3+1] *= (1.0f / 1000.0f);
      xyz[i*3+2] *= (1.0f / 1000.0f);
    }
  }

  // FROM: XnOpenNI.cpp (and slightly edited)
  void KinectInterface::convertXYZToApproxUVD(const uint32_t n_pts, 
    const float* xyz, float* uvd) {
    float depth_vfov_rad = 2.0f * (float)M_PI * (depth_vfov / 360.0f);
    float depth_hfov_rad = 2.0f * (float)M_PI * (depth_hfov / 360.0f);
    float fXToZ = tanf(depth_hfov_rad / 2) * 2;
    float fYToZ = tanf(depth_vfov_rad / 2) * 2;
    
    float fCoeffX = (float)depth_w / fXToZ;
    float fCoeffY = (float)depth_h / fYToZ;
    
    // we can assume resolution is even (so integer div is sufficient)
    uint32_t nHalfXres = depth_w / 2;
    uint32_t nHalfYres = depth_h / 2;
    
    for (uint32_t i = 0; i < n_pts; ++i) {
      uvd[i*3] = (float)fCoeffX * (xyz[i*3]*1000.0f) / (xyz[i*3+2]*1000.0f) + nHalfXres;
      uvd[i*3+1] = nHalfYres - (float)fCoeffY * (xyz[i*3+1]*1000.0f) / (xyz[i*3+2]*1000.0f);
      uvd[i*3+2] = xyz[i*3+2]*1000.0f;
    }
  }

}  // namespace kinect


