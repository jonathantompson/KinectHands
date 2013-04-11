//
//  kinect_interface.cpp
//
//  Source origionally from OpenNI libraries, then modified by Otavio B. for
//  Computer Vision class code, then adapted by Ken Perlin's lab for KinectHands

#include <mutex>
#include <thread>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "jtil/jtil.h"
#include "jtil/clk/clk.h"
#include "jtil/image_util/image_util.h"
#include "jtil/threading/callback.h"
#include "jtil/threading/thread.h"
#include "jtil/settings/settings_manager.h"
#include "jtil/exceptions/wruntime_error.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/hand_net/hand_net.h"
#include "jtil/threading/thread_pool.h"

#include "OpenNI.h"
#include "OniCAPI.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

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
using openni::PixelFormat;
using openni::DepthPixel;
using namespace jtil::file_io;
using namespace kinect_interface::hand_detector;
using namespace kinect_interface::hand_net;

namespace kinect_interface {
  using hand_detector::HandDetector;

  bool KinectInterface::openni_init_ = false;
  std::mutex KinectInterface::openni_static_lock_;
  uint32_t KinectInterface::openni_devices_open_ = 0;

  KinectInterface::KinectInterface(const char* device_uri) {
    device_ = NULL;
    pts_world_ = NULL;
    pts_uvd_ = NULL;
    labels_ = NULL;
    hand_detector_ = NULL;
    hand_net_ = NULL;
    hands_[0] = NULL;
    hands_[1] = NULL;
    image_io_ = NULL;
    openni_funcs_ = NULL;
    clk_ = NULL;
    depth_dim_.zeros();
    rgb_dim_.zeros();
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      streams_[i] = NULL;
      frames_[i] = NULL;
    }
    
    frame_number_ = 0;
    fps_ = 0;
    tp_ = NULL;
    pts_world_thread_cbs_ = NULL;

    init(device_uri);

    //  Now spawn the Kinect Update Thread
    kinect_running_ = true;
    Callback<void>* threadBody = MakeCallableOnce(
      &KinectInterface::kinectUpdateThread, this);
    kinect_thread_ = MakeThread(threadBody);
  }

  KinectInterface::~KinectInterface() {
    // Stop the openNI device
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      frames_[i]->release();
      streams_[i]->stop();
      streams_[i]->destroy();
    }
    device_->close();

    tp_->stop();
    SAFE_DELETE(tp_);

    // Clean up
    SAFE_DELETE(clk_);
    SAFE_DELETE(hand_detector_);
    SAFE_DELETE(image_io_);
    SAFE_DELETE(hand_net_);
    SAFE_DELETE_ARR(pts_uvd_);
    SAFE_DELETE_ARR(pts_world_);
    SAFE_DELETE_ARR(labels_);
    SAFE_DELETE(openni_funcs_);
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      SAFE_DELETE(frames_[i]);
      SAFE_DELETE(streams_[i]);
    }
    SAFE_DELETE(device_);
    openni_static_lock_.lock();
    openni_devices_open_--;
    if (openni_devices_open_ == 0) {
      shutdownOpenNIStatic();
    }
    openni_static_lock_.unlock();
  }

  // ************************************************************
  // Initialization
  // ************************************************************

  void KinectInterface::initOpenNIStatic() {
    // Initialize OpenNI static methods
    openni_static_lock_.lock();
    if (!openni_init_) {
      openni::Status rc = openni::OpenNI::initialize();

      if (rc == openni::STATUS_OK) {
        openni_init_ = true;
      } else {
        std::stringstream ss;
        ss << "KinectInterface::initOpenNI() - ERROR: ";
        ss << "Failed to initilize OpenNI API: ";
        ss << openni::OpenNI::getExtendedError();
        openni_static_lock_.unlock();
        throw std::wruntime_error(ss.str());
      }
    }
    openni_static_lock_.unlock();
  }

  void KinectInterface::shutdownOpenNIStatic() {
    openni_static_lock_.lock();
    openni::OpenNI::shutdown();
    openni_static_lock_.unlock();
  }

  void KinectInterface::checkOpenNIRC(int rc, const char* error_msg) {
    if (rc != openni::STATUS_OK) {
      std::stringstream ss;
      ss << error_msg << ": " << openni::OpenNI::getExtendedError();
      shutdownOpenNIStatic();
      throw std::wruntime_error(ss.str());
    }
  }

  std::string KinectInterface::formatToString(const int mode) {
    switch (mode) {
    case PixelFormat::PIXEL_FORMAT_DEPTH_1_MM:
      return "PIXEL_FORMAT_DEPTH_1_MM";
    case PixelFormat::PIXEL_FORMAT_DEPTH_100_UM:
      return "PIXEL_FORMAT_DEPTH_100_UM";
    case PixelFormat::PIXEL_FORMAT_SHIFT_9_2:
      return "PIXEL_FORMAT_SHIFT_9_2";
    case PixelFormat::PIXEL_FORMAT_SHIFT_9_3:
      return "PIXEL_FORMAT_SHIFT_9_3";
    case PixelFormat::PIXEL_FORMAT_RGB888:
      return "PIXEL_FORMAT_RGB888";
    case PixelFormat::PIXEL_FORMAT_YUV422:
      return "PIXEL_FORMAT_YUV422";
    case PixelFormat::PIXEL_FORMAT_GRAY8:
      return "PIXEL_FORMAT_GRAY8";
    case PixelFormat::PIXEL_FORMAT_GRAY16:
      return "PIXEL_FORMAT_GRAY16";
    case PixelFormat::PIXEL_FORMAT_JPEG:
      return "PIXEL_FORMAT_JPEG";
    default: 
      shutdownOpenNIStatic();
      throw std::wruntime_error("KinectInterface::formatToString() - ERROR: "
        "Format enum not recognized!");
    }
  }

  void KinectInterface::printMode(const openni::VideoMode& mode) {
    std::cout << "Res: " << mode.getResolutionX() << "x";
    std::cout << mode.getResolutionY() << ", fps = " << mode.getFps();
    std::cout << ", format = " << formatToString(mode.getPixelFormat());
    std::cout << std::endl;
  }

  void KinectInterface::init(const char* device_uri) {
    initOpenNI(device_uri);

    // Create a thread pool for parallelizing the UVD to depth calculations
    uint32_t num_threads = KINECT_INTERFACE_NUM_WORKER_THREADS;
    tp_ = new jtil::threading::ThreadPool(num_threads);
    pts_world_thread_cbs_ = new VectorManaged<Callback<void>*>(num_threads);
    uint32_t n_pixels = depth_dim_[0] * depth_dim_[1];
    uint32_t n_pixels_per_thread = 1 + n_pixels / num_threads;  // round up
    for (uint32_t i = 0; i < num_threads; i++) {
      uint32_t start = i * n_pixels_per_thread;
      uint32_t end = std::min<uint32_t>(((i + 1) * n_pixels_per_thread) - 1,
        n_pixels - 1);
      pts_world_thread_cbs_->pushBack(MakeCallableMany(
        &KinectInterface::convertDepthToWorldThread, this, start, end));
    }

    clk_ = new Clk();
    hand_detector_ = new HandDetector(tp_);
    hand_detector_->init(depth_dim_[0], depth_dim_[1]);

    hand_net_ = new HandNet();
    hand_net_->loadFromFile("./data/handmodel.net.convnet");

    image_io_ = new DepthImagesIO();

    if (device_uri) {
      cout << "Finished initializing device " << device_uri << "..." << endl;
    } else {
      cout << "Finished initializing openNI device..." << endl;
    }
  }

  void KinectInterface::initOpenNI(const char* device_uri) {
    initOpenNIStatic();

    if (device_uri) {
      cout << "Initializing device " << device_uri << "..." << endl;
    } else {
      cout << "Initializing first avaliable openNI device..." << endl;
    }

    // Open a connection to the OpenNI device
    const char* deviceURI = openni::ANY_DEVICE;
    if (device_uri != NULL) {
      deviceURI = device_uri;
    }
    device_ = new openni::Device();
    checkOpenNIRC(device_->open(deviceURI), "Failed to connect to device");

    // Open a connection to the device's depth channel
    initDepth();

    const int depth_size = depth_dim_[0] * depth_dim_[1];
    labels_ = new uint8_t[depth_size];
    pts_world_ = new float[3 * depth_size];
    pts_uvd_ = new float[3 * depth_size];

    // Open a connection to the device's rgb channel
    initRGB();

    // Now update all the frames so they're initialized at least once:
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      streams_[i]->readFrame(frames_[i]);
    }

    checkOpenNIRC(device_->setDepthColorSyncEnabled(true),
      "Failed to set depth/color sync");
  }

  openni::VideoMode KinectInterface::findMaxResYFPSMode(
    const openni::SensorInfo& sensor, const int required_format) const {
    const openni::Array<openni::VideoMode>& modes = 
      sensor.getSupportedVideoModes();
    int ibest = -1;
    std::cout << "Supported Modes:" << std::endl;
    for (int i = 0; i < modes.getSize(); i++) {
      const openni::VideoMode& mode = modes[i];
      printMode(mode);
      PixelFormat format = mode.getPixelFormat();
      int res_x = mode.getResolutionX();
      int res_y = mode.getResolutionY();
      int fps = mode.getFps();
      if (format == required_format) {
        if (ibest == -1 || res_y > modes[ibest].getResolutionY()) {
          ibest = i;
        } else if (res_y == modes[ibest].getResolutionY() && 
          fps > modes[ibest].getFps()){
          ibest = i;
        }
      }
    }
    if (ibest == -1) {
      shutdownOpenNIStatic();
      throw std::wruntime_error("KinectInterface::findMaxResYFPSMode() - "
        "ERROR: Couldn't find a good format!");
    }
    return modes[ibest];
  }

  openni::VideoMode KinectInterface::findMatchingMode(
    const openni::SensorInfo& sensor, const jtil::math::Int2& dim, 
    const int fps, const int format) const {
    const openni::Array<openni::VideoMode>& modes = 
      sensor.getSupportedVideoModes();
    int ibest = -1;
    std::cout << "Supported Modes:" << std::endl;
    for (int i = 0; i < modes.getSize(); i++) {
      const openni::VideoMode& mode = modes[i];
      printMode(mode);
      PixelFormat cur_format = mode.getPixelFormat();
      int res_x = mode.getResolutionX();
      int res_y = mode.getResolutionY();
      int cur_fps = mode.getFps();
      if (cur_format == format && res_x == dim[0] && res_y == dim[1] &&
        fps == fps) {
        ibest = i;
      }
    }
    if (ibest == -1) {
      shutdownOpenNIStatic();
      throw std::wruntime_error("KinectInterface::findMatchingMode() - "
        "ERROR: Couldn't find a matching format!");
    }
    return modes[ibest];
  }

  void KinectInterface::initDepth() {
    streams_[DEPTH] = new openni::VideoStream();
    checkOpenNIRC(streams_[DEPTH]->create(*device_, openni::SENSOR_DEPTH),
      "Failed to connect to device depth channel");
    checkOpenNIRC(streams_[DEPTH]->start(), "Failed to start depth stream");
    if (!streams_[DEPTH]->isValid()) {
      shutdownOpenNIStatic();
      throw std::wruntime_error("KinectInterface::init() - ERROR: "
        "Depth stream is not valid");
    }

    // Find the supported mode with the highest depth resolution.  If more
    // than one format has the max resolution, choose the highest fps mode
    const openni::SensorInfo& depth_sensor_info = 
      streams_[DEPTH]->getSensorInfo();
    std::cout << "Depth ";
    openni::VideoMode mode = findMaxResYFPSMode(depth_sensor_info,
      PixelFormat::PIXEL_FORMAT_DEPTH_1_MM);
    std::cout << "Setting Depth mode: ";
    printMode(mode);
    checkOpenNIRC(streams_[DEPTH]->setVideoMode(mode), 
      "Failed to set depth video mode");
    depth_fps_setting_ = mode.getFps();

    // Now retrieve the current mode to make sure everything went OK.
    openni::VideoMode depth_mode = streams_[DEPTH]->getVideoMode();
    if (depth_mode.getPixelFormat() != PixelFormat::PIXEL_FORMAT_DEPTH_1_MM) {
      throw std::wruntime_error("KinectInterface::init() - ERROR: "
        "Depth stream is not a 16 bit grayscale format!");
    }
    depth_dim_.set(depth_mode.getResolutionX(), depth_mode.getResolutionY());
    frames_[DEPTH] = new openni::VideoFrameRef();

    // Update the OpenNIFuncs constants
    openni_funcs_ = new OpenNIFuncs(depth_dim_[0], depth_dim_[1],
      streams_[DEPTH]->getHorizontalFieldOfView(), 
      streams_[DEPTH]->getVerticalFieldOfView());
  }

  void KinectInterface::initRGB() {
    streams_[RGB] = new openni::VideoStream();
    checkOpenNIRC(streams_[RGB]->create(*device_, openni::SENSOR_COLOR),
      "Failed to connect to device rgb channel");
    checkOpenNIRC(streams_[RGB]->start(), "Failed to start rgb stream");
    if (!streams_[RGB]->isValid()) {
      shutdownOpenNIStatic();
      throw std::wruntime_error("KinectInterface::init() - ERROR: "
        "RGB stream is not valid");
    }

    // Find a resolution mode to match the depth mode
    const openni::SensorInfo& rgb_sensor_info = 
      streams_[RGB]->getSensorInfo();
    std::cout << "RGB ";
    openni::VideoMode mode = findMatchingMode(rgb_sensor_info, depth_dim_,
      depth_fps_setting_, PixelFormat::PIXEL_FORMAT_RGB888);
    std::cout << "Setting RGB mode: ";
    printMode(mode);
    checkOpenNIRC(streams_[RGB]->setVideoMode(mode), 
      "Failed to set rgb video mode");
    rgb_fps_setting_ = mode.getFps();

    // Now retrieve the current mode to make sure everything went OK.
    openni::VideoMode rgb_mode = streams_[RGB]->getVideoMode();
    if (rgb_mode.getPixelFormat() != PixelFormat::PIXEL_FORMAT_RGB888) {
      throw std::wruntime_error("KinectInterface::init() - ERROR: "
        "RGB stream is not a 24 bit rgb format!");
    }
    rgb_dim_.set(rgb_mode.getResolutionX(), rgb_mode.getResolutionY());
    frames_[RGB] = new openni::VideoFrameRef(); 
  }

  char* KinectInterface::getStatusMessage() {
    status_str_[0] = '\0';  // Empty string
    return status_str_;
  }

  void KinectInterface::kinectUpdateThread() {
    double last_frame_time = clk_->getTime();
    double time_accum = 0;
    uint64_t last_frame_number = 0;

    while (kinect_running_) {
      data_lock_.lock();

      //int changed_index;
      //openni::Status rc = openni::OpenNI::waitForAnyStream(streams_,
      //  NUM_STREAMS, &changed_index);
      //checkOpenNIRC(rc, "waitForAnyStream() failed");

      //if (changed_index != DEPTH && changed_index != RGB) {
      //  shutdownOpenNIStatic();
      //  throw std::wruntime_error("waitForAnyStream() failed");
      //}
    
      //streams_[changed_index]->readFrame(frames_[changed_index]);
      //std::cout << "changed_index = " << changed_index << std::cout;

      bool crop_depth_to_rgb;
      GET_SETTING("crop_depth_to_rgb", bool, crop_depth_to_rgb);
      if (crop_depth_to_rgb) {
        device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      } else {
        device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
      }

      for (uint32_t i = 0; i < NUM_STREAMS; i++) {
        streams_[i]->readFrame(frames_[i]);
      }

      convertDepthToWorld();

      bool detect_hands, found_hand;
      GET_SETTING("detect_hands", bool, detect_hands);
      if (detect_hands) {
        found_hand = hand_detector_->findHandLabels((int16_t*)depth(), 
          pts_world_, HDLabelMethod::HDFloodfill, labels_);
        if (!found_hand) {
          memset(labels_, 0, sizeof(labels_[0]) * src_dim);
        }
      }
      
      if (!detect_hands || !found_hand) {
        memset(labels_, 0, sizeof(labels_[0]) * src_dim);
      }

      frame_number_++;
      double frame_time = clk_->getTime();
      time_accum += (frame_time - last_frame_time);
      last_frame_time = frame_time;
      if (time_accum > 0.5) {
        fps_ = (frame_number_ - last_frame_number) / time_accum;
        time_accum = 0;
        last_frame_number = frame_number_;
      }
      data_lock_.unlock();

      std::this_thread::yield();

    }  // end while (app::App::app_running)
    cout << "kinectUpdateThread shutting down..." << endl;

  }

  void KinectInterface::convertDepthToWorld() {
    openni_funcs_->ConvertDepthImageToProjective(depth(), pts_uvd_);

    threads_finished_ = 0;
    for (uint32_t i = 0; i < pts_world_thread_cbs_->size(); i++) {
      tp_->addTask((*pts_world_thread_cbs_)[i]);
    }

    // Wait until the other threads are done
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != pts_world_thread_cbs_->size()) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void KinectInterface::convertDepthToWorldThread(const uint32_t start, 
    const uint32_t end) {
    // There are two ways to do this: one is with the 
    // openni::CoordinateConverter class, but looking through the source on
    // github it looks expensive.  The other is digging into the c src directly
    // and defining our own version.
    float* pts_uvd_start = &pts_uvd_[start*3];
    float* pts_world_start = &pts_world_[start*3];
    const uint32_t count = end - start + 1;
    openni_funcs_->convertDepthToWorldCoordinates(pts_uvd_start, 
      pts_world_start, count);

    //for (uint32_t i = 0; i <= end; i++) {
    //  uint32_t u = i % depth_dim_[0];
    //  uint32_t v = i / depth_dim_[0];
    //  openni::CoordinateConverter::convertDepthToWorld(*streams_[DEPTH], u, v, 
    //    depth[i], &pts_world_[i*3], &pts_world_[i*3+1], &pts_world_[i*3+2]);
    //}

    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  const uint8_t* KinectInterface::rgb() const { 
    return (uint8_t*)frames_[RGB]->getData(); 
  } 

  const uint16_t* KinectInterface::depth() const { 
    return (uint16_t*)frames_[DEPTH]->getData(); 
  }

  void KinectInterface::shutdownKinect() {
    cout << "kinectUpdateThread shutdown requested..." << endl;
    kinect_running_ = false;
    kinect_thread_.join();
  }

  void KinectInterface::detectPose(const int16_t* depth, 
    const uint8_t* labels) {
    hand_net_->calcHandCoeffConvnet(depth, labels);
  }

  const uint8_t* KinectInterface::filteredDecisionForestLabels() const {
    return hand_detector_->labels_filtered();
  }

  const uint8_t* KinectInterface::rawDecisionForestLabels() const {
    return hand_detector_->labels_evaluated();
  }


  const float* KinectInterface::coeff_convnet() const {
    return hand_net_->coeff_convnet();
  }

  const jtil::math::Float3& KinectInterface::uvd_com() const {
    return hand_net_->uvd_com();
  }

}  // namespace kinect


