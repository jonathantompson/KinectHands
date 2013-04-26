#include <cmath>
#include <string>
#include <iostream>
#include "kinect_interface/open_ni_funcs.h"
#include "jtil/exceptions/wruntime_error.h"

using std::cout;
using std::endl;

#ifndef XN_STATUS_OK
  #define XN_STATUS_OK   ((uint32_t)0)
#endif

namespace kinect_interface {
  
  // Kinect constants FROM: XnOpenNI.cpp (and slightly edited)
  const double OpenNIFuncs::fHFOV_kinect_ = 1.0144686707507438;
  const double OpenNIFuncs::fVFOV_kinect_ = 0.78980943449644714;

  //const float OpenNIFuncs::fHFOV_primesense_109 = 1.017074704170227f;  // True depth
  //const float OpenNIFuncs::fVFOV_primesense_109 = 0.7919895648956299f;  // True depth
  //const float OpenNIFuncs::fHFOV_primesense_109 = 1.075848937034607f;  // approx rgb
  //const float OpenNIFuncs::fVFOV_primesense_109 = 0.8383198380470276f;  // approx rgb
  const float OpenNIFuncs::fHFOV_primesense_109 = 1.076187422640391f;  // measured rgb
  const float OpenNIFuncs::fVFOV_primesense_109 = 0.844611400289787f;  // measured rgb

  const double OpenNIFuncs::m_fRealWorldXtoZ_kinect_ = tan(OpenNIFuncs::fHFOV_kinect_/2)*2;
  const double OpenNIFuncs::m_fRealWorldYtoZ_kinect_ = tan(OpenNIFuncs::fVFOV_kinect_/2)*2;
  const uint32_t OpenNIFuncs::nXRes_kinect_ = 640;
  const uint32_t OpenNIFuncs::nYRes_kinect_ = 480;
  const uint32_t OpenNIFuncs::nFPS_kinect_ = 30;
  const uint32_t OpenNIFuncs::nXRes_primesense_109 = 640;
  const uint32_t OpenNIFuncs::nYRes_primesense_109 = 480;

  OpenNIFuncs::OpenNIFuncs(const uint32_t nXRes, const uint32_t nYRes, 
      const float hFOV, const float vFOV) {
    nXRes_ = (float)nXRes;
    nYRes_ = (float)nYRes;
    fHFOV_ = hFOV;
    fVFOV_ = vFOV;
    xzFactor_ = tanf(fHFOV_ / 2.0f) * 2.0f;
	  yzFactor_ = tanf(fVFOV_ / 2.0f) * 2.0f;
    halfResX_ = nXRes_ / 2.0f;
	  halfResY_ = nYRes_ / 2.0f;
	  coeffX_ = nXRes_ / xzFactor_;
	  coeffY_ = nYRes_ / yzFactor_;
  }

  OpenNIFuncs::OpenNIFuncs() {
    nXRes_ = (float)nXRes_primesense_109;
    nYRes_ = (float)nYRes_primesense_109;
    fHFOV_ = fHFOV_primesense_109;
    fVFOV_ = fVFOV_primesense_109;
    xzFactor_ = tan(fHFOV_ / 2.0f) * 2.0f;
	  yzFactor_ = tan(fVFOV_ / 2.0f) * 2.0f;
    halfResX_ = nXRes_ / 2.0f;
	  halfResY_ = nYRes_ / 2.0f;
	  coeffX_ = nXRes_ / xzFactor_;
	  coeffY_ = nYRes_ / yzFactor_;
  }

  OpenNIFuncs::~OpenNIFuncs() {

  }

  // FROM: XnOpenNI.cpp (and slightly edited)
  uint32_t OpenNIFuncs::xnConvertProjectiveToRealWorld(uint32_t nCount,
    const float* aProjective, float* aRealWorld) {
    uint32_t nRetVal = XN_STATUS_OK;
    
    /**
     * X_RW = (X_proj / X_res - 1/2) * Z * x_to_z
     */
    
    double fXToZ = GetRealWorldXtoZKinect();
    double fYToZ = GetRealWorldYtoZKinect();
    
    for (uint32_t i = 0; i < nCount; ++i)
    {
      double fNormalizedX = (aProjective[i*3] / nXRes_kinect_ - 0.5);
      aRealWorld[i*3] = (float)(fNormalizedX * aProjective[i*3+2] * fXToZ);
      
      double fNormalizedY = (0.5 - aProjective[i*3+1] / nYRes_kinect_);
      aRealWorld[i*3+1] = (float)(fNormalizedY * aProjective[i*3+2] * fYToZ);
      
      aRealWorld[i*3+2] = aProjective[i*3+2];
    }
    
    return nRetVal;
  }
  
  // FROM: XnOpenNI.cpp (and slightly edited)
  uint32_t OpenNIFuncs::xnConvertRealWorldToProjective(uint32_t nCount,
    const float* aRealWorld, float* aProjective)
  {
    uint32_t nRetVal = XN_STATUS_OK;
    
    /**
     * X_proj = X_res * (X_RW / (z*x_to_z) + 1/2)
     *
     *		= X_res / x_to_z * X_RW / z + X_res/2     (more efficient)
     */
    
    double fXToZ = GetRealWorldXtoZKinect();
    double fYToZ = GetRealWorldYtoZKinect();
    
    double fCoeffX = nXRes_kinect_ / fXToZ;
    double fCoeffY = nYRes_kinect_ / fYToZ;
    
    // we can assume resolution is even (so integer div is sufficient)
    uint32_t nHalfXres = nXRes_kinect_ / 2;
    uint32_t nHalfYres = nYRes_kinect_ / 2;
    
    for (uint32_t i = 0; i < nCount; ++i)
    {
      aProjective[i*3] = (float)fCoeffX * aRealWorld[i*3] / aRealWorld[i*3+2] + nHalfXres;
      aProjective[i*3+1] = nHalfYres - (float)fCoeffY * aRealWorld[i*3+1] / aRealWorld[i*3+2];
      aProjective[i*3+2] = aRealWorld[i*3+2];
    }
    
    return nRetVal;
  }
  
  uint32_t OpenNIFuncs::ConvertDepthImageToProjectiveKinect(const uint16_t* aDepth,
    float* aProjective) {
    int nIndex = 0;
    for (uint32_t nY = 0; nY < nYRes_kinect_; nY += 1) {
      for (uint32_t nX = 0; nX < nXRes_kinect_; nX += 1, nIndex += 1) {
        aProjective[nIndex*3] = static_cast<float>(nX);
        aProjective[nIndex*3+1] = static_cast<float>(nY);
        aProjective[nIndex*3+2] = aDepth[nIndex];
      }
    }
    return XN_STATUS_OK;
  }

  void OpenNIFuncs::ConvertDepthImageToProjective(const uint16_t* aDepth,
    float* aProjective) {
    int nIndex = 0;
    for (uint32_t nY = 0; nY < nYRes_; nY += 1) {
      for (uint32_t nX = 0; nX < nXRes_; nX += 1, nIndex += 1) {
        aProjective[nIndex*3] = static_cast<float>(nX);
        aProjective[nIndex*3+1] = static_cast<float>(nY);
        aProjective[nIndex*3+2] = aDepth[nIndex];
      }
    }
  }

  // From OniStream.cpp (and edited)
  //h ttps://github.com/OpenNI/OpenNI2/blob/master/Source/Core/OniStream.cpp
  void OpenNIFuncs::convertDepthToWorldCoordinates(const float* uvd, float* xyz, 
    const uint32_t nCount) {
    for (uint32_t i = 0; i < nCount; i++) {
      float normalizedX = uvd[i*3] / nXRes_ - .5f;
	    float normalizedY = .5f - uvd[i*3+1] / nYRes_;
      xyz[i*3] = normalizedX * uvd[i*3+2] * xzFactor_;
	    xyz[i*3+1] = normalizedY * uvd[i*3+2] * yzFactor_;
	    xyz[i*3+2] = uvd[i*3+2];
    }
  }
  void OpenNIFuncs::convertWorldToDepthCoordinates(const float* xyz, float* uvd, 
    const uint32_t nCount) {
    for (uint32_t i = 0; i < nCount; i++) {
      uvd[3*i] = coeffX_ * xyz[3*i] / xyz[3*i+2] + halfResX_;
	    uvd[3*i+1] = halfResY_ - coeffY_ * xyz[3*i+1] / xyz[3*i+2];
	    uvd[3*i+2] = xyz[3*i+2];
    }
  }


}  // namespace kinect


