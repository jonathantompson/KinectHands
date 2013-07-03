#include <cmath>
#include <string>
#include <iostream>
#include "kinect_interface/open_ni_funcs.h"

using std::cout;
using std::endl;

#ifndef XN_STATUS_OK
  #define XN_STATUS_OK   ((uint32_t)0)
#endif

namespace kinect_interface {
  
  const double OpenNIFuncs::fHFOV = 1.0144686707507438;
  const double OpenNIFuncs::fVFOV = 0.78980943449644714;
  // FROM: XnOpenNI.cpp (and slightly edited)
  const double OpenNIFuncs::m_fRealWorldXtoZ = tan(OpenNIFuncs::fHFOV/2)*2;
  const double OpenNIFuncs::m_fRealWorldYtoZ = tan(OpenNIFuncs::fVFOV/2)*2;
  const uint32_t OpenNIFuncs::nXRes = 640;
  const uint32_t OpenNIFuncs::nYRes = 480;
  const uint32_t OpenNIFuncs::nFPS = 30;
  
  // FROM: XnOpenNI.cpp (and slightly edited)
  uint32_t OpenNIFuncs::xnConvertProjectiveToRealWorld(uint32_t nCount,
    const float* aProjective, float* aRealWorld) {
    uint32_t nRetVal = XN_STATUS_OK;
    
    /**
     * X_RW = (X_proj / X_res - 1/2) * Z * x_to_z
     */
    
    double fXToZ = GetRealWorldXtoZ();
    double fYToZ = GetRealWorldYtoZ();
    
    for (uint32_t i = 0; i < nCount; ++i)
    {
      double fNormalizedX = (aProjective[i*3] / nXRes - 0.5);
      aRealWorld[i*3] = (float)(fNormalizedX * aProjective[i*3+2] * fXToZ);
      
      double fNormalizedY = (0.5 - aProjective[i*3+1] / nYRes);
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
    
    double fXToZ = GetRealWorldXtoZ();
    double fYToZ = GetRealWorldYtoZ();
    
    double fCoeffX = nXRes / fXToZ;
    double fCoeffY = nYRes / fYToZ;
    
    // we can assume resolution is even (so integer div is sufficient)
    uint32_t nHalfXres = nXRes / 2;
    uint32_t nHalfYres = nYRes / 2;
    
    for (uint32_t i = 0; i < nCount; ++i)
    {
      aProjective[i*3] = (float)fCoeffX * aRealWorld[i*3] / aRealWorld[i*3+2] + nHalfXres;
      aProjective[i*3+1] = nHalfYres - (float)fCoeffY * aRealWorld[i*3+1] / aRealWorld[i*3+2];
      aProjective[i*3+2] = aRealWorld[i*3+2];
    }
    
    return nRetVal;
  }
  
  uint32_t OpenNIFuncs::ConvertDepthImageToProjective(const uint16_t* aDepth,
    float* aProjective) {
    int nIndex = 0;
    for (uint32_t nY = 0; nY < nYRes; nY += 1) {
      for (uint32_t nX = 0; nX < nXRes; nX += 1, nIndex += 1) {
        aProjective[nIndex*3] = static_cast<float>(nX);
        aProjective[nIndex*3+1] = static_cast<float>(nY);
        aProjective[nIndex*3+2] = aDepth[nIndex];
      }
    }
    return XN_STATUS_OK;
  }
  


}  // namespace kinect


