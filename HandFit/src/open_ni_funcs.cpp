//
//  open_ni_dg.h
//
//  Source origionally from OpenNI libraries, then modified by Otavio B. for
//  Computer Vision class code, then adapted by Ken Perlin's lab for KinectHands

#include <cmath>
#include <string>
#include <iostream>
#include "open_ni_funcs.h"

using std::cout;
using std::endl;

#ifndef XN_STATUS_OK
  #define XN_STATUS_OK   ((uint32_t)0)
#endif

namespace kinect {
  
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
                                                       const Vector3D* aProjective,
                                                       Vector3D* aRealWorld) {
    uint32_t nRetVal = XN_STATUS_OK;
    
    /**
     * X_RW = (X_proj / X_res - 1/2) * Z * x_to_z
     */
    
    double fXToZ = GetRealWorldXtoZ();
    double fYToZ = GetRealWorldYtoZ();
    
    for (uint32_t i = 0; i < nCount; ++i)
    {
      double fNormalizedX = (aProjective[i].X / nXRes - 0.5);
      aRealWorld[i].X = (float)(fNormalizedX * aProjective[i].Z * fXToZ);
      
      double fNormalizedY = (0.5 - aProjective[i].Y / nYRes);
      aRealWorld[i].Y = (float)(fNormalizedY * aProjective[i].Z * fYToZ);
      
      aRealWorld[i].Z = aProjective[i].Z;
    }
    
    return nRetVal;
  }
  
  // FROM: XnOpenNI.cpp (and slightly edited)
  uint32_t OpenNIFuncs::xnConvertRealWorldToProjective(uint32_t nCount,
                                                       const Vector3D* aRealWorld,
                                                       Vector3D* aProjective)
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
      aProjective[i].X = (float)fCoeffX * aRealWorld[i].X / aRealWorld[i].Z + nHalfXres;
      aProjective[i].Y = nHalfYres - (float)fCoeffY * aRealWorld[i].Y / aRealWorld[i].Z;
      aProjective[i].Z = aRealWorld[i].Z;
    }
    
    return nRetVal;
  }
  
  uint32_t OpenNIFuncs::ConvertDepthImageToProjective(const uint16_t* aDepth,
                                                      Vector3D* aProjective) {
    int nIndex = 0;
    for (int nY = 0; nY < nYRes; nY += 1) {
      for (int nX = 0; nX < nXRes; nX += 1, nIndex += 1) {
        aProjective[nIndex].X = static_cast<float>(nX);
        aProjective[nIndex].Y = static_cast<float>(nY);
        aProjective[nIndex].Z = aDepth[nIndex];
      }
    }
    return XN_STATUS_OK;
  }
  


}  // namespace kinect


