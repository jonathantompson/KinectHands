//
//  open_ni_dg.h
//
//  This is just a bunch of functions taken out of the OpenNI source code that
//  I need.  It means you can use some of the OpenNI utility functions without
//  having to create a depth generator, user generator, etc...
//
//  Also, all types, such as XnDouble have been converted to just double to
//  remove the need for including the OpenNI headers.
//

#ifndef KINECT_INTERFACE_OPEN_NI_FUNCS_HEADER
#define KINECT_INTERFACE_OPEN_NI_FUNCS_HEADER

#include "jtil/math/math_types.h"
		
namespace kinect_interface {
  
  class OpenNIFuncs {
  public:
    // Top level interface
    
    OpenNIFuncs();
    ~OpenNIFuncs();

    static uint32_t xnConvertProjectiveToRealWorld(uint32_t nCount,
      const float* aProjective, float* aRealWorld);
    static uint32_t xnConvertRealWorldToProjective(uint32_t nCount,
      const float* aRealWorld, float* aProjective);
    static uint32_t ConvertDepthImageToProjective(const uint16_t* aDepth,
      float* aProjective);
    
  private: 
    static const double m_fRealWorldXtoZ;
		static const double m_fRealWorldYtoZ;
    static const double fHFOV;
    static const double fVFOV;
    static const uint32_t nXRes;
    static const uint32_t nYRes;
    static const uint32_t nFPS;

    inline static double GetRealWorldXtoZ() { return m_fRealWorldXtoZ; }
		inline static double GetRealWorldYtoZ() { return m_fRealWorldYtoZ; }
  };
  
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_OPEN_NI_FUNCS_HEADER
