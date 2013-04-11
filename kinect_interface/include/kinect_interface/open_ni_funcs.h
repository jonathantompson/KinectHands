//
//  open_ni_dg.h
//
//  This is just a bunch of functions taken out of the OpenNI2 source code that
//  I need.
//

#ifndef KINECT_INTERFACE_OPEN_NI_FUNCS_HEADER
#define KINECT_INTERFACE_OPEN_NI_FUNCS_HEADER

#include "jtil/math/math_types.h"
		
namespace kinect_interface {
  
  class OpenNIFuncs {
  public:
    // Top level interface
    
    OpenNIFuncs(const uint32_t nXRes, const uint32_t nYRes, 
      const float hFOV, const float vFOV);
    OpenNIFuncs();  // The default is for the primesense 1.09
    ~OpenNIFuncs();

    // This is for the new Primesense 1.09 sensor
    void convertDepthToWorldCoordinates(const float* uvd, float* xyz, 
      const uint32_t nCount);
    void convertWorldToDepthCoordinates(const float* xyz, float* uvd, 
      const uint32_t nCount);
    void ConvertDepthImageToProjective(const uint16_t* aDepth,
      float* aProjective);

    // The following are for the Kinect
    static uint32_t xnConvertProjectiveToRealWorld(uint32_t nCount,
      const float* aProjective, float* aRealWorld);
    static uint32_t xnConvertRealWorldToProjective(uint32_t nCount,
      const float* aRealWorld, float* aProjective);
    static uint32_t ConvertDepthImageToProjectiveKinect(const uint16_t* aDepth,
      float* aProjective);
    
    // Primesense 1.09 constants
    void update109Constants();

  private:
    // Kinect constants
    static const double m_fRealWorldXtoZ_kinect_;
		static const double m_fRealWorldYtoZ_kinect_;
    static const double fHFOV_kinect_;
    static const double fVFOV_kinect_;
    static const uint32_t nXRes_kinect_;
    static const uint32_t nYRes_kinect_;
    static const uint32_t nFPS_kinect_;

    static const float fHFOV_primesense_109_;
    static const float fVFOV_primesense_109_;
    static const uint32_t nXRes_primesense_109_;
    static const uint32_t nYRes_primesense_109_;

    float nXRes_;
    float nYRes_;
    float fHFOV_;
    float fVFOV_;
    float xzFactor_;
    float yzFactor_;
    float halfResX_;
    float halfResY_;
    float coeffX_;
    float coeffY_;

    inline static double GetRealWorldXtoZKinect() { return m_fRealWorldXtoZ_kinect_; }
		inline static double GetRealWorldYtoZKinect() { return m_fRealWorldYtoZ_kinect_; }
  };
  
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_OPEN_NI_FUNCS_HEADER
