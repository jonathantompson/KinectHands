//
//  calibration_data.h
//
//  Container to store the intercepted Primesense calibration data
//

#ifndef KINECT_INTERFACE_CALIBRATION_DATA_HEADER
#define KINECT_INTERFACE_CALIBRATION_DATA_HEADER

#include "jtil/math/math_types.h"
#include "OpenNI.h"
#include "OniCAPI.h"
#include "DepthUtils.h"

#define MAX_Z 65535
		
namespace kinect_interface {
  
  struct CalibrationData {
  public:
    DepthUtilsSensorCalibrationInfo m_blob;
    uint16_t m_pRegistrationTable_QQVGA[160*120*2];
    uint16_t m_pRegistrationTable_QVGA[320*240*2];
    uint16_t m_pRegistrationTable_VGA[640*480*2];

    uint16_t m_pDepthToShiftTable_QQVGA[MAX_Z+1];
    uint16_t m_pDepthToShiftTable_QVGA[MAX_Z+1];
    uint16_t m_pDepthToShiftTable_VGA[MAX_Z+1];

    PadInfo* m_pPadInfo;
  };
  
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_CALIBRATION_DATA_HEADER