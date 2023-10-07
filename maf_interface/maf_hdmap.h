#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_hdmap {

struct image {
  maf_std::Header header;
  std::vector<uint8_t> PrepImageData;
  bool bIsValid;
  bool bExtractFeature;
  int64_t nReceivedMSGTimeFlags;
  int64_t nSendMSGTimeFlags;
  int64_t nImageTimeFlags;
  int64_t nFrameNo;
};

struct location {
  maf_std::Header header;
  bool bExtractFeature;
  double dHDMapPoseX;
  double dHDMapPoseY;
  double dHDMapPoseZ;
  double dHDMapPoseHeading;
  double dHDMapPosePitch;
  double dHDMapPoseRoll;
  double dHDMapVelocityX;
  double dHDMapVelocityY;
  double dHDMapVelocityZ;
  float fConfidence;
  uint8_t cStatus;
  std::vector<uint8_t> cReserved;
  int64_t nReceivedMSGTimeFlags;
  int64_t nSendMSGTimeFlags;
  int64_t nImageTimeFlags;
  int64_t nFrameNo;
};

struct gpfpd {
  maf_std::Header header;
  int32_t nGPSWeek;
  double dGPSTime;
  double dHeading;
  double dPitch;
  double dRoll;
  double dLattitude;
  double dLongitude;
  double dAltitude;
  double dX;
  double dY;
  double dVEast;
  double dVNorth;
  double dVUp;
  double dBaseline;
  int16_t nSatelitesNum1;
  int16_t nSatelitesNum2;
  int8_t cStatus;
  std::vector<int8_t> cReserved;
  int64_t nReceivedMSGTimeFlags;
  int64_t nSendMSGTimeFlags;
  int64_t nImageTimeFlags;
  int64_t nFrameNo;
};

struct gnss {
  maf_std::Header header;
  double dUtcTime;
  double dLatitude;
  double dLongitude;
  double dAltitude;
  double dHightOfgeo;
  double dHDOP;
  int8_t cStatus;
  int16_t nSateNum;
  double dSpeedOfGround;
  double dTrackDeg;
};

struct wheel {
  maf_std::Header header;
  float fFrontLeft;
  float fFrontRight;
  float fRearLeft;
  float fRearRight;
  float fScaleLeft;
  float fScaleRight;
  int64_t nReceivedMSGTimeFlags;
  int64_t nSendMSGTimeFlags;
  int64_t nImageTimeFlags;
  int64_t nFrameNo;
};

struct imu {
  maf_std::Header header;
  int32_t nGPSWeek;
  double dGPSTime;
  double dVX;
  double dVY;
  double dVZ;
  double dAX;
  double dAY;
  double dAZ;
  float fTemperature;
  std::vector<int8_t> cStatus;
  int64_t nReceivedMSGTimeFlags;
  int64_t nSendMSGTimeFlags;
  int64_t nImageTimeFlags;
  int64_t nFrameNo;
};

} // namespace maf_hdmap
