#pragma once
#include "maf_interface/maf_std.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace maf_mla_localization {

struct MLAStatusInfo {
  uint8_t quality;
  uint64_t common;
  uint64_t extended;
  uint8_t type;
  serdes4(quality, common, extended, type)
};

struct MLAReserved {
  uint8_t available;
  std::vector<uint64_t> reserved_data;

  enum : uint8_t {
    MLA_RESERVED_DATA = 1,
  };
  serdes2(available, reserved_data)
};

struct MLALocalizationMeta {
  uint64_t timestamp_us;
  uint64_t seq;
    serdes2(timestamp_us,seq)
};

struct MLAOriStdXyz {
  double std_faix;
  double std_faiy;
  double std_faiz;
  serdes3(std_faix, std_faiy, std_faiz)
};

struct MLAQuaternion {
  double w;
  double x;
  double y;
  double z;
  serdes4(w,x,y,z)
};

struct MLAEuler {
  double roll;
  double pitch;
  double yaw;
  serdes3(roll,pitch,yaw)
};

struct MLAAngVelStdLocal {
  double std_vx;
  double std_vy;
  double std_vz;
  serdes3(std_vx, std_vy, std_vz)
};

struct MLAAngVelLocal {
  double vx;
  double vy;
  double vz;
  serdes3(vx,vy,vz)
};

struct MLAVelGlobal {
  double ve;
  double vn;
  double vu;
  serdes3(ve,vn,vu)
};

struct MLAVelStdGlobal {
  double std_ve;
  double std_vn;
  double std_vu;
  serdes3(std_ve, std_vn, std_vu)
};

struct MLAVelStdLocal {
  double std_vx;
  double std_vy;
  double std_vz;
  serdes3(std_vx, std_vy, std_vz);
};

struct MLAVelLocal {
  double vx;
  double vy;
  double vz;
  serdes3(vx,vy,vz);
};

struct MLAAccGlobal {
  double ae;
  double an;
  double au;
  serdes3(ae,an,au);
};

struct MLAAccStdGlobal {
  double std_ae;
  double std_an;
  double std_au;
  serdes3(std_ae, std_an, std_au);
};

struct MLAAccStdLocal {
  double std_ax;
  double std_ay;
  double std_az;
  serdes3(std_ax, std_ay, std_az);
};

struct MLAAccLocal {
  double ax;
  double ay;
  double az;
  serdes3(ax,ay,az);
};

struct MLAPosStdLocal {
  double std_px;
  double std_py;
  double std_pz;
  serdes3(std_px, std_py, std_pz);
};

struct MLAPosGlobal {
  double latitude;
  double longitude;
  double altitude;
    serdes3(latitude,longitude,altitude)
};

struct MLAPosLocal {
  double x;
  double y;
  double z;
    serdes3(x,y,z)
};

struct MLAPosStdGlobal {
  double std_pe;
  double std_pn;
  double std_pu;
  serdes3(std_pe, std_pn, std_pu);
};

struct MLAPoseDetailInfo {
  uint8_t available;
  bool is_keyframe;
  bool has_scale;
  uint8_t pose_type;

  enum : uint8_t {
    MLA_ESTIMATION_UNKNOWN = 0,
    MLA_ESTIMATION_PER_FRAME = 1,
    MLA_ESTIMATION_WINDOW_OPT = 2,
    MLA_ESTIMATION_DR = 3,
  };
  serdes4(available, is_keyframe, has_scale, pose_type);
};

struct MLAStatus {
  uint8_t available;
  MLAStatusInfo status_info;

  enum : uint8_t {
    MLA_STATUS_INFO = 1,
  };
  serdes2(available, status_info);
};

struct MLATransformInfo {
  MLAQuaternion transform_q;
  MLAPosLocal transform_t;
  MLAPosGlobal transform_center;
  serdes3(transform_q, transform_t, transform_center);
};

struct MLAOrientation {
  uint8_t available;
  MLAEuler euler_global;
  MLAQuaternion quaternion_global;
  MLAEuler euler_local;
  MLAQuaternion quaternion_local;

  enum : uint8_t {
    MLA_EULER_GLOBAL = 1,
    MLA_QUATERNION_GLOBAL = 2,
    MLA_EULER_LOCAL = 4,
    MLA_QUATERNION_LOCAL = 8,
  };
    serdes5(available, euler_global, quaternion_global, euler_local, quaternion_local);
};

struct MLAOrientationStd {
  uint8_t available;
  MLAOriStdXyz ori_std_xyz;

  enum : uint8_t {
    MLA_ORI_STD_XYZ = 1,
  };
  serdes2(available, ori_std_xyz);
};

struct MLAAngularVelocity {
  uint8_t available;
  MLAAngVelLocal angvelocity_local;

  enum : uint8_t {
    MLA_ANGVEL_LOCAL = 1,
  };
  serdes2(available,angvelocity_local)
};

struct MLAAngularVelocityStd {
  uint8_t available;
  MLAAngVelStdLocal angvel_std_local;

  enum : uint8_t {
    MLA_ANGVEL_STD_LOCAL = 1,
  };
  serdes2(available, angvel_std_local);
};

struct MLAVelocityStd {
  uint8_t available;
  MLAVelStdGlobal vel_std_global;
  MLAVelStdLocal vel_std_local;

  enum : uint8_t {
    MLA_VEL_STD_GLOBAL = 1,
    MLA_VEL_STD_LOCAL = 2,
  };
  serdes3(available, vel_std_global, vel_std_local);
};

struct MLAVelocity {
  uint8_t available;
  MLAVelGlobal velocity_global;
  MLAVelLocal velocity_local;

  enum : uint8_t {
    MLA_VEL_GLOBAL = 1,
    MLA_VEL_LOCAL = 2,
  };
  serdes3(available, velocity_global, velocity_local)
};

struct MLAAccelerationStd {
  uint8_t available;
  MLAAccStdGlobal acc_std_global;
  MLAAccStdLocal acc_std_local;

  enum : uint8_t {
    MLA_ACC_STD_GLOBAL = 1,
    MLA_ACC_STD_LOCAL = 2,
  };
  serdes3(available, acc_std_global, acc_std_local);
};

struct MLAAcceleration {
  uint8_t available;
  MLAAccGlobal acceleration_global;
  MLAAccLocal acceleration_local;

  enum : uint8_t {
    MLA_ACC_GLOBAL = 1,
    MLA_ACC_LOCAL = 2,
  };
  serdes3(available, acceleration_global, acceleration_local);
};

struct MLAPosition {
  uint8_t available;
  MLAPosGlobal position_global;
  MLAPosLocal position_local;

  enum : uint8_t {
    MLA_POSITION_GLOBAL = 1,
    MLA_POSITION_LOCAL = 2,
  };
    serdes3(available,position_global,position_local)
};

struct MLAPositionStd {
  uint8_t available;
  MLAPosStdGlobal pos_std_global;
  MLAPosStdLocal pos_std_local;

  enum : uint8_t {
    MLA_POS_STD_GLOBAL = 1,
    MLA_POS_STD_LOCAL = 2,
  };
  serdes3(available, pos_std_global, pos_std_local);
};

struct MLAPoseDetail {
  uint8_t available;
  MLAPoseDetailInfo pose_detail_info;

  enum : uint8_t {
    MLA_POSE_DETAIL_INFO = 1,
  };
  serdes2(available, pose_detail_info);
};

struct MLATransform {
  uint8_t available;
  MLATransformInfo transform_llh_to_boot;
  MLATransformInfo transform_avp_map_to_boot;
  MLATransformInfo transform_ego_motion_to_boot;

  enum : uint8_t {
    MLA_TRANSFORM_LLH_TO_BOOT = 1,
    MLA_TRANSFORM_AVPMAP_TO_BOOT = 2,
    MLA_TRANSFORM_EGOMOTION_TO_BOOT = 4,
  };
  serdes4(available, transform_llh_to_boot, transform_avp_map_to_boot, transform_ego_motion_to_boot);
};

struct MLALocalization {
  maf_std::Header header;
  MLALocalizationMeta meta;
  MLAPosition position;
  MLAVelocity velocity;
  MLAAngularVelocity angular_velocity;
  MLAOrientation orientation;
  MLAAcceleration acceleration;
  MLAPositionStd position_std;
  MLAVelocityStd velocity_std;
  MLAAngularVelocityStd angular_velocity_std;
  MLAOrientationStd orientation_std;
  MLAAccelerationStd acceleration_std;
  MLAStatus status;
  MLATransform transform;
  MLAPoseDetail pose_detail;
  MLAReserved reserved;
    serdes16(header,meta,position,velocity, angular_velocity,
             orientation,acceleration,position_std,velocity_std,angular_velocity_std,
             orientation_std, acceleration_std, status, transform, pose_detail, 
             reserved);
};

} // namespace maf_mla_localization
