#pragma once

#include <stdint.h>

#include "maf_interface/maf_geometry.h"
#include "maf_interface/maf_std.h"

namespace maf_remote_pilot {

struct RemotePilotControl_Struct {
  uint8_t RP_VehWinSunRoofClsReq_u8;
  uint8_t RP_AD4AsyStandStillReqStandStillReq_u8;
  uint8_t RP_AD4WhlLockReqWhlLockReq_u8;
  uint8_t RP_AD4DrvOffReqForLgtCtrlDrvOffReq_u8;
  uint8_t RP_AsyADL3FuncCtrlStsADMod_u8;
  uint8_t RP_AsyADL3FuncCtrlStsCtrlSts_u8;
  uint8_t RP_AsyADL3FuncCtrlStsDegraded_u8;
  uint8_t RP_AsyADL3FuncCtrlStsQf_u8;
  uint8_t RP_AsyADL3FuncCtrlStsSts_u8;
  uint8_t RP_AsyADModeReqADActiveReq_u8;
  uint8_t RP_AsyADModeReqADDeactiveReq_u8;
  float RP_AsyALgtReqRngForSafeMax_f32;
  float RP_AsyALgtReqRngForSafeMin_f32;
  uint8_t RP_AccrOvrdnAllwdForAutDrvYesNo1_u8;
  float RP_AsyPinionAgReqSafeAsyPinionAgReq_f32;
  uint8_t RP_AD4RWSCtrlActReqReq_u8;
  uint8_t RP_AsySteerWhlHptcWarnReq_u8;
  uint8_t RP_BrkHptcWarnReqForAutDrvBrkHptcWarnReqForAutDrv_u8;
  uint8_t RP_GearPrkgAssiReqGroupGearPrkgAssiReq1_u8;
  uint8_t RP_AD4FrtFogLampReq_u8;
  uint8_t RP_AD4FrtLampReq_u8;
  uint8_t RP_AD4HornWarn_u8;
  uint8_t RP_AD4RearFogLampReq_u8;
  uint8_t RP_AD4TrunkCtrlReq_u8;
  uint8_t RP_ADIntrLiCtrlReq_u8;
  uint8_t RP_ADWiprCtrlReq_u8;
  uint8_t RP_AsySftyHWLReq_u8;
  uint8_t RP_MirrOpenClsReq_u8;
  uint8_t RP_PrkgFctVMMModReq_u8;
  uint8_t RP_PrkgUsgDrvModTranReq_u8;
  uint8_t RP_VehBeamLoModReq1_u8;
  uint8_t RP_VehPrkgLockTheftReq_u8;
  uint8_t RP_AD4TurnIndicReqReq_u8;
  uint8_t RP_AD4DrvrBeltWarn_u8;
  uint8_t RP_AD4WinOpenReq_u8;
};  // struct RemotePilotControl

struct RemotePilot_RP2StateMachine_Struct {
  uint8_t RPStatus;
  bool rpActiveSwitchPressed;
  bool rpCancelSwitchPressed;
  bool rpMainSwitchOn;
};  // struct RemotePilot_RP2StateMachine_Struct

struct RemotePilot_RP2Broker_Struct {
  bool RenderedObjInfoUploadTrigger;
};

}  // namespace maf_remote_pilot
