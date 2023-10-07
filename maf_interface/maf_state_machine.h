#pragma once

#include "maf_interface/maf_std.h"

namespace maf_state_machine{
    struct ADAS_Info 
    {
        uint8_t ComAdapter_CllsnMtgtnFctSts_u8;
        uint8_t ComAdapter_CllsnMtgtnWarnSts_u8;
        uint8_t ComAdapter_CllsnMtgtnOnoffSts_u8;
        uint8_t ComAdapter_CllsnFwdWarn_u8;
        uint8_t ComAdapter_CllsnThreat_u8;
        uint8_t ComAdapter_BrkHptcWarnReqForAutDrvBrkHptcWarnReqForAutDrv_u8;
        uint8_t ComAdapter_CllsnAidPost_u8;
        float ComAdapter_SftyDecelGroupSafeAsySftyDecelReq_f32;
        uint8_t ComAdapter_SftyDecelGroupSafeAsySftyEnaDecel_u8;
        uint8_t ComAdapter_AsySftyEnaDecelByDBSAsySftyEnaDecelByDBS_u8;
        uint8_t ComAdapter_BrkPrecActv_u8;
        uint8_t ComAdapter_AsySftyStandStillReq_u8;
        uint8_t ComAdapter_AsySftyHWLReq_u8;
        uint8_t ComAdapter_CllsnWarnReOn1_u8;
        uint8_t ComAdapter_CllsnWarnReIndcn_u8;
        uint8_t ComAdapter_RcwmBrkReqRcwmBrkReq_u8;
        uint8_t ComAdapter_AsyLaneKeepAidSts_u8;
        uint8_t ComAdapter_AsyEmgyLaneKeepAidSts_u8;
        uint8_t ComAdapter_AsyLatCtrlModReqGroupAsyLatCtrlModReq_u8;
        float ComAdapter_AsyPinionAgReqSafeAsyPinionAgReq_f32;
        uint8_t ComAdapter_WarnForAsyEmgyLaneKeepAid_u8;
        uint8_t ComAdapter_AsyLineLeColor_u8;
        uint8_t ComAdapter_AsyLineRiColor_u8;
        float ComAdapter_AgCtrlTqLowrLim_f32;
        float ComAdapter_AgCtrlTqUpprLim_f32;
        uint8_t ComAdapter_AsySteerApplyRqrd_u8;
        float ComAdapter_SteerTqAddlForEmaSteerTqAddlForEma_f32;
        uint8_t ComAdapter_CllsnAidPostEMA_u8;
        uint8_t ComAdapter_CllsnMtgtnFaultSts_u8;
        uint8_t ComAdapter_FctaIndcnLe_u8;
        uint8_t ComAdapter_FctaIndcnRi_u8;
        uint8_t ComAdapter_CtraOn1_u8;
        uint8_t ComAdapter_RctaIndcnLe_u8;
        uint8_t ComAdapter_RctalndcnRi_u8;
        uint8_t ComAdapter_RctaBrkReqRctaBrkReq_u8;
        uint8_t ComAdapter_DoorOpenwarnOn1_u8;
        uint8_t ComAdapter_DoorOpenwarnLeIndcn_u8;
        uint8_t ComAdapter_DoorOpenwarnRiIndcn_u8;
        uint8_t ComAdapter_AsyALgtIndcr_u8;
        uint8_t ComAdapter_CnclWarnLgtForAutDrv_u8;
        uint8_t ComAdapter_AsyALgtStsAsyALgtSts_u8;
        float ComAdapter_DispSpdSetForLgtCtrl_f32;
        float ComAdapter_AsyALgtReqForCmftAsyCmftLgtNegLimForJerk_f32;
        float ComAdapter_AsyALgtReqForCmftAsyCmftLgtPosLimForJerk_f32;
        float ComAdapter_AsyALgtReqForCmftMax_f32;
        float ComAdapter_AsyALgtReqForCmftMin_f32;
        uint8_t ComAdapter_AsyLgtCtrlTakeOverReq_u8;
        uint8_t ComAdapter_AccFusnTrfcSgn_u8;
        uint8_t ComAdapter_SpdLimFirst_u8;
        uint8_t ComAdapter_DispTSIInfoForLgtCtrl_u8;
        uint8_t ComAdapter_TiGapSetForLgtCtrl_u8;
        uint8_t ComAdapter_LcmaOn_u8;
        uint8_t ComAdapter_LcmaAudWarn_u8;
        uint8_t ComAdapter_LcmaIndcnRi_u8;
        uint8_t ComAdapter_LcmaIndcnLe_u8;
        uint8_t ComAdapter_SpdLimCamFirst_u8;
        uint8_t ComAdapter_SpdLimWarnReq_u8;
        uint8_t ComAdapter_CtryForRoadSgnInfo_u8;
        uint8_t ComAdapter_SpdLimUnit_u8;
        uint8_t ComAdapter_SpdLimSec_u8;
        uint8_t ComAdapter_TrfcInfoMiscFirst_u8;
        uint8_t ComAdapter_RoadSgnInfoSts_u8;
        uint8_t ComAdapter_RoadMiscSgnInfoSts_u8;
        uint8_t ComAdapter_SpdAlrmActvSts_u8;
        uint8_t ComAdapter_OffsForSpdWarnSetgSts_u8;
        uint8_t ComAdapter_DrvOffWarn_u8;
        uint8_t ComAdapter_AsyStandStillReqForCmft_u8;
        uint8_t ComAdapter_DrvOffReqForLgtCtrl_u8;
        float ComAdapter_ObjFrnt1DstLgtOfObjFrnt1_f32;
        float ComAdapter_ObjFrnt1DstLatOfObjFrnt1_f32;
        uint8_t ComAdapter_ObjFrnt1TypOfObjFrnt1_u8;
        uint8_t ComAdapter_ObjFrnt1HdDirOfObjFrnt1_u8;
        uint8_t ComAdapter_ObjFrnt1ColorOfObjFrnt1_u8;
        uint8_t ComAdapter_AsySteerWhlHptcWarnReq_u8;
        uint8_t ComAdapter_AsyALatIndcr_u8;
        uint8_t ComAdapter_AsyAutDrvgAvl_u8;
        uint8_t ComAdapter_LaneChgAutActvSts_u8;
        uint8_t ComAdapter_TurnIndicReqByALCA_u8;
        uint8_t ComAdapter_LaneChgAutInfoForAsyHiWay_u8;
        uint8_t ComAdapter_LaneChgAutStsForAsyHiWay_u8;
        uint8_t ComAdapter_AsyObjType_u8;
        uint8_t ComAdapter_RcwmLiReq_u8;
        uint8_t ComAdapter_AsyObjForBigData0Type_u8;
        float ComAdapter_AsyObjForBigData0Confidence_f32;
        float ComAdapter_AsyObjForBigData0TTC_f32;
        float ComAdapter_AsyObjForBigData0RlvtLatDst_f32;
        float ComAdapter_AsyObjForBigData0RlvtLgtDst_f32;
        float ComAdapter_AsyObjForBigData0AbsLatSpd_f32;
        float ComAdapter_AsyObjForBigData0AbsLgtSpd_f32;
        float ComAdapter_AsyObjForBigData0SnsrSrcOfAbsLatSpd_f32;
        float ComAdapter_AsyObjForBigData0SnsrSrcOfAbsLgtSpd_f32;
        uint8_t ComAdapter_AsyLaneChgWarnMod_u8;
        uint8_t ComAdapter_IntvAndWarnModInfoSts_u8;
        uint8_t ComAdapter_LaneKeepAidInfoSts_u8;
        uint8_t ComAdapter_AsyWarnForSteerCncl_u8;
        uint8_t ComAdapter_LaneChgWarnSts_u8;
        uint8_t ComAdapter_AsyEmgyLaneKeepAid_u8;
        uint8_t ComAdapter_AvlStsForLongAutDrv_u8;
        uint8_t ComAdapter_AsyAutDrvCtrlTyp_u8;
        uint8_t ComAdapter_AsyEmgyManoeuvreAidSts_u8;
        float ComAdapter_FrntVehInfFrntVehDis_f32;
        float ComAdapter_FrntVehInfFrntVehSpd_f32;
        uint8_t ComAdapter_FrntVehInfQf_u8;
        uint8_t ComAdapter_SpdLimCoupldFirst_u8;
        uint8_t ComAdapter_SpdLimSpplFirst_u8;
        uint8_t ComAdapter_SpdLimWarnReqAud_u8;
        uint8_t ComAdapter_RiTTCLv_u8;
        uint8_t ComAdapter_LeTTCLv_u8;
    };

    struct StateMachine_Hmi_Struct
    {
        uint8_t statemachine_switch_available;
        uint8_t statemachine_e2e_hardactive_switch;
        uint8_t statemachine_e2e_hardcancel_switch;
        uint8_t statemachine_e2e_hardresum_switch;
        uint8_t statemachine_e2e_main_switch_status;
        uint8_t statemachine_e2e_soft_active_switch_status;
        uint8_t statemachine_e2e_active;
        uint8_t statemachine_e2e_quit;
        uint8_t statemachine_avpRemindSwitch;
        uint8_t statemachine_road_condition_available;
        uint8_t statemachine_road_condition;
        uint16_t statemachine_distance_to_special_area;
        uint16_t statemachine_distance_out_special_area;
        uint8_t statemachine_odd_warning_available;
        uint16_t statemachine_distance_to_odd_area;
        uint8_t statemachine_odd_remind;
        uint8_t statemachine_odd_warning;
        uint8_t statemachine_warning_level;
        uint8_t statemachine_punishment_mode;
        uint16_t statemachine_distance_out_handfree_odd;
        uint16_t statemachine_distance_out_e2e_odd;
        uint8_t statemachine_e2e_exit_available;
        uint8_t statemachine_exit_active;
        uint8_t statemachine_activation_failed;
        uint8_t statemachine_AD4HmiLimitedSts;
        uint8_t statemachine_Seat_belt_not_fastened;
        uint8_t statemachine_door_open;
        uint8_t statemachine_front_hatch_cover_open;
        uint8_t statemachine_charging_port_not_closed;
        uint8_t statemachine_gear_lever_in_manual_mode;
        uint8_t statemachine_TCS_active;
        uint8_t statemachine_VDC_active;
        uint8_t statemachine_HDC_active;
        uint8_t statemachine_ABS_active;
        uint8_t statemachine_VDC_has_been_closed;
        uint8_t statemachine_AVH_actived;
        uint8_t statemachine_tow_coupler_open;
        uint8_t statemachine_E2E_main_switch_not_open;
        uint8_t statemachine_E2E_switch_on_navigation_not_open;
        uint8_t statemachine_not_in_ODD_range;
        uint8_t statemachine_gear_position_not_in_D;
        uint8_t statemachine_brake_pedal_pressed;
        uint8_t statemachine_steering_wheel_torque_too_large;
        uint8_t statemachine_vehicle_speed_not_in_range;
        uint8_t statemachine_EPB_actived;
        uint8_t statemachine_acceleration_pedal_pressed;
        uint8_t statemachine_navigation_off;
        uint8_t statemachine_driver_hand_off;
        uint8_t statemachine_punishment_mode_active;
        uint8_t statemachine_planning_failed;
        uint8_t statemachine_control_failed;
        uint8_t statemachine_eps_control_failed;
        uint8_t statemachine_esc_control_failed;
        uint8_t statemachine_vcu_control_failed;
        uint8_t statemachine_perception_error;
        uint8_t statemachine_mirr_fold;
        uint8_t statemachine_crash_actived;
        uint8_t statemachine_tire_pressure_warning;
        uint8_t statemachine_others_reason;
        uint8_t statemachine_speed_mode;
        uint8_t statemachine_speed_offset;
        uint8_t statemachine_speed_set;
        uint8_t statemachine_speed_limit;
        uint8_t statemachine_speed_limit_state;
        uint8_t statemachine_AD4DrvrBeltWarn;
        float statemachine_ego_pitch;
        float statemachine_ego_yaw;
        float statemachine_ego_roll;
        uint8_t statemachine_obj1_obj_id;
        uint8_t statemachine_obj1_following;
        uint8_t statemachine_obj1_confidence;
        uint8_t statemachine_obj1_speed;
        uint8_t statemachine_obj1_obj_type;
        uint8_t statemachine_obj1_predict_status;
        uint8_t statemachine_obj1_colour;
        float statemachine_obj1_pos_x;
        float statemachine_obj1_pos_y;
        float statemachine_obj1_pos_angle;
        float statemachine_obj1_pos_z;
        uint8_t statemachine_obj1_break_light;
        uint8_t statemachine_obj1_turn_light;
        float statemachine_obj1_obj_length;
        float statemachine_obj1_obj_width;
        float statemachine_obj1_obj_height;
        uint8_t statemachine_obj2_obj_id;
        uint8_t statemachine_obj2_following;
        uint8_t statemachine_obj2_confidence;
        uint8_t statemachine_obj2_speed;
        uint8_t statemachine_obj2_obj_type;
        uint8_t statemachine_obj2_predict_status;
        uint8_t statemachine_obj2_colour;
        float statemachine_obj2_pos_x;
        float statemachine_obj2_pos_y;
        float statemachine_obj2_pos_angle;
        float statemachine_obj2_pos_z;
        uint8_t statemachine_obj2_break_light;
        uint8_t statemachine_obj2_turn_light;
        float statemachine_obj2_obj_length;
        float statemachine_obj2_obj_width;
        float statemachine_obj2_obj_height;
        uint8_t statemachine_obj3_obj_id;
        uint8_t statemachine_obj3_following;
        uint8_t statemachine_obj3_confidence;
        uint8_t statemachine_obj3_speed;
        uint8_t statemachine_obj3_obj_type;
        uint8_t statemachine_obj3_predict_status;
        uint8_t statemachine_obj3_colour;
        float statemachine_obj3_pos_x;
        float statemachine_obj3_pos_y;
        float statemachine_obj3_pos_angle;
        float statemachine_obj3_pos_z;
        uint8_t statemachine_obj3_break_light;
        uint8_t statemachine_obj3_turn_light;
        float statemachine_obj3_obj_length;
        float statemachine_obj3_obj_width;
        float statemachine_obj3_obj_height;
        uint8_t statemachine_obj4_obj_id;
        uint8_t statemachine_obj4_following;
        uint8_t statemachine_obj4_confidence;
        uint8_t statemachine_obj4_speed;
        uint8_t statemachine_obj4_obj_type;
        uint8_t statemachine_obj4_predict_status;
        uint8_t statemachine_obj4_colour;
        float statemachine_obj4_pos_x;
        float statemachine_obj4_pos_y;
        float statemachine_obj4_pos_angle;
        float statemachine_obj4_pos_z;
        uint8_t statemachine_obj4_break_light;
        uint8_t statemachine_obj4_turn_light;
        float statemachine_obj4_obj_length;
        float statemachine_obj4_obj_width;
        float statemachine_obj4_obj_height;
        uint8_t statemachine_obj5_obj_id;
        uint8_t statemachine_obj5_following;
        uint8_t statemachine_obj5_confidence;
        uint8_t statemachine_obj5_speed;
        uint8_t statemachine_obj5_obj_type;
        uint8_t statemachine_obj5_predict_status;
        uint8_t statemachine_obj5_colour;
        float statemachine_obj5_pos_x;
        float statemachine_obj5_pos_y;
        float statemachine_obj5_pos_angle;
        float statemachine_obj5_pos_z;
        uint8_t statemachine_obj5_break_light;
        uint8_t statemachine_obj5_turn_light;
        float statemachine_obj5_obj_length;
        float statemachine_obj5_obj_width;
        float statemachine_obj5_obj_height;
        uint8_t statemachine_obj6_obj_id;
        uint8_t statemachine_obj6_following;
        uint8_t statemachine_obj6_confidence;
        uint8_t statemachine_obj6_speed;
        uint8_t statemachine_obj6_obj_type;
        uint8_t statemachine_obj6_predict_status;
        uint8_t statemachine_obj6_colour;
        float statemachine_obj6_pos_x;
        float statemachine_obj6_pos_y;
        float statemachine_obj6_pos_angle;
        float statemachine_obj6_pos_z;
        uint8_t statemachine_obj6_break_light;
        uint8_t statemachine_obj6_turn_light;
        float statemachine_obj6_obj_length;
        float statemachine_obj6_obj_width;
        float statemachine_obj6_obj_height;
        uint8_t statemachine_obj7_obj_id;
        uint8_t statemachine_obj7_following;
        uint8_t statemachine_obj7_confidence;
        uint8_t statemachine_obj7_speed;
        uint8_t statemachine_obj7_obj_type;
        uint8_t statemachine_obj7_predict_status;
        uint8_t statemachine_obj7_colour;
        float statemachine_obj7_pos_x;
        float statemachine_obj7_pos_y;
        float statemachine_obj7_pos_angle;
        float statemachine_obj7_pos_z;
        uint8_t statemachine_obj7_break_light;
        uint8_t statemachine_obj7_turn_light;
        float statemachine_obj7_obj_length;
        float statemachine_obj7_obj_width;
        float statemachine_obj7_obj_height;
        uint8_t statemachine_obj8_obj_id;
        uint8_t statemachine_obj8_following;
        uint8_t statemachine_obj8_confidence;
        uint8_t statemachine_obj8_speed;
        uint8_t statemachine_obj8_obj_type;
        uint8_t statemachine_obj8_predict_status;
        uint8_t statemachine_obj8_colour;
        float statemachine_obj8_pos_x;
        float statemachine_obj8_pos_y;
        float statemachine_obj8_pos_angle;
        float statemachine_obj8_pos_z;
        uint8_t statemachine_obj8_break_light;
        uint8_t statemachine_obj8_turn_light;
        float statemachine_obj8_obj_length;
        float statemachine_obj8_obj_width;
        float statemachine_obj8_obj_height;
        uint8_t statemachine_obj9_obj_id;
        uint8_t statemachine_obj9_following;
        uint8_t statemachine_obj9_confidence;
        uint8_t statemachine_obj9_speed;
        uint8_t statemachine_obj9_obj_type;
        uint8_t statemachine_obj9_predict_status;
        uint8_t statemachine_obj9_colour;
        float statemachine_obj9_pos_x;
        float statemachine_obj9_pos_y;
        float statemachine_obj9_pos_angle;
        float statemachine_obj9_pos_z;
        uint8_t statemachine_obj9_break_light;
        uint8_t statemachine_obj9_turn_light;
        float statemachine_obj9_obj_length;
        float statemachine_obj9_obj_width;
        float statemachine_obj9_obj_height;
        uint8_t statemachine_left_line_confidence;
        uint8_t statemachine_left_line_AsyLineLeTyp;
        uint8_t statemachine_left_line_AsyLineLeColor;
        float statemachine_left_line_c0;
        float statemachine_left_line_c1;
        float statemachine_left_line_c2;
        float statemachine_left_line_c3;
        float statemachine_left_line_start;
        float statemachine_left_line_end;
        float statemachine_left_line_second_c0;
        float statemachine_left_line_second_c1;
        float statemachine_left_line_second_c2;
        float statemachine_left_line_second_c3;
        float statemachine_left_line_second_start;
        float statemachine_left_line_second_end;
        uint8_t statemachine_left_left_line_confidence;
        uint8_t statemachine_left_left_line_AsyLineLeTyp;
        uint8_t statemachine_left_left_line_AsyLineLeColor;
        float statemachine_left_left_line_c0;
        float statemachine_left_left_line_c1;
        float statemachine_left_left_line_c2;
        float statemachine_left_left_line_c3;
        float statemachine_left_left_line_start;
        float statemachine_left_left_line_end;
        float statemachine_left_left_line_second_c0;
        float statemachine_left_left_line_second_c1;
        float statemachine_left_left_line_second_c2;
        float statemachine_left_left_line_second_c3;
        float statemachine_left_left_line_second_start;
        float statemachine_left_left_line_second_end;
        uint8_t statemachine_right_right_line_confidence;
        uint8_t statemachine_right_right_line_AsyLineLeTyp;
        uint8_t statemachine_right_right_line_AsyLineLeColor;
        float statemachine_right_right_line_c0;
        float statemachine_right_right_line_c1;
        float statemachine_right_right_line_c2;
        float statemachine_right_right_line_c3;
        float statemachine_right_right_line_start;
        float statemachine_right_right_line_end;
        float statemachine_right_right_line_second_c0;
        float statemachine_right_right_line_second_c1;
        float statemachine_right_right_line_second_c2;
        float statemachine_right_right_line_second_c3;
        float statemachine_right_right_line_second_start;
        float statemachine_right_right_line_second_end;
        uint8_t statemachine_right_line_confidence;
        uint8_t statemachine_right_line_AsyLineLeTyp;
        uint8_t statemachine_right_line_AsyLineLeColor;
        float statemachine_right_line_c0;
        float statemachine_right_line_c1;
        float statemachine_right_line_c2;
        float statemachine_right_line_c3;
        float statemachine_right_line_start;
        float statemachine_right_line_end;
        float statemachine_right_line_second_c0;
        float statemachine_right_line_second_c1;
        float statemachine_right_line_second_c2;
        float statemachine_right_line_second_c3;
        float statemachine_right_line_second_start;
        float statemachine_right_line_second_end;
        uint8_t statemachine_construction1_id;
        uint8_t statemachine_construction1_confidence;
        uint8_t statemachine_construction1_obj_type;
        uint8_t statemachine_construction1_colour;
        float statemachine_construction1_pos_x;
        float statemachine_construction1_pos_y;
        float statemachine_construction1_pos_z;
        uint8_t statemachine_construction2_id;
        uint8_t statemachine_construction2_confidence;
        uint8_t statemachine_construction2_obj_type;
        uint8_t statemachine_construction2_colour;
        float statemachine_construction2_pos_x;
        float statemachine_construction2_pos_y;
        float statemachine_construction2_pos_z;
        uint8_t statemachine_construction3_id;
        uint8_t statemachine_construction3_confidence;
        uint8_t statemachine_construction3_obj_type;
        uint8_t statemachine_construction3_colour;
        float statemachine_construction3_pos_x;
        float statemachine_construction3_pos_y;
        float statemachine_construction3_pos_z;
        uint8_t statemachine_construction4_id;
        uint8_t statemachine_construction4_confidence;
        uint8_t statemachine_construction4_obj_type;
        uint8_t statemachine_construction4_colour;
        float statemachine_construction4_pos_x;
        float statemachine_construction4_pos_y;
        float statemachine_construction4_pos_z;
        uint8_t statemachine_construction5_id;
        uint8_t statemachine_construction5_confidence;
        uint8_t statemachine_construction5_obj_type;
        uint8_t statemachine_construction5_colour;
        float statemachine_construction5_pos_x;
        float statemachine_construction5_pos_y;
        float statemachine_construction5_pos_z;
        uint8_t statemachine_construction6_id;
        uint8_t statemachine_construction6_confidence;
        uint8_t statemachine_construction6_obj_type;
        uint8_t statemachine_construction6_colour;
        float statemachine_construction6_pos_x;
        float statemachine_construction6_pos_y;
        float statemachine_construction6_pos_z;
        uint8_t statemachine_stopline_id;
        uint8_t statemachine_stopline_type;
        uint8_t statemachine_stopline_confidence;
        uint8_t statemachine_stopline_colour;
        float statemachine_stopline_pos_x;
        float statemachine_stopline_pos_y;
        float statemachine_stopline_pos_angle;
        float statemachine_stopline_pos_z;
        float statemachine_stopline_obj_length;
        float statemachine_stopline_obj_width;
        float statemachine_stopline_obj_height;
        uint8_t statemachine_ego_HmiAsyVehiclePahDisp;
        uint8_t statemachine_ego_HmiAsyVehicleColor;
        uint8_t statemachine_ego_SelfLaneColor;
        float statemachine_ego_c0;
        float statemachine_ego_c1;
        float statemachine_ego_c2;
        float statemachine_ego_c3;
        float statemachine_ego_end;
        uint8_t statemachine_ego_enable;
        uint8_t statemachine_lan_change_available;
        uint8_t statemachine_lan_change_mode;
        uint8_t statemachine_overtaking_lane_set;
        uint8_t statemachine_driver_lane_change_req;
        uint8_t statemachine_driver_lane_change_suppression;
        uint8_t statemachine_LaneChgAutStsForAsyHiWay;
        uint8_t statemachine_lan_change_direction;
        uint8_t statemachine_lan_change_request;
        uint8_t statemachine_remind_warning;
        uint8_t statemachine_remind_req;
        uint8_t statemachine_ADGeneralMsgGroup;
        uint8_t statemachine_ADIntrLiCtrlReq;
        uint8_t statemachine_takeover_available;
        uint8_t statemachine_takeover_warning;
        uint8_t statemachine_takeover_req;
        uint8_t statemachine_safety_stop_status;
        uint8_t statemachine_enable;
        uint8_t statemachine_progress_percentage;
        uint16_t statemachine_reserve_distance;
        uint8_t statemachine_frog_lamp_req;
        uint8_t statemachine_AD4FrtLampReq;
        uint8_t statemachine_internal_light_req;
        uint8_t statemachine_AD4TurnIndicReqReq;
        uint8_t statemachine_AsySftyHWLReq;
    };

    struct StateMachine_State_Struct 
    {
        uint64_t statemachine_timestamp_us;
        uint8_t statemachine_st_available;
        uint8_t statemachine_e2e_status;
        uint8_t statemachine_e2e_status_his;
        uint64_t statemachine_status_dur_time;
        uint64_t statemachine_status_change_res;
        uint8_t statemachine_emerg_func_active;
        uint8_t statemachine_emerg_func_type;
        uint8_t statemachine_takovr_level;
        uint64_t statemachine_takovr_res;
        uint64_t statemachine_takovr_dur_time;
        uint8_t statemachine_takovr_confidence;
        uint8_t statemachine_punish_st;
        uint64_t statemachine_punish_res;
        uint64_t statemachine_punish_time;
        uint8_t statemachine_e2e_supp;
        uint64_t statemachine_e2e_supp_res;
        uint8_t statemachine_warning_timeout;
 uint64_t statemachine_error;
 std::vector<uint8_t>statemachine_reserved_int;
 std::vector<float>statemachine_reserved_double;
    };
}
