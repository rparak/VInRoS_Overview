#include <bur/plctypes.h>

#ifdef _DEFAULT_INCLUDES
#include <AsDefault.h>
#endif

#include "ABB_Library.h"

// Custom Lib.: Base conversion between data.
#include "Converter.h"

void Sim_PROFINET_Mapping_IO_Data(struct ABB_Library* inst, USINT offset);

typedef enum Robot_State_ID_enum{
	ROB_STATE_ACTIVE,
	ROB_STATE_POWER,
	ROB_STATE_WAIT,
	ROB_STATE_UPD_PARAMETERS_1,
	ROB_STATE_UPD_PARAMETERS_2,
	ROB_STATE_MOTION_1,
	ROB_STATE_MOTION_2,
	ROB_STATE_HOME_UPD_PARAMETERS_1,
	ROB_STATE_HOME_UPD_PARAMETERS_2,
	ROB_STATE_HOME_MOTION_1,
	ROB_STATE_HOME_MOTION_2,
	ROB_STATE_STOP,
	ROB_STATE_SAFETY,
	ROB_STATE_ERROR
}Robot_State_ID_enum;

_LOCAL struct ABB_Library ABB_Library_Rob_1;

void Set_Trajectory_Parameters(struct Traj_Str* inst){
	// 
	inst->Targets.Joint[0].Q[0] = 130.0; inst->Targets.Joint[0].Q[1] = -10.0;
	inst->Targets.Joint[0].Q[2] = 15.0; inst->Targets.Joint[0].Q[3] = -50.0;
	inst->Targets.Joint[0].Q[4] = 60.0; inst->Targets.Joint[0].Q[5] = 60.0;
	inst->Targets.Speed[0] = vSPEED_150;
	inst->Targets.Zone[0]  = zZone_fine;
	// 
	inst->Targets.Joint[1].Q[0] = 110.0; inst->Targets.Joint[1].Q[1] = 60.0;
	inst->Targets.Joint[1].Q[2] = -20.0; inst->Targets.Joint[1].Q[3] = 0.0;
	inst->Targets.Joint[1].Q[4] = -60.0; inst->Targets.Joint[1].Q[5] = -40.0;
	inst->Targets.Speed[1] = vSPEED_300;
	inst->Targets.Zone[1]  = zZone_fine;
	// 
	inst->Targets.Joint[2].Q[0] = -30.0; inst->Targets.Joint[2].Q[1] = 20.0;
	inst->Targets.Joint[2].Q[2] = 10.0; inst->Targets.Joint[2].Q[3] = 40.0;
	inst->Targets.Joint[2].Q[4] = 90.0; inst->Targets.Joint[2].Q[5] = 0.0;
	inst->Targets.Speed[2] = vSPEED_500;
	inst->Targets.Zone[2]  = zZone_fine;
	// 
	inst->Targets.Joint[3].Q[0] = 0.0; inst->Targets.Joint[3].Q[1] = -50.0;
	inst->Targets.Joint[3].Q[2] = -20.0; inst->Targets.Joint[3].Q[3] = 90.0;
	inst->Targets.Joint[3].Q[4] = -40.0; inst->Targets.Joint[3].Q[5] = 100.0;
	inst->Targets.Speed[3] = vSPEED_400;
	inst->Targets.Zone[3]  = zZone_fine;
	// 
	inst->Targets.Joint[4].Q[0] = 65.0; inst->Targets.Joint[4].Q[1] = 10.0;
	inst->Targets.Joint[4].Q[2] = 20.0; inst->Targets.Joint[4].Q[3] = 50.0;
	inst->Targets.Joint[4].Q[4] = 55.0; inst->Targets.Joint[4].Q[5] = -20.0;
	inst->Targets.Speed[4] = vSPEED_200;
	inst->Targets.Zone[4]  = zZone_fine;
	
	// 
	inst->Targets.Joint[5].Q[0] = 90.0; inst->Targets.Joint[5].Q[1] = 0.0;
	inst->Targets.Joint[5].Q[2] = 0.0; inst->Targets.Joint[5].Q[3] = 0.0;
	inst->Targets.Joint[5].Q[4] = 90.0; inst->Targets.Joint[5].Q[5] = 0.0;
	inst->Targets.Speed[5] = vSPEED_150;
	inst->Targets.Zone[5]  = zZone_fine;
	
	//
	inst->Length = 6;
}

void Sim_PROFINET_Mapping_IO_Data(struct ABB_Library* inst, USINT offset){
	// Mapping Input Data.
	inst->PROFINET_Mapping_IN.STATUS.INPUT = ROB_1_Sim_PROFINET_Mapping_IN[0 + offset];
	inst->PROFINET_Mapping_IN.SYSTEM.INPUT = ROB_1_Sim_PROFINET_Mapping_IN[1 + offset];
	inst->PROFINET_Mapping_IN.STATE_uID = ROB_1_Sim_PROFINET_Mapping_IN[2 + offset];
	inst->PROFINET_Mapping_IN.STATE_mID = ROB_1_Sim_PROFINET_Mapping_IN[3 + offset];
	inst->PROFINET_Mapping_IN.MOTION_TRAJECTORY_ID.INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[4 + offset];
	inst->PROFINET_Mapping_IN.MOTION_TRAJECTORY_ID.INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[5 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[0].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[6 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[0].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[7 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[1].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[8 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[1].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[9 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[2].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[10 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[2].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[11 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[3].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[12 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[3].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[13 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[4].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[14 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[4].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[15 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[5].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[16 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[5].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[17 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[6].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[18 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS[6].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[19 + offset];
	inst->PROFINET_Mapping_IN.JOINT_POS_SIGN.INPUT = ROB_1_Sim_PROFINET_Mapping_IN[20 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[0].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[21 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[0].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[22 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[0].INPUT[2] = ROB_1_Sim_PROFINET_Mapping_IN[23 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[0].INPUT[3] = ROB_1_Sim_PROFINET_Mapping_IN[24 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[1].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[25 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[1].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[26 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[1].INPUT[2] = ROB_1_Sim_PROFINET_Mapping_IN[27 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[1].INPUT[3] = ROB_1_Sim_PROFINET_Mapping_IN[28 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[2].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[29 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[2].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[30 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[2].INPUT[2] = ROB_1_Sim_PROFINET_Mapping_IN[31 + offset];
	inst->PROFINET_Mapping_IN.TCP_POS[2].INPUT[3] = ROB_1_Sim_PROFINET_Mapping_IN[32 + offset];
	inst->PROFINET_Mapping_IN.TCP_ROT[0].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[33 + offset];
	inst->PROFINET_Mapping_IN.TCP_ROT[0].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[34 + offset];
	inst->PROFINET_Mapping_IN.TCP_ROT[1].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[35 + offset];
	inst->PROFINET_Mapping_IN.TCP_ROT[1].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[36 + offset];
	inst->PROFINET_Mapping_IN.TCP_ROT[2].INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[37 + offset];
	inst->PROFINET_Mapping_IN.TCP_ROT[2].INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[38 + offset];
	inst->PROFINET_Mapping_IN.TCP_EX_POS.INPUT[0] = ROB_1_Sim_PROFINET_Mapping_IN[39 + offset];
	inst->PROFINET_Mapping_IN.TCP_EX_POS.INPUT[1] = ROB_1_Sim_PROFINET_Mapping_IN[40 + offset];
	inst->PROFINET_Mapping_IN.TCP_SIGN.INPUT = ROB_1_Sim_PROFINET_Mapping_IN[41 + offset];
	inst->PROFINET_Mapping_IN.TCP_CFG[0] = ROB_1_Sim_PROFINET_Mapping_IN[42 + offset];
	inst->PROFINET_Mapping_IN.TCP_CFG[1] = ROB_1_Sim_PROFINET_Mapping_IN[43 + offset];
	inst->PROFINET_Mapping_IN.TCP_CFG[2] = ROB_1_Sim_PROFINET_Mapping_IN[44 + offset];
	inst->PROFINET_Mapping_IN.TCP_CFG[3] = ROB_1_Sim_PROFINET_Mapping_IN[45 + offset];
	inst->PROFINET_Mapping_IN.TCP_CFG_SIGN.INPUT = ROB_1_Sim_PROFINET_Mapping_IN[46 + offset];

	// Mapping Output Data.
	ROB_1_Sim_PROFINET_Mapping_OUT[0 + offset] = inst->PROFINET_Mapping_OUT.STATUS.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[1 + offset] = inst->PROFINET_Mapping_OUT.SYSTEM.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[2 + offset] = inst->PROFINET_Mapping_OUT.COMMAND.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[3 + offset] = inst->PROFINET_Mapping_OUT.COMMAND_mID;
	ROB_1_Sim_PROFINET_Mapping_OUT[4 + offset] = inst->PROFINET_Mapping_OUT.COMMAND_uID;
	ROB_1_Sim_PROFINET_Mapping_OUT[5 + offset] = inst->PROFINET_Mapping_OUT.TRAJECTORY_SIZE.OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[6 + offset] = inst->PROFINET_Mapping_OUT.TRAJECTORY_SIZE.OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[7 + offset] = inst->PROFINET_Mapping_OUT.SPEED;
	ROB_1_Sim_PROFINET_Mapping_OUT[8 + offset] = inst->PROFINET_Mapping_OUT.ZONE;
	ROB_1_Sim_PROFINET_Mapping_OUT[9 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[10 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[11 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[12 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[13 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[14 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[15 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[3].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[16 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[3].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[17 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[4].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[18 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[4].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[19 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[5].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[20 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[5].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[21 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[6].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[22 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS[6].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[23 + offset] = inst->PROFINET_Mapping_OUT.JOINT_POS_SIGN.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[24 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[25 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[26 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[0].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[27 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[0].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[28 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[29 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[30 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[1].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[31 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[1].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[32 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[33 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[34 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[2].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[35 + offset] = inst->PROFINET_Mapping_OUT.TCP_POS[2].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[36 + offset] = inst->PROFINET_Mapping_OUT.TCP_ROT[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[37 + offset] = inst->PROFINET_Mapping_OUT.TCP_ROT[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[38 + offset] = inst->PROFINET_Mapping_OUT.TCP_ROT[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[39 + offset] = inst->PROFINET_Mapping_OUT.TCP_ROT[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[40 + offset] = inst->PROFINET_Mapping_OUT.TCP_ROT[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[41 + offset] = inst->PROFINET_Mapping_OUT.TCP_ROT[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[42 + offset] = inst->PROFINET_Mapping_OUT.TCP_EX_POS.OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[43 + offset] = inst->PROFINET_Mapping_OUT.TCP_EX_POS.OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[44 + offset] = inst->PROFINET_Mapping_OUT.TCP_SIGN.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[45 + offset] = inst->PROFINET_Mapping_OUT.TCP_CFG[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[46 + offset] = inst->PROFINET_Mapping_OUT.TCP_CFG[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[47 + offset] = inst->PROFINET_Mapping_OUT.TCP_CFG[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[48 + offset] = inst->PROFINET_Mapping_OUT.TCP_CFG[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[49 + offset] = inst->PROFINET_Mapping_OUT.TCP_CFG_SIGN.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[50 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_RH.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[51 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_POS[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[52 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_POS[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[53 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_POS[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[54 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_POS[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[55 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_POS[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[56 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_POS[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[57 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_POS_SIGN.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[58 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[59 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[60 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[0].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[61 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[0].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[62 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[63 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[64 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[1].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[65 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[1].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[66 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[67 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[68 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[2].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[69 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[2].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[70 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[3].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[71 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[3].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[72 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[3].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[73 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[3].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[74 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT_SIGN.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[75 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MASS.OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[76 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MASS.OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[77 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[78 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[79 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[0].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[80 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[0].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[81 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[82 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[83 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[1].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[84 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[1].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[85 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[86 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[87 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[2].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[88 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG[2].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[89 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_COG_SIGN.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[90 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[91 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[92 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[0].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[93 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[0].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[94 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[95 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[96 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[1].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[97 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[1].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[98 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[99 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[100 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[2].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[101 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[2].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[102 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[3].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[103 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[3].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[104 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[3].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[105 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[3].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[106 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT_SIGN.OUTPUT;
	ROB_1_Sim_PROFINET_Mapping_OUT[107 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[0].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[108 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[0].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[109 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[0].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[110 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[0].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[111 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[1].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[112 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[1].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[113 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[1].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[114 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[1].OUTPUT[3];
	ROB_1_Sim_PROFINET_Mapping_OUT[115 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[2].OUTPUT[0];
	ROB_1_Sim_PROFINET_Mapping_OUT[116 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[2].OUTPUT[1];
	ROB_1_Sim_PROFINET_Mapping_OUT[117 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[2].OUTPUT[2];
	ROB_1_Sim_PROFINET_Mapping_OUT[118 + offset] = inst->PROFINET_Mapping_OUT.RTOOL_MOI[2].OUTPUT[3];
}
