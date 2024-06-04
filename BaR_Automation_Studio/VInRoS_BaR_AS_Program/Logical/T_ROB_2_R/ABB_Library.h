#ifndef HEADER_FILE_ABB_LIBRARY
#define HEADER_FILE_ABB_LIBRARY

// Include B&R Automation libraries (declarations for B&R ANSI C extensions).
#include <bur/plctypes.h>

// Custom Lib.: Base conversion between data.
#include "Converter.h"

// Include other libraries.
#include <math.h>

// Directive (macro definition).
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define DegToRad(value) ((value) * M_PI / 180.0)
#define RadToDeg(value) ((value) * 180.0 / M_PI)

#define TRUE 1
#define FALSE (!TRUE)
#define NULL 0
#define CEIL_DEFUALT_FACTOR 10

_LOCAL UINT aux_trajectory_ID;

typedef enum ABB_Library_statesPLC_enum{
	ABBt_STATE_INITIALIZATION     = 0,
	ABBt_STATE_START_INIT         = 5,
	ABBt_STATE_START_MOTOR_ON     = 6,
	ABBt_STATE_START_PP_TO_M      = 7,
	ABBt_STATE_START_EXECUTE      = 8,
	ABBt_STATE_STOP_INIT          = 10,
	ABBt_STATE_STOP_EXECUTE       = 11,
	ABBt_STATE_STOP_MOTOR_OFF     = 12,
	ABBt_STATE_STOPPED_PP_TO_MAIN = 13,
	ABBt_STATE_STOPPED_TO_START   = 14,
	ABBt_STATE_WAIT               = 20,
	ABBt_STATE_MOVE_JOINT_CHECK   = 30,
	ABBt_STATE_MOVE_JOINT_EXECUTE = 31,
	ABBt_STATE_MOVE_TCP_CHECK     = 40,
	ABBt_STATE_MOVE_TCP_EXECUTE   = 41
}ABB_Library_statesPLC_enum;

typedef enum ABB_Library_statesROB_m_enum{
	ABBr_STATE_INITIALIZATION = 10,
	ABBr_STATE_WAIT           = 20,
	ABBr_STATE_MOVE_JOINT     = 30,
	ABBr_STATE_MOVE_TCP       = 40
}ABB_Library_statesROB_m_enum;

typedef enum ABB_Library_statesROB_u_enum{
	ABBu_STATE_INITIALIZATION      = 10, 
	ABBu_STATE_UPDATE_JOINT_TRAJ_1 = 40,
	ABBu_STATE_UPDATE_JOINT_TRAJ_2 = 41,
	ABBu_STATE_UPDATE_JOINT_TRAJ_3 = 42,
	ABBu_STATE_UPDATE_JOINT_TRAJ_4 = 43,
	ABBu_STATE_UPDATE_TCP_TRAJ_1   = 50,
	ABBu_STATE_UPDATE_TCP_TRAJ_2   = 51,
	ABBu_STATE_UPDATE_TCP_TRAJ_3   = 52,
	ABBu_STATE_UPDATE_TCP_TRAJ_4   = 53,
	ABBu_STATE_UPDATE_TOOL_PARAM   = 100
}ABB_Library_statesROB_u_enum;

typedef enum ABB_Library_cmd_m_state_enum{
	MAIN_ID_EMPTY		   = 0,
	MAIN_ID_JOINT_ABSOLUTE = 1,
	MAIN_ID_TCP_LINEAR     = 2,
}ABB_Library_cmd_m_state_enum;

typedef enum ABB_Library_cmd_u_state_enum{
	UPT_ID_EMPTY			= 0,
	UPT_ID_TRAJECTORY_JOINT = 1,
	UPT_ID_TRAJECTORY_TCP   = 2,
	UPT_ID_ROB_TOOL	        = 10
}ABB_Library_cmd_u_state_enum;

typedef enum ABB_Library_update_enum{
	UPT_STATE_INITIALIZATION = 0,
	UPT_STATE_WAIT           = 10,
	UPT_STATE_JOINT_TRAJ_1   = 40,
	UPT_STATE_JOINT_TRAJ_2   = 41,
	UPT_STATE_JOINT_TRAJ_3   = 42,
	UPT_STATE_JOINT_TRAJ_4   = 43,
	UPT_STATE_TCP_TRAJ_1     = 50,
	UPT_STATE_TCP_TRAJ_2     = 51,
	UPT_STATE_TCP_TRAJ_3     = 52,
	UPT_STATE_TCP_TRAJ_4     = 53,
	UPT_STATE_RTOOL          = 100
}ABB_Library_update_enum;

typedef enum ABB_Library_mParam_speed_enum{
	vSPEED_5,
	vSPEED_10, 
	vSPEED_20, 
	vSPEED_30, 
	vSPEED_40, 
	vSPEED_50, 
	vSPEED_60, 
	vSPEED_80,
	vSPEED_100,
	vSPEED_150, 
	vSPEED_200, 
	vSPEED_300, 
	vSPEED_400, 
	vSPEED_500, 
	vSPEED_600, 
	vSPEED_800,
	vSPEED_1000,
	vSPEED_1500,
	vSPEED_2000,
	vSPEED_2500,
	vSPEED_3000,
	vSPEED_4000,
	vSPEED_5000,
	vSPEED_6000
}ABB_Library_mParam_speed_enum;

typedef enum ABB_Library_mParam_zone_enum{
	zZone_fine,
	zZone_0,
	zZone_1,
	zZone_5,
	zZone_10,
	zZone_15,
	zZone_20,
	zZone_30,
	zZone_40,
	zZone_50,
	zZone_60,
	zZone_80,
	zZone_100,
	zZone_150,
	zZone_200
}ABB_Library_mParam_zone_enum;

typedef struct ABB_Library_internal_update_str{
	UINT ID;
	ABB_Library_update_enum actual_state;
}ABB_Library_internal_update_str;

typedef struct ABB_Library_internal_str{
	ABB_Library_statesPLC_enum actual_state;
	ABB_Library_statesPLC_enum before_state;
	ABB_Library_internal_update_str Update;
	REAL ACCURACY_FACTOR;
	REAL ACCURACY_FACTOR_QUATERNION;
}ABB_Library_internal_str;
	
typedef struct ABB_Library_profinet_in_str{
	struct Convert_USINT_To_BOOL_Array STATUS;
	USINT STATE_mID;
	struct Convert_USINT_To_BOOL_Array SYSTEM;
	USINT STATE_uID;
	struct Convert_USINT_Array_To_UINT MOTION_TRAJECTORY_ID;
	struct Convert_USINT_Array_To_UINT JOINT_POS[7];
	struct Convert_USINT_To_BOOL_Array JOINT_POS_SIGN;
	struct Convert_USINT_Array_To_UDINT TCP_POS[3];
	struct Convert_USINT_Array_To_UINT TCP_ROT[3];
	struct Convert_USINT_Array_To_UINT TCP_EX_POS;
	struct Convert_USINT_To_BOOL_Array TCP_SIGN;
	USINT TCP_CFG[4];
	struct Convert_USINT_To_BOOL_Array TCP_CFG_SIGN;
}ABB_Library_profinet_in_str;

typedef struct ABB_Library_profinet_out_str{
	struct Convert_BOOL_Array_To_USINT STATUS;
	struct Convert_BOOL_Array_To_USINT SYSTEM;
	struct Convert_BOOL_Array_To_USINT COMMAND;
	USINT COMMAND_mID;
	USINT COMMAND_uID;
	struct Convert_UINT_To_USINT_Array TRAJECTORY_SIZE;
	USINT SPEED;
	USINT ZONE;
	struct Convert_UINT_To_USINT_Array JOINT_POS[7];
	struct Convert_BOOL_Array_To_USINT JOINT_POS_SIGN;
	struct Convert_UDINT_To_USINT_Array TCP_POS[3];
	struct Convert_UINT_To_USINT_Array TCP_ROT[3];
	struct Convert_UINT_To_USINT_Array TCP_EX_POS;
	struct Convert_BOOL_Array_To_USINT TCP_SIGN;
	USINT TCP_CFG[4];
	struct Convert_BOOL_Array_To_USINT TCP_CFG_SIGN;
	struct Convert_BOOL_Array_To_USINT RTOOL_RH;
	struct Convert_UINT_To_USINT_Array RTOOL_POS[3];
	struct Convert_BOOL_Array_To_USINT RTOOL_POS_SIGN;
	struct Convert_UDINT_To_USINT_Array RTOOL_ROT_QUAT[4];
	struct Convert_BOOL_Array_To_USINT RTOOL_ROT_QUAT_SIGN;
	struct Convert_UINT_To_USINT_Array RTOOL_MASS;
	struct Convert_UDINT_To_USINT_Array RTOOL_COG[3];
	struct Convert_BOOL_Array_To_USINT RTOOL_COG_SIGN;
	struct Convert_UDINT_To_USINT_Array RTOOL_AOM_QUAT[4];
	struct Convert_BOOL_Array_To_USINT RTOOL_AOM_QUAT_SIGN;
	struct Convert_UDINT_To_USINT_Array RTOOL_MOI[3];
}ABB_Library_profinet_out_str;	

typedef struct ABB_Library_sysOUT_str{
	BOOL MOTOR_ON;
	BOOL MOTOR_OFF;
	BOOL PP_TO_MAIN;
	BOOL START;
	BOOL STOP;
}ABB_Library_sysOUT_str;

typedef struct ABB_Library_id_cmd_str{
	ABB_Library_cmd_m_state_enum Motion;
	ABB_Library_cmd_u_state_enum Update;
}ABB_Library_id_cmd_str;

typedef struct ABB_Library_command_str{
	BOOL START;
	BOOL STOP;
	BOOL UPDATE;
	ABB_Library_id_cmd_str ID;
	ABB_Library_sysOUT_str System;
}ABB_Library_command_str;

typedef struct ABB_Library_TCP_str{
	REAL Position[3];
	REAL Rotation[3];
	SINT Configuration[4];
	REAL External_Position;
}ABB_Library_TCP_str;

typedef struct ABB_Library_profinet_rtData_str{
	REAL Q[7];
	ABB_Library_TCP_str TCP;
}ABB_Library_profinet_rtData_str;

typedef struct ABB_Library_profinet_Jw_str{
	REAL Q[7];
}ABB_Library_profinet_Jw_str;

typedef struct ABB_Library_parameter_str{
	ABB_Library_profinet_Jw_str Joint[100];
	ABB_Library_TCP_str TCP[100];
	ABB_Library_mParam_speed_enum Speed[100];
	ABB_Library_mParam_zone_enum Zone[100];
	UINT Trajectory_Size;
}ABB_Library_parameter_str;

typedef struct ABB_Library_rTool_tf_str{
	// Translation part.
	REAL Position[3];
	// Rotation part (Quaternion). 
	REAL Rotation[4];
}ABB_Library_rTool_tf_str;

typedef struct ABB_Library_rTool_tl_str{
	// Tool Mass.
	REAL mass;
	// Center of Gravity.
	REAL center_of_gravity[3];
	// Inertial axes of tool load.
	REAL intertial_axes[4];
	// Momets of inertia.
	REAL moment_of_inertia[3];
}ABB_Library_rTool_tl_str;

typedef struct ABB_Library_robTool_str{
	// The robot is holding the tool (TRUE / FALSE).
	BOOL robHold;	
	// Tool Frame.			 
	ABB_Library_rTool_tf_str tFrame;
	// Tool Load.
	ABB_Library_rTool_tl_str tLoad;
}ABB_Library_robTool_str;

typedef struct ABB_Library_statusPLC_str{
	BOOL Active;
	BOOL Module_OK;
	BOOL Update_Done;
}ABB_Library_statusPLC_str;

typedef struct ABB_Library_sysIN_str{
	BOOL MOTOR_ON;
	BOOL MOTOR_OFF;
	BOOL PP_MOVED;
	BOOL CYCLE_ON;
}ABB_Library_sysIN_str;

typedef struct ABB_Library_id_statesROB_str{
	ABB_Library_statesROB_m_enum Motion;
	ABB_Library_statesROB_u_enum Update;
}ABB_Library_id_statesROB_str;

typedef struct ABB_Library_statusROB_str{
	BOOL Active;
	BOOL In_Position;
	ABB_Library_id_statesROB_str ID;
	BOOL Update_Done;
	ABB_Library_sysIN_str System;
	UINT Trajectory_ID;
}ABB_Library_statusROB_str;

typedef struct ABB_Library_status_str{
	ABB_Library_statusPLC_str PLC;
	ABB_Library_statusROB_str Robot;
}ABB_Library_status_str;

typedef struct ABB_Library
{
	/**
	Description:
		Function block ...
	*/
	
	/**
 	 * FUNCTION BLOCK: ABB_Library
	 * INPUT VARIABLES
 	 */
	BOOL Enable;
	BOOL Power_ON;
	BOOL Power_OFF;
	ABB_Library_command_str Command;
	ABB_Library_parameter_str Parameter;
	ABB_Library_robTool_str Rob_Tool;
	ABB_Library_profinet_in_str PROFINET_Mapping_IN;
	
	/**
 	 * FUNCTION BLOCK: ABB_Library
	 * OUTPUT VARIABLES
 	 */
	ABB_Library_status_str Status;
	ABB_Library_profinet_out_str PROFINET_Mapping_OUT;
	ABB_Library_profinet_rtData_str RT_Data;
	
	/**
 	 * FUNCTION BLOCK: ABB_Library
	 * INTERNAL VARIABLES
 	 */
	ABB_Library_internal_str Internal;
	
} ABB_Library_typ;

/**
 * Declaration of the function block ....
 */
_BUR_PUBLIC void ABB_Library(struct ABB_Library* inst);
void Template_Reset(struct ABB_Library* inst);
void Update_Parameters(struct ABB_Library* inst);
void Read_Data_ROB_to_PLC(struct ABB_Library* inst);
void Write_Data_PLC_to_ROB(struct ABB_Library* inst);

void ABB_Library(struct ABB_Library* inst){
	/**
	Description:
		Function block ...
	
	Args:
		(1) inst [struct ABB_Library*]: Function block instance.
	*/
	
	if(inst->Enable != TRUE){
		Template_Reset(inst);
	}else{		
		Read_Data_ROB_to_PLC(inst);
	}
	
	switch(inst->Internal.actual_state){
		case ABBt_STATE_INITIALIZATION:
			{
				inst->Internal.ACCURACY_FACTOR 			    = 100;
				inst->Internal.ACCURACY_FACTOR_QUATERNION   = 100000000;
				
				if(inst->Status.PLC.Active == TRUE){
					inst->Internal.actual_state = ABBt_STATE_START_INIT;
				}
			}
			break;
		
		case ABBt_STATE_START_INIT:
			{
				inst->Power_OFF = FALSE;
				
				memset(&inst->Command.System, NULL, sizeof(inst->Command.System));
				
				if(inst->Power_ON == TRUE && inst->Status.Robot.System.MOTOR_ON == FALSE){
					inst->Internal.actual_state = ABBt_STATE_START_MOTOR_ON;
				}else if(inst->Power_ON == TRUE && inst->Status.Robot.System.MOTOR_ON == TRUE){
					inst->Internal.actual_state = ABBt_STATE_START_PP_TO_M;
				}
			}
			break;
		
		case ABBt_STATE_START_MOTOR_ON:
			{
				inst->Command.System.MOTOR_ON = TRUE;
				
				if(inst->Status.Robot.System.MOTOR_ON == TRUE){
					inst->Command.System.MOTOR_ON = FALSE;
					inst->Internal.actual_state   = ABBt_STATE_START_PP_TO_M;
				}
			}
			break;
		
		case ABBt_STATE_START_PP_TO_M:
			{
				inst->Command.System.PP_TO_MAIN = TRUE;
				
				if(inst->Status.Robot.System.PP_MOVED == TRUE){
					inst->Command.System.PP_TO_MAIN = FALSE;
					inst->Internal.actual_state     = ABBt_STATE_START_EXECUTE;
				}
			}
			break;
		
		case ABBt_STATE_START_EXECUTE:
			{
				inst->Command.System.START = TRUE;
				
				if(inst->Status.Robot.System.CYCLE_ON == TRUE && inst->Status.Robot.Active == TRUE){
					inst->Command.System.START  = FALSE;
					inst->Internal.actual_state = ABBt_STATE_WAIT;
				}
			}
			break;
		
		case ABBt_STATE_STOP_EXECUTE:
			{
				inst->Command.System.STOP = TRUE;
				
				if(inst->Status.Robot.System.CYCLE_ON == FALSE){
					inst->Command.System.STOP   = FALSE;
					if(inst->Power_OFF == TRUE){
						inst->Power_OFF             = FALSE;
						inst->Internal.actual_state = ABBt_STATE_STOP_MOTOR_OFF;
					}else if(inst->Command.STOP == TRUE){
						inst->Command.STOP = FALSE;
						inst->Internal.actual_state = ABBt_STATE_STOPPED_PP_TO_MAIN;
					}
				}	
			}
			break;
		
		case ABBt_STATE_STOP_MOTOR_OFF:
			{
				inst->Command.System.MOTOR_OFF = TRUE;
				
				if(inst->Status.Robot.System.MOTOR_OFF == TRUE){
					inst->Command.System.MOTOR_OFF = FALSE;
					inst->Internal.actual_state    = ABBt_STATE_START_INIT;
				}
			}
			break;
		
		case ABBt_STATE_STOPPED_PP_TO_MAIN:
			{
				inst->Command.System.PP_TO_MAIN = TRUE;
				
				if(inst->Status.Robot.System.PP_MOVED == TRUE){
					inst->Command.System.PP_TO_MAIN = FALSE;
					inst->Internal.actual_state     = ABBt_STATE_STOPPED_TO_START;
				}
			}
			break;
		
		case ABBt_STATE_STOPPED_TO_START:
			{
				inst->Command.System.START = TRUE;
				
				if(inst->Status.Robot.System.CYCLE_ON == TRUE && inst->Status.Robot.Active == TRUE){
					inst->Command.System.START  = FALSE;
					inst->Internal.actual_state = ABBt_STATE_WAIT;
				}
			}
			break;
		
		case ABBt_STATE_WAIT:
			{
				inst->Power_ON = FALSE;
				
				if(inst->Command.START == TRUE && inst->Command.ID.Motion == MAIN_ID_JOINT_ABSOLUTE){
					inst->Command.STOP          = FALSE;
					inst->Internal.actual_state = ABBt_STATE_MOVE_JOINT_CHECK;
				}else if(inst->Command.START == TRUE && inst->Command.ID.Motion == MAIN_ID_TCP_LINEAR){
					inst->Command.STOP          = FALSE;
					inst->Internal.actual_state = ABBt_STATE_MOVE_TCP_CHECK;
				}
				
				if(inst->Power_OFF == TRUE){
					inst->Internal.actual_state = ABBt_STATE_STOP_EXECUTE;
				}
			}
			break;
	
		case ABBt_STATE_MOVE_JOINT_CHECK:
			{
				inst->Internal.before_state = ABBt_STATE_MOVE_JOINT_CHECK;
						
				if(inst->Status.Robot.ID.Motion == ABBr_STATE_MOVE_JOINT && inst->Status.Robot.In_Position == FALSE){
					inst->Command.START     = FALSE;
					inst->Command.ID.Motion = MAIN_ID_EMPTY;
					
					inst->Internal.actual_state = ABBt_STATE_MOVE_JOINT_EXECUTE;
				}
			}
			break;

		case ABBt_STATE_MOVE_JOINT_EXECUTE:
			{
				inst->Internal.before_state = ABBt_STATE_MOVE_JOINT_EXECUTE;
						
				if(inst->Status.Robot.In_Position == TRUE){
					inst->Command.START     = FALSE;
					inst->Command.ID.Motion = MAIN_ID_EMPTY;
					
					inst->Internal.actual_state = ABBt_STATE_WAIT;
				}
			}
			break;
		
		case ABBt_STATE_MOVE_TCP_CHECK:
			{
				inst->Internal.before_state = ABBt_STATE_MOVE_TCP_CHECK;
						
				if(inst->Status.Robot.ID.Motion == ABBr_STATE_MOVE_TCP && inst->Status.Robot.In_Position == FALSE){
					inst->Command.START     = FALSE;
					inst->Command.ID.Motion = MAIN_ID_EMPTY;
					
					inst->Internal.actual_state = ABBt_STATE_MOVE_TCP_EXECUTE;
				}
			}
			break;

		case ABBt_STATE_MOVE_TCP_EXECUTE:
			{
				inst->Internal.before_state = ABBt_STATE_MOVE_TCP_EXECUTE;
						
				if(inst->Status.Robot.In_Position == TRUE){
					inst->Command.START     = FALSE;
					inst->Command.ID.Motion = MAIN_ID_EMPTY;
					
					inst->Internal.actual_state = ABBt_STATE_WAIT;
				}
			}
			break;
	}
	
	if(inst->Internal.actual_state == ABBt_STATE_MOVE_JOINT_CHECK || inst->Internal.actual_state == ABBt_STATE_MOVE_JOINT_EXECUTE ||
	   inst->Internal.actual_state == ABBt_STATE_MOVE_TCP_CHECK || inst->Internal.actual_state == ABBt_STATE_MOVE_TCP_EXECUTE){
		if(inst->Command.STOP == TRUE){
			inst->Internal.actual_state = ABBt_STATE_STOP_EXECUTE;
		}
	}
	
	if(inst->Status.PLC.Module_OK == TRUE){
		inst->Status.PLC.Active = TRUE;
		
		Write_Data_PLC_to_ROB(inst);
	}else{
		inst->Status.PLC.Active = FALSE;
	}
	
	Update_Parameters(inst);
}

void Template_Reset(struct ABB_Library* inst){
	/**
	Description:
		Function to initialize and reset parameters of the template.
	
	Args:
		(1) inst [struct ABB_Library*]: Function block instance.
	*/
	
	/**
	 * INPUT VARIABLES
 	 */
	inst->Power_ON = inst->Power_OFF = FALSE;
	memset(&inst->Command, NULL, sizeof(inst->Command));
	memset(&inst->Parameter, NULL, sizeof(inst->Parameter));
	memset(&inst->Rob_Tool, NULL, sizeof(inst->Rob_Tool));
	
	/**
	 * OUTPUT VARIABLES
 	 */
	memset(&inst->Status, NULL, sizeof(inst->Status));
	memset(&inst->RT_Data, NULL, sizeof(inst->RT_Data));
	
	/**
	 * INTERNAL VARIABLES
 	 */
	inst->Internal.actual_state = inst->Internal.before_state = ABBt_STATE_INITIALIZATION;
	inst->Internal.Update.actual_state = UPT_STATE_INITIALIZATION;
}

void Update_Parameters(struct ABB_Library* inst){
	switch(inst->Internal.Update.actual_state){
		case UPT_STATE_INITIALIZATION:
			{
				if(inst->Status.Robot.Active == TRUE){
					inst->Internal.Update.actual_state = UPT_STATE_WAIT;
				}
			}
			break;
				
		case UPT_STATE_WAIT:
			{
				if(inst->Command.UPDATE == TRUE && inst->Command.ID.Update == UPT_ID_TRAJECTORY_JOINT){
					inst->Internal.Update.actual_state = UPT_STATE_JOINT_TRAJ_1;
				}else if(inst->Command.UPDATE == TRUE && inst->Command.ID.Update == UPT_ID_TRAJECTORY_TCP){
					inst->Internal.Update.actual_state = UPT_STATE_TCP_TRAJ_1;
				}else if(inst->Command.UPDATE == TRUE && inst->Command.ID.Update  == UPT_ID_ROB_TOOL){
					inst->Internal.Update.actual_state = UPT_STATE_RTOOL;
				} 
			}
			break;
		
		case UPT_STATE_JOINT_TRAJ_1:
			{
				if(inst->Parameter.Trajectory_Size == inst->Internal.Update.ID){
					if(inst->Status.Robot.ID.Update == ABBu_STATE_INITIALIZATION){
						inst->Internal.Update.ID = NULL;
						aux_trajectory_ID                   = NULL;
						inst->Internal.Update.actual_state  = UPT_STATE_WAIT;
					}
				}else{
					if(inst->Status.Robot.ID.Update == ABBu_STATE_UPDATE_JOINT_TRAJ_3){
						aux_trajectory_ID = inst->Internal.Update.ID;
						
						inst->Command.UPDATE                = FALSE;
						inst->Command.ID.Update             = UPT_ID_EMPTY;
						inst->Internal.Update.actual_state  = UPT_STATE_JOINT_TRAJ_2;
					}
				}	
			}
			break;
		
		case UPT_STATE_JOINT_TRAJ_2:
			{
				if(inst->Internal.Update.ID == aux_trajectory_ID){
					inst->Internal.Update.ID = aux_trajectory_ID + 1;
				}
				
				inst->Internal.Update.actual_state  = UPT_STATE_JOINT_TRAJ_3;
			}
			break;
		
		case UPT_STATE_JOINT_TRAJ_3:
			{
				inst->PROFINET_Mapping_OUT.SPEED = (unsigned char)inst->Parameter.Speed[inst->Internal.Update.ID - 1];
				inst->PROFINET_Mapping_OUT.ZONE  = (unsigned char)inst->Parameter.Zone[inst->Internal.Update.ID - 1];
					
				unsigned char i_j;
				for(i_j = 0; i_j < (unsigned char)(sizeof(inst->Parameter.Joint[0].Q)/sizeof(inst->Parameter.Joint[0].Q[0])); i_j++){
					if(inst->Parameter.Joint[inst->Internal.Update.ID - 1].Q[i_j] >= 0.0){
						inst->PROFINET_Mapping_OUT.JOINT_POS_SIGN.INPUT[i_j] = TRUE;
						inst->PROFINET_Mapping_OUT.JOINT_POS[i_j].INPUT      = (UINT)(ceil((inst->Parameter.Joint[inst->Internal.Update.ID - 1].Q[i_j] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					}else{
						inst->PROFINET_Mapping_OUT.JOINT_POS_SIGN.INPUT[i_j] = FALSE;
						inst->PROFINET_Mapping_OUT.JOINT_POS[i_j].INPUT      = (-1) * ((UINT)(ceil((inst->Parameter.Joint[inst->Internal.Update.ID - 1].Q[i_j] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
					}
					
					Convert_UINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.JOINT_POS[i_j]);
				}
				
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.JOINT_POS_SIGN);
				
				inst->Status.PLC.Update_Done = TRUE;
				
				if(inst->Status.Robot.ID.Update == ABBu_STATE_UPDATE_JOINT_TRAJ_4){
					inst->Internal.Update.actual_state = UPT_STATE_JOINT_TRAJ_4;
				}
			}
			break;
		
		case UPT_STATE_JOINT_TRAJ_4:
			{
				inst->Status.PLC.Update_Done = FALSE;
				
				if(inst->Status.Robot.Update_Done == TRUE){
					inst->Internal.Update.actual_state = UPT_STATE_JOINT_TRAJ_1;
				}
			}
			break;
		
		case UPT_STATE_TCP_TRAJ_1:
			{
				if(inst->Parameter.Trajectory_Size == inst->Internal.Update.ID){
					if(inst->Status.Robot.ID.Update == ABBu_STATE_INITIALIZATION){
						inst->Internal.Update.ID = NULL;
						aux_trajectory_ID                   = NULL;
						inst->Internal.Update.actual_state  = UPT_STATE_WAIT;
					}
				}else{
					if(inst->Status.Robot.ID.Update == ABBu_STATE_UPDATE_TCP_TRAJ_3){
						aux_trajectory_ID = inst->Internal.Update.ID;
						
						inst->Command.UPDATE                = FALSE;
						inst->Command.ID.Update             = UPT_ID_EMPTY;
						inst->Internal.Update.actual_state  = UPT_STATE_TCP_TRAJ_2;
					}
				}	
			}
			break;
		
		case UPT_STATE_TCP_TRAJ_2:
			{
				if(inst->Internal.Update.ID == aux_trajectory_ID){
					inst->Internal.Update.ID = aux_trajectory_ID + 1;
				}
				
				inst->Internal.Update.actual_state  = UPT_STATE_TCP_TRAJ_3;
			}
			break;
		
		case UPT_STATE_TCP_TRAJ_3:
			{
				inst->PROFINET_Mapping_OUT.SPEED = (unsigned char)inst->Parameter.Speed[inst->Internal.Update.ID - 1];
				inst->PROFINET_Mapping_OUT.ZONE  = (unsigned char)inst->Parameter.Zone[inst->Internal.Update.ID - 1];
					
				unsigned char i_tcp;
				for(i_tcp = 0; i_tcp < (unsigned char)(sizeof(inst->Parameter.TCP[0].Position)/sizeof(inst->Parameter.TCP[0].Position[0])); i_tcp++){
					if(inst->Parameter.TCP[inst->Internal.Update.ID - 1].Position[i_tcp] >= 0.0){
						inst->PROFINET_Mapping_OUT.TCP_SIGN.INPUT[i_tcp] = TRUE;
						inst->PROFINET_Mapping_OUT.TCP_POS[i_tcp].INPUT  = (UDINT)(ceil((inst->Parameter.TCP[inst->Internal.Update.ID - 1].Position[i_tcp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					}else{
						inst->PROFINET_Mapping_OUT.TCP_SIGN.INPUT[i_tcp] = FALSE;
						inst->PROFINET_Mapping_OUT.TCP_POS[i_tcp].INPUT  = (-1) * ((UDINT)(ceil((inst->Parameter.TCP[inst->Internal.Update.ID - 1].Position[i_tcp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
					}
					Convert_UDINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.TCP_POS[i_tcp]);
					
					if(inst->Parameter.TCP[inst->Internal.Update.ID - 1].Rotation[i_tcp] >= 0.0){
						inst->PROFINET_Mapping_OUT.TCP_SIGN.INPUT[3 + i_tcp] = TRUE;
						inst->PROFINET_Mapping_OUT.TCP_ROT[i_tcp].INPUT  = (UDINT)(ceil((inst->Parameter.TCP[inst->Internal.Update.ID - 1].Rotation[i_tcp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					}else{
						inst->PROFINET_Mapping_OUT.TCP_SIGN.INPUT[3 + i_tcp] = FALSE;
						inst->PROFINET_Mapping_OUT.TCP_ROT[i_tcp].INPUT  = (-1) * ((UDINT)(ceil((inst->Parameter.TCP[inst->Internal.Update.ID - 1].Rotation[i_tcp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
					}
					Convert_UINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.TCP_ROT[i_tcp]);
				}
				
				if(inst->Parameter.TCP[inst->Internal.Update.ID - 1].External_Position >= 0.0){
					inst->PROFINET_Mapping_OUT.TCP_SIGN.INPUT[6] = TRUE;
					inst->PROFINET_Mapping_OUT.TCP_EX_POS.INPUT  = (UDINT)(ceil((inst->Parameter.TCP[inst->Internal.Update.ID - 1].External_Position * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
				}else{
					inst->PROFINET_Mapping_OUT.TCP_SIGN.INPUT[6] = FALSE;
					inst->PROFINET_Mapping_OUT.TCP_EX_POS.INPUT  = (-1) * ((UDINT)(ceil((inst->Parameter.TCP[inst->Internal.Update.ID - 1].External_Position * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
				}
				
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.TCP_SIGN);
				
				unsigned char i_cfg;
				for(i_cfg = 0; i_cfg < (unsigned char)(sizeof(inst->Parameter.TCP[0].Configuration)/sizeof(inst->Parameter.TCP[0].Configuration[0])); i_cfg++){
					if(inst->Parameter.TCP[inst->Internal.Update.ID - 1].Configuration[i_cfg] >= 0){
						inst->PROFINET_Mapping_OUT.TCP_CFG_SIGN.INPUT[i_cfg] = TRUE;
						inst->PROFINET_Mapping_OUT.TCP_CFG[i_cfg]            = inst->Parameter.TCP[inst->Internal.Update.ID - 1].Configuration[i_cfg];
					}else{
						inst->PROFINET_Mapping_OUT.TCP_CFG_SIGN.INPUT[i_cfg] = FALSE;
						inst->PROFINET_Mapping_OUT.TCP_CFG[i_cfg]            = (-1) * inst->Parameter.TCP[inst->Internal.Update.ID - 1].Configuration[i_cfg];
					}
				}
				
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.TCP_CFG_SIGN);
				
				inst->Status.PLC.Update_Done = TRUE;
				
				if(inst->Status.Robot.ID.Update == ABBu_STATE_UPDATE_TCP_TRAJ_4){
					inst->Internal.Update.actual_state = UPT_STATE_TCP_TRAJ_4;
				}
			}
			break;
		
		case UPT_STATE_TCP_TRAJ_4:
			{
				inst->Status.PLC.Update_Done = FALSE;
				
				if(inst->Status.Robot.Update_Done == TRUE){
					inst->Internal.Update.actual_state = UPT_STATE_TCP_TRAJ_1;
				}
			}
			break;
		
		case UPT_STATE_RTOOL:
			{
				inst->PROFINET_Mapping_OUT.RTOOL_RH.INPUT[0] = inst->Rob_Tool.robHold;
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.RTOOL_RH);
				
				inst->PROFINET_Mapping_OUT.RTOOL_MASS.INPUT = (UINT)(ceil((inst->Rob_Tool.tLoad.mass * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
				Convert_UINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.RTOOL_MASS);
				
				unsigned char i_tp;
				for(i_tp = 0; i_tp < (unsigned char)(sizeof(inst->Rob_Tool.tFrame.Position)/sizeof(inst->Rob_Tool.tFrame.Position[0])); i_tp++){
					if(inst->Rob_Tool.tFrame.Position[i_tp] >= 0.0){
						inst->PROFINET_Mapping_OUT.RTOOL_POS_SIGN.INPUT[i_tp] = TRUE;
						inst->PROFINET_Mapping_OUT.RTOOL_POS[i_tp].INPUT      = (UDINT)(ceil((inst->Rob_Tool.tFrame.Position[i_tp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					}else{
						inst->PROFINET_Mapping_OUT.RTOOL_POS_SIGN.INPUT[i_tp] = FALSE;
						inst->PROFINET_Mapping_OUT.RTOOL_POS[i_tp].INPUT      = (-1) * ((UDINT)(ceil((inst->Rob_Tool.tFrame.Position[i_tp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
					}
					
					Convert_UINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.RTOOL_POS[i_tp]);
					
					if(inst->Rob_Tool.tLoad.center_of_gravity[i_tp] >= 0.0){
						inst->PROFINET_Mapping_OUT.RTOOL_COG_SIGN.INPUT[i_tp] = TRUE;
						inst->PROFINET_Mapping_OUT.RTOOL_COG[i_tp].INPUT      = (UDINT)(ceil((inst->Rob_Tool.tLoad.center_of_gravity[i_tp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					}else{
						inst->PROFINET_Mapping_OUT.RTOOL_COG_SIGN.INPUT[i_tp] = FALSE;
						inst->PROFINET_Mapping_OUT.RTOOL_COG[i_tp].INPUT      = (-1) * ((UDINT)(ceil((inst->Rob_Tool.tLoad.center_of_gravity[i_tp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
					}
					
					Convert_UDINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.RTOOL_COG[i_tp]);
					
					inst->PROFINET_Mapping_OUT.RTOOL_MOI[i_tp].INPUT = (UDINT)(ceil((inst->Rob_Tool.tLoad.moment_of_inertia[i_tp] * inst->Internal.ACCURACY_FACTOR)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					Convert_UDINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.RTOOL_MOI[i_tp]);
				}
				
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.RTOOL_POS_SIGN);
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.RTOOL_COG_SIGN);
				
				unsigned char i_fp;
				for(i_fp = 0; i_fp < (unsigned char)(sizeof(inst->Rob_Tool.tFrame.Rotation)/sizeof(inst->Rob_Tool.tFrame.Rotation[0])); i_fp++){
					if(inst->Rob_Tool.tFrame.Rotation[i_fp] >= 0.0){
						inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT_SIGN.INPUT[i_fp] = TRUE;
						inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[i_fp].INPUT      = (UDINT)(ceil((inst->Rob_Tool.tFrame.Rotation[i_fp] * inst->Internal.ACCURACY_FACTOR_QUATERNION)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					}else{
						inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT_SIGN.INPUT[i_fp] = FALSE;
						inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[i_fp].INPUT      = (-1) * ((UDINT)(ceil((inst->Rob_Tool.tFrame.Rotation[i_fp] * inst->Internal.ACCURACY_FACTOR_QUATERNION)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
					}
					
					Convert_UDINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT[i_fp]);
					
					if(inst->Rob_Tool.tLoad.intertial_axes[i_fp] >= 0.0){
						inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT_SIGN.INPUT[i_fp] = TRUE;
						inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[i_fp].INPUT      = (UDINT)(ceil((inst->Rob_Tool.tLoad.intertial_axes[i_fp] * inst->Internal.ACCURACY_FACTOR_QUATERNION)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR);
					}else{
						inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT_SIGN.INPUT[i_fp] = FALSE;
						inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[i_fp].INPUT      = (-1) * ((UDINT)(ceil((inst->Rob_Tool.tLoad.intertial_axes[i_fp] * inst->Internal.ACCURACY_FACTOR_QUATERNION)*CEIL_DEFUALT_FACTOR)/CEIL_DEFUALT_FACTOR));
					}
					
					Convert_UDINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT[i_fp]);
				}
				
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.RTOOL_ROT_QUAT_SIGN);
				Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.RTOOL_AOM_QUAT_SIGN);
						
				if(inst->Status.Robot.Update_Done == TRUE){
					inst->Command.UPDATE    = FALSE;
					inst->Command.ID.Update = UPT_ID_EMPTY;
					
					inst->Internal.Update.actual_state = UPT_STATE_WAIT;
				}
			}
			break;
	}
}

void Write_Data_PLC_to_ROB(struct ABB_Library* inst){
	/**
	Description:
		Function to write the basic data to the robot controller.
	
	Args:
		(1) inst [struct ABB_Library*]: Function block instance.
	*/
	
	inst->PROFINET_Mapping_OUT.STATUS.INPUT[0] = inst->Status.PLC.Active;
	inst->PROFINET_Mapping_OUT.STATUS.INPUT[1] = inst->Status.PLC.Update_Done;
	Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.STATUS);
	
	inst->PROFINET_Mapping_OUT.SYSTEM.INPUT[0] = inst->Command.System.MOTOR_ON;
	inst->PROFINET_Mapping_OUT.SYSTEM.INPUT[1] = inst->Command.System.MOTOR_OFF;
	inst->PROFINET_Mapping_OUT.SYSTEM.INPUT[2] = inst->Command.System.PP_TO_MAIN;
	inst->PROFINET_Mapping_OUT.SYSTEM.INPUT[3] = inst->Command.System.START;
	inst->PROFINET_Mapping_OUT.SYSTEM.INPUT[4] = inst->Command.System.STOP;
	Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.SYSTEM);
	
	inst->PROFINET_Mapping_OUT.COMMAND_mID = inst->Command.ID.Motion;
	inst->PROFINET_Mapping_OUT.COMMAND_uID = inst->Command.ID.Update;
	
	inst->PROFINET_Mapping_OUT.COMMAND.INPUT[0] = inst->Command.START;
	inst->PROFINET_Mapping_OUT.COMMAND.INPUT[1] = inst->Command.STOP;
	inst->PROFINET_Mapping_OUT.COMMAND.INPUT[2] = inst->Command.UPDATE;
	Convert_BOOL_Array_To_USINT(&inst->PROFINET_Mapping_OUT.COMMAND);
	
	inst->PROFINET_Mapping_OUT.TRAJECTORY_SIZE.INPUT = inst->Parameter.Trajectory_Size;
	Convert_UINT_To_USINT_Array(&inst->PROFINET_Mapping_OUT.TRAJECTORY_SIZE);	
}

void Read_Data_ROB_to_PLC(struct ABB_Library* inst){
	/**
	Description:
		Function to collect data from the robot controller.
	
	Args:
		(1) inst [struct ABB_Library*]: Function block instance.
	*/
	
	Convert_USINT_To_BOOL_Array(&inst->PROFINET_Mapping_IN.STATUS);
	inst->Status.Robot.Active      = inst->PROFINET_Mapping_IN.STATUS.OUTPUT[0];
	inst->Status.Robot.Update_Done = inst->PROFINET_Mapping_IN.STATUS.OUTPUT[1];
	inst->Status.Robot.In_Position = inst->PROFINET_Mapping_IN.STATUS.OUTPUT[2];
	
	inst->Status.Robot.ID.Motion = inst->PROFINET_Mapping_IN.STATE_mID;
	
	Convert_USINT_To_BOOL_Array(&inst->PROFINET_Mapping_IN.SYSTEM);
	inst->Status.Robot.System.MOTOR_ON  = inst->PROFINET_Mapping_IN.SYSTEM.OUTPUT[0];
	inst->Status.Robot.System.MOTOR_OFF = inst->PROFINET_Mapping_IN.SYSTEM.OUTPUT[1];
	inst->Status.Robot.System.PP_MOVED  = inst->PROFINET_Mapping_IN.SYSTEM.OUTPUT[2];
	inst->Status.Robot.System.CYCLE_ON  = inst->PROFINET_Mapping_IN.SYSTEM.OUTPUT[3];

	inst->Status.Robot.ID.Update = inst->PROFINET_Mapping_IN.STATE_uID;

	Convert_USINT_Array_To_UINT(&inst->PROFINET_Mapping_IN.MOTION_TRAJECTORY_ID);
	inst->Status.Robot.Trajectory_ID = inst->PROFINET_Mapping_IN.MOTION_TRAJECTORY_ID.OUTPUT;
	
	Convert_USINT_To_BOOL_Array(&inst->PROFINET_Mapping_IN.JOINT_POS_SIGN);
	unsigned char i_j;
	for(i_j = 0; i_j < (unsigned char)(sizeof(inst->RT_Data.Q)/sizeof(inst->RT_Data.Q[0])); i_j++){
		Convert_USINT_Array_To_UINT(&inst->PROFINET_Mapping_IN.JOINT_POS[i_j]);
		
		if(inst->PROFINET_Mapping_IN.JOINT_POS_SIGN.OUTPUT[i_j] == TRUE){
			inst->RT_Data.Q[i_j] = (REAL)(inst->PROFINET_Mapping_IN.JOINT_POS[i_j].OUTPUT) / inst->Internal.ACCURACY_FACTOR;
		}else{
			inst->RT_Data.Q[i_j] = (-1) * ((REAL)(inst->PROFINET_Mapping_IN.JOINT_POS[i_j].OUTPUT) / inst->Internal.ACCURACY_FACTOR);
		}
	}
	
	Convert_USINT_To_BOOL_Array(&inst->PROFINET_Mapping_IN.TCP_SIGN);
	unsigned char i_tcp;
	for(i_tcp = 0; i_tcp < (unsigned char)(sizeof(inst->RT_Data.TCP.Position)/sizeof(inst->RT_Data.TCP.Position[0])); i_tcp++){
		Convert_USINT_Array_To_UDINT(&inst->PROFINET_Mapping_IN.TCP_POS[i_tcp]);
		Convert_USINT_Array_To_UINT(&inst->PROFINET_Mapping_IN.TCP_ROT[i_tcp]);
		
		if(inst->PROFINET_Mapping_IN.TCP_SIGN.OUTPUT[i_tcp] == TRUE){
			inst->RT_Data.TCP.Position[i_tcp] = (REAL)(inst->PROFINET_Mapping_IN.TCP_POS[i_tcp].OUTPUT) / inst->Internal.ACCURACY_FACTOR;
		}else{
			inst->RT_Data.TCP.Position[i_tcp] = (-1) * ((REAL)(inst->PROFINET_Mapping_IN.TCP_POS[i_tcp].OUTPUT) / inst->Internal.ACCURACY_FACTOR);
		}
		
		if(inst->PROFINET_Mapping_IN.TCP_SIGN.OUTPUT[i_tcp + 3] == TRUE){
			inst->RT_Data.TCP.Rotation[i_tcp] = (REAL)(inst->PROFINET_Mapping_IN.TCP_ROT[i_tcp].OUTPUT) / inst->Internal.ACCURACY_FACTOR;
		}else{
			inst->RT_Data.TCP.Rotation[i_tcp] = (-1) * ((REAL)(inst->PROFINET_Mapping_IN.TCP_ROT[i_tcp].OUTPUT) / inst->Internal.ACCURACY_FACTOR);
		}
	}
	
	if(inst->PROFINET_Mapping_IN.TCP_SIGN.OUTPUT[6] == TRUE){
		inst->RT_Data.TCP.External_Position = (REAL)(inst->PROFINET_Mapping_IN.TCP_EX_POS.OUTPUT) / inst->Internal.ACCURACY_FACTOR;
	}else{
		inst->RT_Data.TCP.External_Position = (-1) * ((REAL)(inst->PROFINET_Mapping_IN.TCP_EX_POS.OUTPUT) / inst->Internal.ACCURACY_FACTOR);
	}
	
	Convert_USINT_To_BOOL_Array(&inst->PROFINET_Mapping_IN.TCP_CFG_SIGN);
	unsigned char i_cfg;
	for(i_cfg = 0; i_cfg < (unsigned char)(sizeof(inst->RT_Data.TCP.Configuration)/sizeof(inst->RT_Data.TCP.Configuration[0])); i_cfg++){
		if(inst->PROFINET_Mapping_IN.TCP_CFG_SIGN.OUTPUT[i_cfg] == TRUE){
			inst->RT_Data.TCP.Configuration[i_cfg] = (SINT)inst->PROFINET_Mapping_IN.TCP_CFG[i_cfg];
		}else{
			inst->RT_Data.TCP.Configuration[i_cfg] = (SINT)((-1) * inst->PROFINET_Mapping_IN.TCP_CFG[i_cfg]);
		}
	}
}

#endif