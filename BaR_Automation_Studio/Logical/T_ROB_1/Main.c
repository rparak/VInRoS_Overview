#include <bur/plctypes.h>

#ifdef _DEFAULT_INCLUDES
#include <AsDefault.h>
#endif

#include <Main.h>

_LOCAL enum Robot_State_ID_enum state_id;

void _INIT ProgramInit(void)
{
	// 
	memset(&Global_VInRoS_Str.Rob_Id_1, 0, sizeof(Global_VInRoS_Str.Rob_Id_1));
	
	ABB_Library_Rob_1.Enable = TRUE;
	
	//
	ABB_Library_Rob_1.Rob_Tool.robHold = FALSE;
	ABB_Library_Rob_1.Rob_Tool.tFrame.Position[0] = 0.0; ABB_Library_Rob_1.Rob_Tool.tFrame.Position[1] = 0.0; 
	ABB_Library_Rob_1.Rob_Tool.tFrame.Position[2] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tFrame.Rotation[0] = 1.0; ABB_Library_Rob_1.Rob_Tool.tFrame.Rotation[1] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tFrame.Rotation[2] = 0.0; ABB_Library_Rob_1.Rob_Tool.tFrame.Rotation[3] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tLoad.mass = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tLoad.center_of_gravity[0] = 0.0; ABB_Library_Rob_1.Rob_Tool.tLoad.center_of_gravity[1] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tLoad.center_of_gravity[2] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tLoad.intertial_axes[0] = 1.0; ABB_Library_Rob_1.Rob_Tool.tLoad.intertial_axes[1] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tLoad.intertial_axes[2] = 0.0; ABB_Library_Rob_1.Rob_Tool.tLoad.intertial_axes[3] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tLoad.moment_of_inertia[0] = 0.0; ABB_Library_Rob_1.Rob_Tool.tLoad.moment_of_inertia[1] = 0.0;
	ABB_Library_Rob_1.Rob_Tool.tLoad.moment_of_inertia[2] = 0.0;
	
	//
	Set_Trajectory_Parameters(&Trajectory_Str);
}

void _CYCLIC ProgramCyclic(void)
{
	switch(state_id){
		case ROB_STATE_ACTIVE:
			{
				if(Global_VInRoS_Str.Rob_Id_1.Info.Active == TRUE){
					state_id = ROB_STATE_POWER;
				}
			}
			break;
			
		case ROB_STATE_POWER:
			{
				if(Global_VInRoS_Str.Rob_Id_1.Command.Power == TRUE){
					ABB_Library_Rob_1.Power_ON = TRUE;
					Global_VInRoS_Str.Rob_Id_1.Command.Power = FALSE;
				}
				
				if(Global_VInRoS_Str.Rob_Id_1.Info.Power == TRUE && ABB_Library_Rob_1.Status.Robot.ID.Motion == ABBr_STATE_WAIT){
					state_id = ROB_STATE_WAIT;
				}
			}
			break;
			
		case ROB_STATE_WAIT:
			{
				ABB_Library_Rob_1.Command.STOP  = FALSE;
				ABB_Library_Rob_1.Command.START = FALSE;
				
				if(Global_VInRoS_Str.Rob_Id_1.Command.Home == TRUE){
					state_id = ROB_STATE_HOME_UPD_PARAMETERS_1;
				}
				
				if(Global_VInRoS_Str.Rob_Id_1.Command.Start == TRUE){
					state_id = ROB_STATE_UPD_PARAMETERS_1;
				}
			}
			break;
			
		case ROB_STATE_UPD_PARAMETERS_1:
			{
				Global_VInRoS_Str.Rob_Id_1.Info.Update_Done = FALSE;

				memcpy(ABB_Library_Rob_1.Parameter.Joint, Trajectory_Str.Targets.Joint, 
					(sizeof(Trajectory_Str.Targets.Joint) / sizeof(Trajectory_Str.Targets.Joint[0])) * sizeof(REAL));
				int i;
				for(i = 0; i < Trajectory_Str.Length; i++){
					ABB_Library_Rob_1.Parameter.Speed[i] = Trajectory_Str.Targets.Speed[i];
					ABB_Library_Rob_1.Parameter.Zone[i] = Trajectory_Str.Targets.Zone[i];
				}
				
				ABB_Library_Rob_1.Parameter.Trajectory_Size = Trajectory_Str.Length;
				
				ABB_Library_Rob_1.Command.ID.Update = UPT_ID_TRAJECTORY_JOINT;
				ABB_Library_Rob_1.Command.UPDATE    = TRUE;
				
				if(ABB_Library_Rob_1.Internal.Update.actual_state != UPT_STATE_WAIT){
					state_id = ROB_STATE_UPD_PARAMETERS_2;
				}
			}
			break;
			
		case ROB_STATE_UPD_PARAMETERS_2:
			{
				if(ABB_Library_Rob_1.Status.Robot.Update_Done == TRUE && ABB_Library_Rob_1.Internal.Update.actual_state == UPT_STATE_WAIT){
					Global_VInRoS_Str.Rob_Id_1.Info.Update_Done = TRUE;
					state_id = ROB_STATE_MOTION_1;
				}
			}
			break;
		
		case ROB_STATE_MOTION_1:
			{
				ABB_Library_Rob_1.Command.ID.Motion = MAIN_ID_JOINT_ABSOLUTE;
				ABB_Library_Rob_1.Command.START = TRUE;
				
				if(Global_VInRoS_Str.Rob_Id_1.Info.Move_Active == TRUE){
					state_id = ROB_STATE_MOTION_2;
				}
			}
			break;
			
		case ROB_STATE_MOTION_2:
			{
				ABB_Library_Rob_1.Command.START = FALSE;
				
				Trajectory_Str.Iteration = ABB_Library_Rob_1.Status.Robot.Trajectory_ID;
				
				if(Global_VInRoS_Str.Rob_Id_1.Command.Stop == TRUE){
					state_id = ROB_STATE_STOP;
				}else{
					if(Global_VInRoS_Str.Rob_Id_1.Info.In_Position == TRUE){
						state_id = ROB_STATE_WAIT;
					}
				}
			}
			break;
			
		case ROB_STATE_HOME_UPD_PARAMETERS_1:
			{
				Global_VInRoS_Str.Rob_Id_1.Info.Update_Done = FALSE;
				Global_VInRoS_Str.Rob_Id_1.Command.Home = FALSE;
				
				ABB_Library_Rob_1.Parameter.Joint[0].Q[0] = 90.0; ABB_Library_Rob_1.Parameter.Joint[0].Q[1] = 0.0;
				ABB_Library_Rob_1.Parameter.Joint[0].Q[2] = 0.0; ABB_Library_Rob_1.Parameter.Joint[0].Q[3] = 0.0;
				ABB_Library_Rob_1.Parameter.Joint[0].Q[4] = 90.0; ABB_Library_Rob_1.Parameter.Joint[0].Q[5] = 0.0;
				ABB_Library_Rob_1.Parameter.Speed[0] = vSPEED_100;
				ABB_Library_Rob_1.Parameter.Zone[0]  = zZone_fine;
				
				ABB_Library_Rob_1.Parameter.Trajectory_Size = 1;
				
				ABB_Library_Rob_1.Command.ID.Update = UPT_ID_TRAJECTORY_JOINT;
				ABB_Library_Rob_1.Command.UPDATE    = TRUE;
				
				if(ABB_Library_Rob_1.Internal.Update.actual_state != UPT_STATE_WAIT){
					state_id = ROB_STATE_HOME_UPD_PARAMETERS_2;
				}
			}
			break;
			
		case ROB_STATE_HOME_UPD_PARAMETERS_2:
			{
				if(ABB_Library_Rob_1.Status.Robot.Update_Done == TRUE && ABB_Library_Rob_1.Internal.Update.actual_state == UPT_STATE_WAIT){
					Global_VInRoS_Str.Rob_Id_1.Info.Update_Done = TRUE;
					state_id = ROB_STATE_HOME_MOTION_1;
				}
			}
			break;
			
		case ROB_STATE_HOME_MOTION_1:
			{
				Global_VInRoS_Str.Rob_Id_1.Info.Home = FALSE;
				
				ABB_Library_Rob_1.Command.ID.Motion = MAIN_ID_JOINT_ABSOLUTE;
				ABB_Library_Rob_1.Command.START = TRUE;
				
				if(Global_VInRoS_Str.Rob_Id_1.Info.Move_Active == TRUE){
					state_id = ROB_STATE_HOME_MOTION_2;
				}
			}
			break;
			
		case ROB_STATE_HOME_MOTION_2:
			{
				ABB_Library_Rob_1.Command.START = FALSE;
				
				if(Global_VInRoS_Str.Rob_Id_1.Command.Stop == TRUE){
					state_id = ROB_STATE_STOP;
				}else{
					if(Global_VInRoS_Str.Rob_Id_1.Info.In_Position == TRUE){
						Global_VInRoS_Str.Rob_Id_1.Info.Home = TRUE;
						state_id = ROB_STATE_WAIT;
					}
				}
			}
			break;
			
		case ROB_STATE_STOP:
			{
				Global_VInRoS_Str.Rob_Id_1.Command.Home  = FALSE;
				Global_VInRoS_Str.Rob_Id_1.Command.Start = FALSE;
				Global_VInRoS_Str.Rob_Id_1.Command.Stop  = FALSE;
				
				if(ABB_Library_Rob_1.Internal.actual_state == ABBt_STATE_WAIT && ABB_Library_Rob_1.Status.Robot.ID.Motion == ABBr_STATE_WAIT){
					state_id = ROB_STATE_WAIT;
				}else{
					ABB_Library_Rob_1.Command.STOP = TRUE;
				}
			}
			break;
			
		case ROB_STATE_SAFETY:
			{
			}
			break;
			
		case ROB_STATE_ERROR:
			{	
			}
			break;
	}
	
	ABB_Library(&ABB_Library_Rob_1);
	
	Global_VInRoS_Str.Rob_Id_1.Info.Active = ABB_Library_Rob_1.Status.Robot.Active;
	Global_VInRoS_Str.Rob_Id_1.Info.Power = ABB_Library_Rob_1.Status.Robot.System.MOTOR_ON;
	if(ABB_Library_Rob_1.Status.Robot.In_Position == TRUE && ABB_Library_Rob_1.Status.Robot.ID.Motion == ABBr_STATE_WAIT){
		Global_VInRoS_Str.Rob_Id_1.Info.In_Position = TRUE;
	}else{
		Global_VInRoS_Str.Rob_Id_1.Info.In_Position = FALSE;
	}
	Global_VInRoS_Str.Rob_Id_1.Info.Error  = FALSE;
	Global_VInRoS_Str.Rob_Id_1.Info.Safety = FALSE;
	
	if(Global_VInRoS_Str.Rob_Id_1.Info.Power == TRUE){
		if(Global_VInRoS_Str.Rob_Id_1.Info.In_Position == FALSE){
			Global_VInRoS_Str.Rob_Id_1.Info.Move_Active = TRUE;
		}else{
			Global_VInRoS_Str.Rob_Id_1.Info.Move_Active = FALSE;
		}
	}
	
	memcpy(Global_VInRoS_Str.Rob_Id_1.Position.Q, ABB_Library_Rob_1.RT_Data.Q, 
		(sizeof(ABB_Library_Rob_1.RT_Data.Q) / sizeof(ABB_Library_Rob_1.RT_Data.Q[0])) * sizeof(REAL));
	
	if(Global_VInRoS_Str.Rob_Id_1.Info.Error == TRUE){
		state_id = ROB_STATE_ERROR;	
	}
}

