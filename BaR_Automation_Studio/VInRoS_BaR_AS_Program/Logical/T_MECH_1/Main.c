#include <bur/plctypes.h>

#ifdef _DEFAULT_INCLUDES
#include <AsDefault.h>
#endif

#include "Main.h"

_LOCAL struct MpAxisBasic MpAxisBasic_0;
_LOCAL struct MpAxisBasicParType AxisParameters;
_LOCAL enum Mechanism_State_ID_enum state_id;

void _INIT ProgramInit(void)
{
	memset(&Global_VInRoS_Str.Mech_Id_1, 0, sizeof(Global_VInRoS_Str.Mech_Id_1));
	
	AxisParameters.Homing.Mode = mcHOMING_ABSOLUTE;
	AxisParameters.Homing.Position = 1770365;
	
	MpAxisBasic_0.Enable     = 1;
	MpAxisBasic_0.MpLink     = &gAxis_Mech_01;
	MpAxisBasic_0.Parameters = &AxisParameters;
	
	Trajectory_Str.Targets.Position[0] = 100.0;
	Trajectory_Str.Targets.Position[1] = 1100.0;
	Trajectory_Str.Targets.Position[2] = 700.0;
	Trajectory_Str.Targets.Position[3] = 200.0;
	Trajectory_Str.Targets.Position[4] = 1200.0;
	
	Trajectory_Str.Targets.Velocity[0] = 500.0;
	Trajectory_Str.Targets.Velocity[1] = 1000.0;
	Trajectory_Str.Targets.Velocity[2] = 200.0;
	Trajectory_Str.Targets.Velocity[3] = 500.0;
	Trajectory_Str.Targets.Velocity[4] = 400.0;
	
	Trajectory_Str.Length = 5;
	
	state_id = MECH_STATE_ACTIVE;
}

void _CYCLIC ProgramCyclic(void)
{
	switch(state_id){
		case MECH_STATE_ACTIVE:
			{
				if(Global_VInRoS_Str.Mech_Id_1.Info.Active == TRUE){
					state_id = MECH_STATE_POWER;
				}
			}
			break;
		
		case MECH_STATE_POWER:
			{
				if(Global_VInRoS_Str.Mech_Id_1.Info.Power == TRUE){
					state_id = MECH_STATE_HOME;
				}
			}
			break;
		
		case MECH_STATE_HOME:
			{
				if(Global_VInRoS_Str.Mech_Id_1.Info.Home == TRUE){
					MpAxisBasic_0.Home = FALSE;
					Global_VInRoS_Str.Mech_Id_1.Command.Home = FALSE;
					state_id = MECH_STATE_WAIT;
				}
			}
			break;
		
		case MECH_STATE_WAIT:
			{
				if(Global_VInRoS_Str.Mech_Id_1.Command.Home == TRUE){
					state_id = MECH_STATE_HOME_UPD_PARAMETERS;
				}
				
				if(Global_VInRoS_Str.Mech_Id_1.Command.Start == TRUE){
					Trajectory_Str.Iteration = 0;
					state_id = MECH_STATE_UPD_PARAMETERS;
				}
			}
			break;
		
		case MECH_STATE_UPD_PARAMETERS:
			{
				Global_VInRoS_Str.Mech_Id_1.Parameters.Position = Trajectory_Str.Targets.Position[Trajectory_Str.Iteration];
				Global_VInRoS_Str.Mech_Id_1.Parameters.Velocity = Trajectory_Str.Targets.Velocity[Trajectory_Str.Iteration];
				Global_VInRoS_Str.Mech_Id_1.Parameters.Acc_Dec  = Trajectory_Str.Targets.Velocity[Trajectory_Str.Iteration]*10;
				
				Global_VInRoS_Str.Mech_Id_1.Command.Update = TRUE;
				if(Global_VInRoS_Str.Mech_Id_1.Info.Update_Done == TRUE){
					state_id = MECH_STATE_MOTION_1;
				}
			}
			break;
		
		case MECH_STATE_MOTION_1:
			{
				Global_VInRoS_Str.Mech_Id_1.Command.Move = TRUE;
				if(Global_VInRoS_Str.Mech_Id_1.Info.Move_Active == TRUE){
					state_id = MECH_STATE_MOTION_2;
				}
			}
			break;
		
		case MECH_STATE_MOTION_2:
			{
				if(Global_VInRoS_Str.Mech_Id_1.Command.Stop == TRUE){
					state_id = MECH_STATE_STOP;
				}else{
					if(Global_VInRoS_Str.Mech_Id_1.Info.In_Position == TRUE){
						Global_VInRoS_Str.Mech_Id_1.Command.Move = 0;
					
						if(Trajectory_Str.Iteration == Trajectory_Str.Length - 1){
							state_id = MECH_STATE_WAIT;
						}else{
							Trajectory_Str.Iteration++;
							state_id = MECH_STATE_UPD_PARAMETERS;
						}
					}
				}
			}
			break;
		
		case MECH_STATE_HOME_UPD_PARAMETERS:
			{
				Global_VInRoS_Str.Mech_Id_1.Parameters.Position = 700.0;
				Global_VInRoS_Str.Mech_Id_1.Parameters.Velocity = 500.0;
				Global_VInRoS_Str.Mech_Id_1.Parameters.Acc_Dec  = 5000.0;
				
				Global_VInRoS_Str.Mech_Id_1.Command.Update = TRUE;
				if(Global_VInRoS_Str.Mech_Id_1.Info.Update_Done == TRUE){
					state_id = MECH_STATE_HOME_MOTION_1;
				}
			}
			break;
		
		case MECH_STATE_HOME_MOTION_1:
			{		
				Global_VInRoS_Str.Mech_Id_1.Command.Move = TRUE;
				if(Global_VInRoS_Str.Mech_Id_1.Info.Move_Active == TRUE){
					state_id = MECH_STATE_HOME_MOTION_2;
				}	
			}
			break;
		
		case MECH_STATE_HOME_MOTION_2:
			{
				if(Global_VInRoS_Str.Mech_Id_1.Info.In_Position == TRUE){
					Global_VInRoS_Str.Mech_Id_1.Command.Move = FALSE;
					Global_VInRoS_Str.Mech_Id_1.Command.Home = FALSE;
					state_id = MECH_STATE_WAIT;
				}
			}
			break;
		
		
		case MECH_STATE_STOP:
			{
				Global_VInRoS_Str.Mech_Id_1.Command.Stop  = FALSE;
				Global_VInRoS_Str.Mech_Id_1.Command.Start = FALSE;
				
				Global_VInRoS_Str.Mech_Id_1.Command.Move = FALSE;
				if(Global_VInRoS_Str.Mech_Id_1.Info.Move_Active == FALSE){
					state_id = MECH_STATE_WAIT;
				}
			}
			break;
		
		case MECH_STATE_SAFETY:
			{
			}
			break;
		
		case MECH_STATE_ERROR:
			{
				Global_VInRoS_Str.Mech_Id_1.Command.Home   = FALSE;
				Global_VInRoS_Str.Mech_Id_1.Command.Update = FALSE;
				Global_VInRoS_Str.Mech_Id_1.Command.Stop   = FALSE;
				Global_VInRoS_Str.Mech_Id_1.Command.Start  = FALSE;
				Global_VInRoS_Str.Mech_Id_1.Command.Move   = FALSE;	
				
				if(Global_VInRoS_Str.Mech_Id_1.Info.Error == FALSE){
					state_id = MECH_STATE_WAIT;	
				}
			}
			break;
		
	}
	
	AxisParameters.Position     = Global_VInRoS_Str.Mech_Id_1.Parameters.Position;
	AxisParameters.Velocity     = Global_VInRoS_Str.Mech_Id_1.Parameters.Velocity;
	AxisParameters.Acceleration = Global_VInRoS_Str.Mech_Id_1.Parameters.Acc_Dec;
	AxisParameters.Deceleration = Global_VInRoS_Str.Mech_Id_1.Parameters.Acc_Dec;
	
	MpAxisBasic_0.Power = Global_VInRoS_Str.Mech_Id_1.Command.Power;
	if(Global_VInRoS_Str.Mech_Id_1.Command.Home == TRUE && Global_VInRoS_Str.Mech_Id_1.Info.Home == FALSE){
		MpAxisBasic_0.Home = TRUE;
	}
	MpAxisBasic_0.ErrorReset = Global_VInRoS_Str.Mech_Id_1.Command.Reset_Error;
	MpAxisBasic_0.Update     = Global_VInRoS_Str.Mech_Id_1.Command.Update;
	MpAxisBasic_0.MoveAbsolute = Global_VInRoS_Str.Mech_Id_1.Command.Move;
	
	MpAxisBasic(&MpAxisBasic_0);
	
	Global_VInRoS_Str.Mech_Id_1.Info.Active = MpAxisBasic_0.Active;
	Global_VInRoS_Str.Mech_Id_1.Info.Power  = MpAxisBasic_0.PowerOn;
	Global_VInRoS_Str.Mech_Id_1.Info.Home   = MpAxisBasic_0.IsHomed;
	Global_VInRoS_Str.Mech_Id_1.Info.Error  = MpAxisBasic_0.Error;
	Global_VInRoS_Str.Mech_Id_1.Info.Update_Done = MpAxisBasic_0.UpdateDone;
	Global_VInRoS_Str.Mech_Id_1.Info.Move_Active = MpAxisBasic_0.MoveActive;
	if(MpAxisBasic_0.InPosition == TRUE && MpAxisBasic_0.MoveDone == TRUE){
		Global_VInRoS_Str.Mech_Id_1.Info.In_Position = TRUE;
	}else{
		Global_VInRoS_Str.Mech_Id_1.Info.In_Position = FALSE;
	}
	Global_VInRoS_Str.Mech_Id_1.Info.Safety = FALSE;
	
	Global_VInRoS_Str.Mech_Id_1.Position = MpAxisBasic_0.Position;
	
	if(Global_VInRoS_Str.Mech_Id_1.Info.Error == TRUE){
		state_id = MECH_STATE_ERROR;	
	}
}

