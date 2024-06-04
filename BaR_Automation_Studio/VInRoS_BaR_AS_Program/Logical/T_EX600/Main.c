// Include B&R Automation libraries (declarations for B&R ANSI C extensions).
#include <bur/plctypes.h>

#ifdef _DEFAULT_INCLUDES
#include <AsDefault.h>
#endif

#include <Main.h>

// Custom Lib.: Base conversion between data.
#include <Converter.h>

/*  Local Variables.  */
_LOCAL enum EX600_State_ID_enum state_id;

_LOCAL struct Convert_BOOL_Array_To_USINT C_BOOL_Arr_To_USINT_1;
_LOCAL struct Convert_USINT_To_BOOL_Array C_USINT_To_BOOL_Arr_1;
_LOCAL struct Convert_USINT_To_BOOL_Array C_USINT_To_BOOL_Arr_2;

/**
 * Program Intitialization
 */
void _INIT ProgramInit(void)
{
	memset(&Global_VInRoS_Str.EX_600, 0, sizeof(Global_VInRoS_Str.EX_600));
	
	state_id = EX600_STATE_ACTIVE;
}

/**
 * Program Cyclic 
 * 
 * Duration (Cycle Time): 10000 [ms] 
 * Tolerance            : 10000 [ms]
 */
void _CYCLIC ProgramCyclic(void)
{
	switch(state_id){
		case EX600_STATE_ACTIVE:
			{
				if(Global_VInRoS_Str.EX_600.Slave.Info.Active == TRUE){
					state_id = EX600_STATE_HOME;
				}
			}
			break;
		
		case EX600_STATE_HOME:
			{
				if(Global_VInRoS_Str.EX_600.Slave.Command.Home == TRUE){
					memset(C_BOOL_Arr_To_USINT_1.INPUT, 0, sizeof(C_BOOL_Arr_To_USINT_1.INPUT));
					C_USINT_To_BOOL_Arr_1.INPUT = FALSE;
					C_USINT_To_BOOL_Arr_2.INPUT = FALSE;
					
					Global_VInRoS_Str.EX_600.Slave.Info.Home = TRUE;
					Global_VInRoS_Str.EX_600.Slave.Command.Home = FALSE;
					state_id = EX600_STATE_WAIT;
				}
			}
			break;
		
		case EX600_STATE_WAIT:
			{
				int i;
				for(i = FALSE; i < (int)(sizeof(C_BOOL_Arr_To_USINT_1.INPUT)/sizeof(C_BOOL_Arr_To_USINT_1.INPUT[0])); i++){
					C_BOOL_Arr_To_USINT_1.INPUT[i] = Global_VInRoS_Str.EX_600.Slave.Output[i];
				}
				Convert_BOOL_Array_To_USINT(&C_BOOL_Arr_To_USINT_1);
				
				Convert_USINT_To_BOOL_Array(&C_USINT_To_BOOL_Arr_1);
				int j;
				for(j = FALSE; j < (int)(sizeof(C_USINT_To_BOOL_Arr_1.OUTPUT)/sizeof(C_USINT_To_BOOL_Arr_1.OUTPUT[0])); j++){
					Global_VInRoS_Str.EX_600.Slave.Input[j] = C_USINT_To_BOOL_Arr_1.OUTPUT[j];
				}
				Convert_USINT_To_BOOL_Array(&C_USINT_To_BOOL_Arr_2);
				int k;
				for(k = FALSE; k < (int)(sizeof(C_USINT_To_BOOL_Arr_2.OUTPUT)/sizeof(C_USINT_To_BOOL_Arr_2.OUTPUT[0])); k++){
					Global_VInRoS_Str.EX_600.Slave.Input[k + 8] = C_USINT_To_BOOL_Arr_2.OUTPUT[k];
				}
				
			}
			break;
		
		case EX600_STATE_ERROR:
			{
				if(Global_VInRoS_Str.EX_600.Slave.Info.Error == FALSE){
					state_id = EX600_STATE_WAIT;	
				}
			}
			break;
	}
	
	if(Global_VInRoS_Str.EX_600.Slave.Info.Error == TRUE){
		state_id = EX600_STATE_ERROR;	
	}
}
