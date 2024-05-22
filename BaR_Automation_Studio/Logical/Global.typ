
TYPE
	VInRoS_Str : 	STRUCT 
		Mech_Id_1 : Mechanism_Str;
		Mech_Id_2 : Mechanism_Str;
		Mech_Id_3 : Mechanism_Str;
		Rob_Id_1 : Robot_Str;
		Rob_Id_2_L : Robot_Str;
		Rob_Id_2_R : Robot_Str;
		EX_600 : EX_600_Str;
	END_STRUCT;
	Mechanism_Str : 	STRUCT 
		Command : Mechanism_Command_Str;
		Info : Mechanism_Info_Str;
		Parameters : Mechanism_Parameters_Str;
		Position : LREAL;
	END_STRUCT;
	Mechanism_Command_Str : 	STRUCT 
		Power : BOOL;
		Home : BOOL;
		Start : BOOL;
		Stop : BOOL;
		Reset_Error : BOOL;
		Reset_Safety : BOOL;
		Update : BOOL;
		Move : BOOL;
	END_STRUCT;
	Mechanism_Info_Str : 	STRUCT 
		Active : BOOL;
		Power : BOOL;
		Home : BOOL;
		Safety : BOOL;
		Error : BOOL;
		In_Position : BOOL;
		Update_Done : BOOL;
		Move_Active : BOOL;
	END_STRUCT;
	Mechanism_Parameters_Str : 	STRUCT 
		Position : LREAL;
		Velocity : REAL;
		Acc_Dec : REAL;
	END_STRUCT;
	EX_600_Str : 	STRUCT 
		Slave : EX_600_Slave_Str;
	END_STRUCT;
	EX_600_Slave_Str : 	STRUCT 
		Command : EX_600_Command_Str;
		Info : EX_600_Info_Str;
		Input : ARRAY[0..15]OF BOOL;
		Output : ARRAY[0..7]OF BOOL;
	END_STRUCT;
	EX_600_Command_Str : 	STRUCT 
		Home : BOOL;
		Reset_Error : BOOL;
	END_STRUCT;
	EX_600_Info_Str : 	STRUCT 
		Active : BOOL;
		Home : BOOL;
		Error : BOOL;
	END_STRUCT;
	Robot_Str : 	STRUCT 
		Command : Robot_Command_Str;
		Info : Robot_Info_Str;
		Position : Robot_Position_Str;
	END_STRUCT;
	Robot_Command_Str : 	STRUCT 
		Power : BOOL;
		Home : BOOL;
		Start : BOOL;
		Stop : BOOL;
		Reset_Error : BOOL;
		Reset_Safety : BOOL;
	END_STRUCT;
	Robot_Info_Str : 	STRUCT 
		Active : BOOL;
		Power : BOOL;
		Home : BOOL;
		Safety : BOOL;
		Error : BOOL;
		In_Position : BOOL;
		Update_Done : BOOL;
		Move_Active : BOOL;
	END_STRUCT;
	Robot_Position_Str : 	STRUCT 
		Q : ARRAY[0..6]OF REAL;
	END_STRUCT;
END_TYPE
