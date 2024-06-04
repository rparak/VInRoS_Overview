
TYPE
	Traj_Str : 	STRUCT 
		Targets : Targets_Str;
		Length : USINT;
		Iteration : USINT;
	END_STRUCT;
	Targets_Str : 	STRUCT 
		Position : ARRAY[0..99]OF LREAL;
		Velocity : ARRAY[0..99]OF REAL;
	END_STRUCT;
END_TYPE
