
TYPE
	Traj_Str : 	STRUCT 
		Targets : Targets_Str;
		Length : USINT;
		Iteration : USINT;
	END_STRUCT;
	Targets_Str : 	STRUCT 
		Joint : ARRAY[0..99]OF Targets_Joint_Str;
		Speed : ARRAY[0..99]OF USINT;
		Zone : ARRAY[0..99]OF USINT;
	END_STRUCT;
	Targets_Joint_Str : 	STRUCT 
		Q : ARRAY[0..6]OF REAL;
	END_STRUCT;
END_TYPE
