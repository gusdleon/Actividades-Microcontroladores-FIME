
		AREA		myCode, CODE,READONLY
		ENTRY
		EXPORT		__main
	
__main
LOOP
		MOV			R1,#0
		MOV			R2,#0
		;Modo inmediato
		MOV			R1,#4		
		MOV			R2,#3
		
		ADD			R3,R1,R2
		B			LOOP

		END