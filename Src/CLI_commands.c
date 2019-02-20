/*
 * CLI_commands.c
 *
 *  Created on: 19-Oct-2018
 *      Author: medprime
 */

#include "CLI_commands.h"

#define NO_OF_INDEX 4
#define X_AXIS_INDEX 0
#define Y_AXIS_INDEX 1
#define Z_AXIS_INDEX 2
#define M_AXIS_INDEX 3


#define NO_OF_MOTORS 4
char motor_name[NO_OF_MOTORS] = {'x','y','z','m'};
char motor_name_caps[NO_OF_MOTORS] = {'X','Y','Z','M'};
char millis_int_to_str[10] = {0};
extern uint32_t   millis1;
extern UART_HandleTypeDef huart2;

char new_line[] = "\r\n";



uint8_t str_to_int(const char *str, uint32_t* steps, uint8_t* direction) //return if success
    {
    const char* str_copy = 0;
    uint8_t xreturn = 1;
    str_copy = str;

    if(*str_copy == '-')
	{
	str_copy++;
	str++;//skip sign
	*direction = 1; //reverse direction
	}
    if(*str_copy  == '\0' || *str_copy == ' ')//first char is null or space
	{
	xreturn = 0;//not number
	}
    while(*str_copy  != '\0' && *str_copy != ' ')
	{
	if(*str_copy < '0' || *str_copy > '9')
	    {
	    xreturn = 0;//not number
	    }
	str_copy++;
	}
    if(xreturn)
	{
	*steps = atoi(str);
	}
    return xreturn;
    }


/*************************************************************************/
 BaseType_t G28_callback(char *pcWriteBuffer, size_t xWriteBufferLen,
	const char *pcCommandString)
    {
     const char* invalid_parameter = "Invalid parameter ";
     const char* pcParameter;
     BaseType_t xParameterStringLength;
     UBaseType_t uxParameterNumber = 1;
     uint8_t cmd_rcv_flag[NO_OF_INDEX] ={0}; // command received for this axis
     uint8_t is_command_valid = 1;  //reset if any param is invalid
     uint8_t param_received = 0; //set if at least one param is valid

     /* Remove compile time warnings about unused parameters, and check the
      write buffer is not NULL.  NOTE - for simplicity, this example assumes the
      write buffer length is adequate, so does not check for buffer overflows. */
     (void) pcCommandString;
     (void) xWriteBufferLen;

     // to make sure pcWriteBuffer is always null terminated.
     memset(pcWriteBuffer, 0x00, xWriteBufferLen);

     sprintf(pcWriteBuffer, "G28 Ok:\r\n");

     do
 	{
 	/* Obtain the parameter string. */
 	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
 	uxParameterNumber, /* Return the next parameter. */
 	&xParameterStringLength /* Store the parameter string length. */
 	);

 	if (pcParameter != NULL)
 	    {

 	    uxParameterNumber++;

 	    switch (*pcParameter)
 		{

 	    case 'x':
 	    case 'X':
 		{
 		if (*(pcParameter+1) == ' ' || *(pcParameter+1) == '\0')
 		    {
 		    //success
 		    cmd_rcv_flag[X_AXIS_INDEX] = 1;
 		    param_received = 1;
 		    strncat(pcWriteBuffer, "X Home",strlen("X Home"));
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }
 		else
 		    {
 		    //invalid argument for x
 		    is_command_valid = 0;
 		    strncat(pcWriteBuffer, invalid_parameter, strlen(invalid_parameter));
 		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }

 		}
 		break;

 	    case 'y':
 	    case 'Y':
 		{
 		if (*(pcParameter+1) == ' ' || *(pcParameter+1) == '\0')
 		    {
 		    // string to int success
 		    cmd_rcv_flag[Y_AXIS_INDEX] = 1;
 		    param_received = 1;
 		    strncat(pcWriteBuffer, "Y Home",strlen("Y Home"));
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }
 		else
 		    {
 		    //invalid argument for y
 		    is_command_valid = 0;
 		    strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
 		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }
 		}
 		break;

 	    case 'z':
 	    case 'Z':
 		{
 		if (*(pcParameter+1) == ' ' || *(pcParameter+1) == '\0')
 		    {
 		    // success
 		    cmd_rcv_flag[Z_AXIS_INDEX] = 1;
 		    param_received = 1;
 		    strncat(pcWriteBuffer, "Z Home",strlen("Z Home"));
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }
 		else
 		    {
 		    //invalid argument for z
 		    is_command_valid = 0;
 		    strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
 		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }
 		}
 		break;

 	    case 'm':
 	    case 'M':
 		{
 		if (*(pcParameter+1) == ' ' || *(pcParameter+1) == '\0')
 		    {
 		    // success
 		    cmd_rcv_flag[M_AXIS_INDEX] = 1;
 		    param_received = 1;
 		    strncat(pcWriteBuffer, "M Home",strlen("M Home"));
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }
 		else
 		    {
 		    //invalid argument for m
 		    is_command_valid = 0;
 		    strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
 		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
 		    strncat(pcWriteBuffer, new_line, strlen(new_line));
 		    }
 		}
 		break;

 	    default:
 		{
 		//invalid parameter(s)
 		param_received = 1; //received but invalid
 		is_command_valid = 0;
 		strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
 		strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
 		strncat(pcWriteBuffer, new_line, strlen(new_line));
 		}

 		}
 	    }
 	else if(param_received == 0 && is_command_valid == 1) //zero param entered
 	    {
 		is_command_valid = 0;
 		strncat(pcWriteBuffer, "Parameter Not Entered",strlen("Parameter Not Entered"));
 		strncat(pcWriteBuffer, new_line, strlen(new_line));
 	    }


 	}
     while (pcParameter != NULL);

     /*Execute command*/

     if (is_command_valid == 1 && param_received == 1)
 	{

	    uint8_t perform_action = 0;
	    if (cmd_rcv_flag[X_AXIS_INDEX])
		{
		perform_action = 1;
		L6470_PrepareGoUntil(X_AXIS_INDEX, L6470_ACT_RST_ID,
			L6470_DIR_FWD_ID, 200000);
		}
	    if (cmd_rcv_flag[Y_AXIS_INDEX])
		{
		perform_action = 1;
		L6470_PrepareGoUntil(Y_AXIS_INDEX, L6470_ACT_RST_ID,
			L6470_DIR_REV_ID, 200000);
		}
	    if (cmd_rcv_flag[Z_AXIS_INDEX])
		{
		perform_action = 1;
		L6470_PrepareGoUntil(Z_AXIS_INDEX, L6470_ACT_RST_ID,
			L6470_DIR_REV_ID, 200000);
		}
	    if (cmd_rcv_flag[M_AXIS_INDEX])
		{
		perform_action = 1;
		L6470_PrepareGoUntil(M_AXIS_INDEX, L6470_ACT_RST_ID,
			L6470_DIR_REV_ID, 200000);
		}
	    if (perform_action)
		{
		L6470_PerformPreparedApplicationCommand();
		}

 	}

     return pdFALSE;
    }

const CLI_Command_Definition_t G28_defination =
    {
    "G28", /* The command string to type. */
    "\r\nG28: Motor Home\r\n",
    &G28_callback, /* The function to run. */
    -1 /* The user can enter any number of parameter. */
    };
const CLI_Command_Definition_t g28_defination =
    {
    "g28", /* The command string to type. */
    "",
    &G28_callback, /* The function to run. */
    -1 /* The user can enter any number of parameter. */
    };
/*************************************************************************/


/*************************************************************************/

BaseType_t G1_callback(char *pcWriteBuffer, size_t xWriteBufferLen,
	const char *pcCommandString)
    {
    strcpy( pcWriteBuffer, "G1 OK\r\n" );
    return pdFALSE;
    }

const CLI_Command_Definition_t G1_defination =
    {
    "G1", /* The command string to type. */
    "\r\nG1: Motor Move\r\n",
    &G1_callback, /* The function to run. */
    -1 /* The user can enter any number of parameter. */
    };

const CLI_Command_Definition_t g1_defination =
    {
    "g1", /* The command string to type. */
    "",
    &G1_callback, /* The function to run. */
    -1 /* The user can enter any number of parameter. */
    };
/*************************************************************************/



/*************************************************************************/

BaseType_t G0_callback(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
    {

    const char* invalid_parameter = "Invalid parameter ";
    const char* pcParameter;
    BaseType_t xParameterStringLength;
    UBaseType_t uxParameterNumber = 1;
    uint32_t motor_steps[NO_OF_MOTORS] = {0};
    uint8_t  motor_direction[NO_OF_MOTORS] = {0};
    uint8_t is_command_valid = 1;  //reset if any param is invalid
    uint8_t param_received = 0; //set if at least one param is valid

    /* Remove compile time warnings about unused parameters, and check the
     write buffer is not NULL.  NOTE - for simplicity, this example assumes the
     write buffer length is adequate, so does not check for buffer overflows. */
    (void) pcCommandString;
    (void) xWriteBufferLen;

    // to make sure pcWriteBuffer is always null terminated.
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);

    sprintf(pcWriteBuffer, "G0 Ok:\r\n");

    do
	{
	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
	uxParameterNumber, /* Return the next parameter. */
	&xParameterStringLength /* Store the parameter string length. */
	);

	if (pcParameter != NULL)
	    {

	    uxParameterNumber++;

	    switch (*pcParameter)
		{

	    case 'x':
	    case 'X':
		{
		if (str_to_int((pcParameter + 1), &motor_steps[X_AXIS_INDEX], &motor_direction[X_AXIS_INDEX]) == 1)
		    {
		    // string to int success
		    param_received = 1;
		    strncat(pcWriteBuffer, "X=",2);
		    strncat(pcWriteBuffer, (pcParameter + 1), (size_t) (xParameterStringLength - 1));
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }
		else
		    {
		    //invalid argument for x
		    is_command_valid = 0;
		    strncat(pcWriteBuffer, invalid_parameter, strlen(invalid_parameter));
		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }

		}
		break;

	    case 'y':
	    case 'Y':
		{
		if (str_to_int((pcParameter + 1), &motor_steps[Y_AXIS_INDEX], &motor_direction[Y_AXIS_INDEX]) == 1)
		    {
		    // string to int success
		    param_received = 1;
		    strncat(pcWriteBuffer, "Y=",2);
		    strncat(pcWriteBuffer, (pcParameter + 1), (size_t) (xParameterStringLength - 1));
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }
		else
		    {
		    //invalid argument for y
		    is_command_valid = 0;
		    strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }
		}
		break;

	    case 'z':
	    case 'Z':
		{
		if (str_to_int((pcParameter + 1), &motor_steps[Z_AXIS_INDEX], &motor_direction[Z_AXIS_INDEX]) == 1)
		    {
		    // success
		    param_received = 1;
		    strncat(pcWriteBuffer, "Z=",2);
		    strncat(pcWriteBuffer, (pcParameter + 1), (size_t) (xParameterStringLength - 1));
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }
		else
		    {
		    //invalid argument for z
		    is_command_valid = 0;
		    strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }
		}
		break;

	    case 'm':
	    case 'M':
		{
		if (str_to_int((pcParameter + 1), &motor_steps[M_AXIS_INDEX], &motor_direction[M_AXIS_INDEX]) == 1)
		    {
		    // success
		    param_received = 1;
		    strncat(pcWriteBuffer, "M=",2);
		    strncat(pcWriteBuffer, (pcParameter + 1), (size_t) (xParameterStringLength - 1));
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }
		else
		    {
		    //invalid argument for m
		    is_command_valid = 0;
		    strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
		    strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
		    strncat(pcWriteBuffer, new_line, strlen(new_line));
		    }
		}
		break;

	    default:
		{
		//invalid parameter(s)
		param_received = 1; //received but invalid
		is_command_valid = 0;
		strncat(pcWriteBuffer, invalid_parameter,strlen(invalid_parameter));
		strncat(pcWriteBuffer, pcParameter, (size_t) xParameterStringLength);
		strncat(pcWriteBuffer, new_line, strlen(new_line));
		}

		}
	    }
	else if(param_received == 0 && is_command_valid == 1) //zero param entered
	    {
		is_command_valid = 0;
		strncat(pcWriteBuffer, "Parameter Not Entered",strlen("Parameter Not Entered"));
		strncat(pcWriteBuffer, new_line, strlen(new_line));
	    }


	}
    while (pcParameter != NULL);

    /*Execute command*/

    if (is_command_valid == 1 && param_received == 1)
	{

	L6470_PrepareMove(X_AXIS_INDEX, motor_direction[X_AXIS_INDEX],
		motor_steps[X_AXIS_INDEX]);

	L6470_PrepareMove(Y_AXIS_INDEX, motor_direction[Y_AXIS_INDEX],
		motor_steps[Y_AXIS_INDEX]);

	L6470_PrepareMove(Z_AXIS_INDEX, motor_direction[Z_AXIS_INDEX],
		motor_steps[Z_AXIS_INDEX]);

	L6470_PrepareMove(M_AXIS_INDEX, motor_direction[M_AXIS_INDEX],
		motor_steps[M_AXIS_INDEX]);

	L6470_PerformPreparedApplicationCommand();

	}

    return pdFALSE;

    }

const CLI_Command_Definition_t G0_defination =
    {
    "G0", /* The command string to type. */
    "\r\nG0: Motor Move\r\n",
    &G0_callback, /* The function to run. */
    -1 /* The user can enter any number of parameter. */
    };

const CLI_Command_Definition_t g0_defination =
    {
    "g0", /* The command string to type. */
    "",
    &G0_callback, /* The function to run. */
    -1 /* The user can enter any number of parameter. */
    };
/*************************************************************************/


void CLI_Commands_Register()
    {
    FreeRTOS_CLIRegisterCommand(&G28_defination);
    FreeRTOS_CLIRegisterCommand(&g28_defination);

    FreeRTOS_CLIRegisterCommand(&G1_defination);
    FreeRTOS_CLIRegisterCommand(&g1_defination);


    FreeRTOS_CLIRegisterCommand(&G0_defination);
    FreeRTOS_CLIRegisterCommand(&g0_defination);
    }



