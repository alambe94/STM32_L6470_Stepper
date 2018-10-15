/*
 * uart_parse.c
 *
 *  Created on: 15-Oct-2018
 *      Author: medprime
 */

#include "uart_parse.h"

#define NO_OF_INDEX 5
#define X_AXIS_INDEX 0
#define Y_AXIS_INDEX 1
#define Z_AXIS_INDEX 2
#define M_AXIS_INDEX 3
#define G_CODE_INDEX 4

extern UART_HandleTypeDef huart2;

void UART_Loop()
    {

    uint8_t str_to_int[15] ={0};
    uint8_t cmd_rcv_flag[NO_OF_INDEX] ={0}; // command received for this axis
    uint8_t arg_rcv_flag[NO_OF_INDEX] ={0}; // argument received for this axis
    uint8_t motor_dir[4] ={0};
    uint8_t code_id = 0;
    uint8_t rx_digit_cnt = 0;
    uint8_t rx_byte = 0;
    uint8_t command_valid = 1;
    uint32_t rx_value[5] ={0}; // x,y,z,m and gcode

    if (Ring_Buffer_Check_Count() > 0)
	{

	uint8_t check_char ;
	Ring_Buffer_Check_Char(&check_char);

	if (check_char == '\n') // complete command received

	    {

	    uint8_t while_loop_timeout_1 = 128; //equal to ring buffer size

	    while (rx_byte != '\r' && --while_loop_timeout_1)
		{

		SCAN_AXIS:

		Ring_Buffer_Get_Char(&rx_byte);

		if (rx_byte == 'X' || rx_byte == 'x')
		    {
		    code_id = X_AXIS_INDEX;
		    cmd_rcv_flag[X_AXIS_INDEX] = 1;
		    goto PARSE_ARGUMENT;
		    }
		else if (rx_byte == 'Y' || rx_byte == 'y')
		    {
		    code_id = Y_AXIS_INDEX;
		    cmd_rcv_flag[Y_AXIS_INDEX] = 1;
		    goto PARSE_ARGUMENT;
		    }
		else if (rx_byte == 'Z' || rx_byte == 'z')
		    {
		    code_id = Z_AXIS_INDEX;
		    cmd_rcv_flag[Z_AXIS_INDEX] = 1;
		    goto PARSE_ARGUMENT;
		    }
		else if (rx_byte == 'M' || rx_byte == 'm')
		    {
		    code_id = M_AXIS_INDEX;
		    cmd_rcv_flag[M_AXIS_INDEX] = 1;
		    goto PARSE_ARGUMENT;
		    }
		else if (rx_byte == 'G' || rx_byte == 'g')
		    {
		    code_id = G_CODE_INDEX;
		    cmd_rcv_flag[G_CODE_INDEX] = 1;
		    goto PARSE_ARGUMENT;
		    }
		else
		    {
		    goto SKIP;
		    //skip parsing
		    }

		/**********************************PARSE_ARGUMENT******************************/
		PARSE_ARGUMENT:

		Ring_Buffer_Get_Char(&rx_byte);

		if (rx_byte == ' ') //if space
		    {
		    //rx_byte = Ring_Buffer_Get_Char(); //Ignore space
		    }
		if (rx_byte == '-') //if minus
		    {
		    if (code_id == G_CODE_INDEX) // G was detected
			{
			command_valid = 0;
			}
		    else
			{
			motor_dir[code_id] = L6470_DIR_FWD_ID; // L6470_DIR_REV_ID at reset
			Ring_Buffer_Get_Char(&rx_byte);
			}
		    }

		rx_digit_cnt = 0;

		uint8_t while_loop_timeout_2 = 128; //equal to ring bugger size

		while (rx_byte != '\r' && --while_loop_timeout_2)
		    {

		    if (rx_byte >= '0' && rx_byte <= '9') //if number
			{
			str_to_int[rx_digit_cnt++] = rx_byte;
			}
		    else if (rx_byte == ' ') // space found - scan for next axis
			{
			str_to_int[rx_digit_cnt++] = '\0'; // close string
			rx_value[code_id] = atoi((char*) str_to_int);
			arg_rcv_flag[code_id] = 1;
			goto SCAN_AXIS;
			}
		    else
			{
			command_valid = 0;
			}

		    Ring_Buffer_Get_Char(&rx_byte);

		    if (rx_byte == '\r')
			{
			str_to_int[rx_digit_cnt++] = '\0'; // close string
			rx_value[code_id] = atoi((char*) str_to_int);
			arg_rcv_flag[code_id] = 1;
			}

		    }

		SKIP:
		    {
		    }

		}
	    /**********************************PARSE_ARGUMENT******************************/

	    /***********command received without argument**********/

	     uint8_t arg_received = 0;
	     for (uint8_t i = 0; i < NO_OF_INDEX; i++)
	     {
	     if (cmd_rcv_flag[i])//command received
	     {
	     arg_received++;
	     if (arg_rcv_flag[i] == 0)//argument not received
	     {
	     command_valid = 0;
	     }
	     }
	     }
	     if(arg_received < 2)//at least two argument  g+arg  x,y,z or m+arg
	     {
	     command_valid = 0;
	     }

	    /***********command received without argument**********/

	    if (command_valid == 1)
		{

		switch (rx_value[G_CODE_INDEX])
		    //g code number
		    {
		case 1: //g1
		    {

		    L6470_PrepareMove(X_AXIS_INDEX, motor_dir[X_AXIS_INDEX],
			    rx_value[X_AXIS_INDEX]);

		    L6470_PrepareMove(Y_AXIS_INDEX, motor_dir[Y_AXIS_INDEX],
			    rx_value[Y_AXIS_INDEX]);

		    L6470_PrepareMove(Z_AXIS_INDEX, motor_dir[Z_AXIS_INDEX],
			    rx_value[Z_AXIS_INDEX]);

		    L6470_PrepareMove(M_AXIS_INDEX, motor_dir[M_AXIS_INDEX],
			    rx_value[M_AXIS_INDEX]);

		    if (cmd_rcv_flag[0] | cmd_rcv_flag[1] | cmd_rcv_flag[2]
			    | cmd_rcv_flag[3])
			{
			L6470_PerformPreparedApplicationCommand();
			}

		    }
		    break;
		case 0:
		    {

		    }
		    break;
		case 28: //g28 homing
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
		    break;
		default:
		    {

		    }
		    }

		HAL_UART_Transmit(&huart2, (uint8_t*) "OK\n", 3, 2);

		}
	    else
		{
		HAL_UART_Transmit(&huart2, (uint8_t*) "Invalid Command\n", 16,
			2);
		}

	    }
	}
    }
