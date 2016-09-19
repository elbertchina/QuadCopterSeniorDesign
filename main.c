/*******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description:
*   Enumerates as a Virtual Com port.  Receives data from hyper terminal, then 
*   send received data backward. LCD shows the Line settings.
*   
*  To test project:
*   1. Build the project and program the hex file on to the target device.
*   2. Select 3.3V in SW3 and plug-in power to the CY8CKIT-001
*   3. Connect USB cable from the computer to the CY8CKIT-001.
*   4. Select the USB_UART.inf file from the project directory, as the driver 
*      for this example once Windows asks for it.
*   5. Open "Device Manager" and note the COM port number for "Example Project"
*      device.
*   6. Open "HyperTerminal" application and make new connection to noted COM port
*   7. Type the message, observe echo data received.
*
* Related Document:
*  Universal Serial Bus Specification Revision 2.0 
*  Universal Serial Bus Class Definitions for Communications Devices 
*  Revision 1.2
*
********************************************************************************
* Copyright 2012, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

#include <device.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "float.h" 

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow the usage of floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif

/* The size of the buffer is equal to maximum packet size of the 
*  IN and OUT bulk endpoints. 
*/
#define BUFFER_LEN  64u

float yawave[10],pitchave[10],rollave[10],psiave[10],thetaave[10],phiave[10];
 uint16 count;
	int counter = 0;
    char buffer[4],buffer1[4];
    char8 lineStr[20];
    uint8 state;
	int i = 0;
	char sensor[30],sensor1[30],sensor2[30];
	int recordFlag = 0;
	int dataReady = 0;
	int yaw_int,pitch_int,roll_int,P_int,Q_int,R_int;
	int phi_int,psi_int,theta_int;
	int motor1_int,motor2_int,motor3_int,motor4_int,motor1RPM_int,motor2RPM_int,motor3RPM_int,motor4RPM_int;
	char temp[6];
	char print_YPR[120];
	float yaw,pitch,roll,phi,theta,psi,P,Q,R;
	float yawsum=0,pitchsum=0,rollsum=0;
	float yaw_p = 0,pitch_p = 0,roll_p = 0;
	float freq = 10;
	float motor1, motor1RPM, motor1Throttle, motor2, motor2RPM, motor2Throttle;
	float motor3, motor3RPM, motor3Throttle, motor4, motor4RPM, motor4Throttle;
	float scale = 1;





void setMotor1(float throttle);
void setMotor2(float throttle);
void setMotor3(float throttle);
void setMotor4(float throttle);

CY_ISR(Uart)
{
	isr_ClearPending();
	*buffer = UART_GetChar();
	if(*buffer!=0)
	{
						
		if (recordFlag == 1)
		{
			sensor1[i] = *buffer;
			i++;
			if(*buffer == '!')
			{
				i = 0;
				dataReady = 1;
				memcpy(&sensor, &sensor1, sizeof(sensor));
				memset(sensor1, 0, sizeof(sensor1));
				recordFlag = 2;
			}
		}
		else if (recordFlag == 2)
		{
			sensor2[i] = *buffer;
			i++;
			if(*buffer == '!')
			{
				i = 0;
				dataReady = 1;
				memcpy(&sensor, &sensor2, sizeof(sensor));
				memset(sensor2, 0, sizeof(sensor2));
				recordFlag = 1;
			}
		}
		else //(recordFlag == 0)
		{
			memset(sensor1, 0, sizeof(sensor1));
			memset(sensor2, 0, sizeof(sensor2));
			if (*buffer == '!')
			{recordFlag = 1;}
		}
	}
}


int main()
{
   /* uint16 count;
	int counter = 0;
    char buffer[4],buffer1[4];
    char8 lineStr[20];
    uint8 state;
	int i = 0;
	char sensor[50];
	int recordFlag = 0;
	int dataReady = 0;
	int yaw_int,pitch_int,roll_int,P_int,Q_int,R_int;
	int phi_int,psi_int,theta_int;
	int motor1_int,motor2_int,motor3_int,motor4_int,motor1RPM_int,motor2RPM_int,motor3RPM_int,motor4RPM_int;
	char temp[6];
	char print_YPR[120];
	double yaw,pitch,roll,phi,theta,psi,P,Q,R;
	double yawsum=0,pitchsum=0,rollsum=0;
	double yaw_p = 0,pitch_p = 0,roll_p = 0;
	double freq = 34;
	double motor1, motor1RPM, motor1Throttle, motor2, motor2RPM, motor2Throttle;
	double motor3, motor3RPM, motor3Throttle, motor4, motor4RPM, motor4Throttle;
	//Gain matrix
	float MotorGain1[6] = { -0.0000,31.5951,100.9382,0.0000,22.3607,15.8114 };
	float MotorGain2[6] = { -31.5951,0.0000,-100.9382,-22.3607,0.0000,-15.8114 };
	float MotorGain3[6] = { 0.0000,-31.5951,100.9382,-0.0000,-22.3607,15.8114 };
	float MotorGain4[6] = { 31.5951,-0.0000,-100.9382,22.3607,-0.0000,-15.8114};
	//4s
	
	float MotorGain1[6] = { 0, 10.0788, 32.2686, 0, 2.2361,    1.5811};
	float MotorGain2[6] = { -10.0788,    0.0000,  -32.2686 ,  -2.2361 ,   0.0000 ,  -1.5811};
	float MotorGain3[6] = {  0.0000,  -10.0788 ,  32.2686  ,  0.0000 ,  -2.2361 ,   1.5811 };
	float MotorGain4[6] = {10.0788 ,  -0.0000 , -32.2686 ,   2.2361 ,  -0.0000 ,  -1.5811};
	//10s
	
	float MotorGain1[6] = { 0.0000,31.5167,100.9260,-0.0000,22.3607,15.8114 };
	float MotorGain2[6] = {-31.5167,-0.0000,-100.9260,-22.3607,-0.0000,-15.8114};
	float MotorGain3[6] = {-0.0000,-31.5167,100.9260,0.0000,-22.3607,15.8114};
	float MotorGain4[6] = {31.5167,0.0000,-100.9260,22.3607,0.0000,-15.8114};
	//7s 
	
	float MotorGain1[6] = {-0.0000,5.6488,17.9544,0.0000,0.7071,0.5000};
	float MotorGain2[6] = {-5.6488,0.0000,-17.9544,-0.7071,-0.0000,-0.5000};
	float MotorGain3[6] = {0.0000,-5.6488,17.9544,-0.0000,-0.7071,0.5000};
	float MotorGain4[6] = {5.6488,-0.0000,-17.9544,0.7071,0.0000,-0.5000 };
	//30s
	*/
	float MotorGain1[6] = {-0.0000,5.7539,18.1595,-0.0000,0.7071,0.5000};
	float MotorGain2[6] = {-5.7539,0.0000,-18.1595,-0.7071,-0.0000,-0.5000};
	float MotorGain3[6] = {0.0000,-5.7539,18.1595,0.0000,-0.7071,0.5000};
	float MotorGain4[6] = {5.7539,-0.0000,-18.1595,0.7071,0.0000,-0.5000};
	//20s
	
	/*
	float MotorGain1[6] = {-0.0000,17.7368,56.7570,0.0000,7.0711,5.0000};
	float MotorGain2[6] = {-17.7368,0.0000,-56.7570,-7.0711,0.0000,-5.0000};
	float MotorGain3[6] = {0.0000,-17.7368,56.7570,-0.0000,-7.0711,5.0000};
	float MotorGain4[6] = {17.7368,-0.0000,-56.7570,7.0711,-0.0000,-5.0000};
	//13s
	
	
	float MotorGain1[6] = {0.0000,10.2140,31.9547,-0.0000,2.2361,1.5811};
	float MotorGain2[6] = {-10.2140,-0.0000,-31.9547,-2.2361,-0.0000,-1.5811};
	float MotorGain3[6] = {-0.0000,-10.2140,31.9547,0.0000,-2.2361,1.5811};
	float MotorGain4[6] = {10.2140,0.0000,-31.9547,2.2361,0.0000,-1.5811};
	//11s
	
	float MotorGain1[6] = {-0.0000,9.9913,31.9195,-0.0000,2.2361,1.5811};
	float MotorGain2[6] = {-9.9913,0.0000,-31.9195,-2.2361,-0.0000,-1.5811};
	float MotorGain3[6] = {0.0000,-9.9913,31.9195,0.0000,-2.2361,1.5811};
	float MotorGain4[6] = {9.9913,-0.0000,-31.9195,2.2361,0.0000,-1.5811};
	// 10s
	
	float MotorGain1[6] = {0.0000,11.8940,37.9608,-0.0000,3.1623,2.2361};
	float MotorGain2[6] = {-11.8940,-0.0000,-37.9608,-3.1623,-0.0000,-2.2361};
	float MotorGain3[6] = {-0.0000,-11.8940,37.9608,0.0000,-3.1623,2.2361};
	float MotorGain4[6] = {11.8940,0.0000,-37.9608,3.1623,0.0000,-2.2361};
	//9s
	*/
	
	int HoverAngularVelocity = 4200;
    int cali=0;
	int caliy = 0;
    float calipitch=0, caliyaw=0, caliroll=0;
	int modelb = 1359;
	int modelcr = 73;
    int pp=0;
	int CommaPos = 0;
	
    /* Enable Global Interrupts */
    CyGlobalIntEnable;                      
	isr_StartEx(Uart);

    /* Start USBFS Operation with 3V operation */
    USBUART_1_Start(0u, USBUART_1_3V_OPERATION);
	UART_Start();

    count = 2;
	//Start PWM
	Motor1_Start();
   	Motor2_Start();
	Motor3_Start();
	Motor4_Start();
 
	CyDelay(5000);
	
    /* Main Loop: */
    for(;;)
    {
		//*buffer = UART_GetChar();

        if(USBUART_1_IsConfigurationChanged() != 0u) /* Host could send double SET_INTERFACE request */
        {
            if(USBUART_1_GetConfiguration() != 0u)   /* Init IN endpoints when device configured */
            {
                /* Enumeration is done, enable OUT endpoint for receive data from Host */
                USBUART_1_CDC_Init();
            }
        }         
        if(USBUART_1_GetConfiguration() != 0u)    /* Service USB CDC when device configured */
        {
            if(USBUART_1_DataIsReady() == 0u)               /* Check for input data from PC */
            {   
                if(count != 0u)
                {
						
                    while(USBUART_1_CDCIsReady() == 0u);    /* Wait till component is ready to send more data to the PC */ 
                   
					if(dataReady == 1)
					{
						dataReady = 0;//reset
						counter++;
						int j = 0;
						while (j<26)
						{
							int k = 0;
							//yaw obtain
							memset(temp, 0, sizeof(temp));
							while((k<7)&& (sensor[k]!=0x2C))//,
							{
								temp[k] =  (sensor[k]);
								k++;
								
							}
                            if (caliy==0) {caliyaw=atof(temp), caliy=1;}
							yaw = atof(temp)-caliyaw;
							yawave[9] = yawave[8];
							yawave[8] = yawave[7];
							yawave[7] = yawave[6];
							yawave[6] = yawave[5];
							yawave[5] = yawave[4];
							yawave[4] = yawave[3];
							yawave[3] = yawave[2];
							yawave[2] = yawave[1];
							yawave[1] = yawave[0];
							yawave[0] = yaw;
							yaw = (yawave[0] + yawave[1] + yawave[2]+ yawave[3]+ yawave[4]+ pitchave[5]+ pitchave[6]+ pitchave[7]+ pitchave[8]+ pitchave[9])/10;
							psi = (yaw - yaw_p)*freq;
							psiave[9] = psiave[8];
							psiave[8] = psiave[7];
							psiave[7] = psiave[6];
							psiave[6] = psiave[5];
							psiave[5] = psiave[4];
							psiave[4] = psiave[3];
							psiave[3] = psiave[2];
							psiave[2] = psiave[1];
							psiave[1] = psiave[0];
							psiave[0] = psi;
							psi = (psiave[0] + psiave[1] + psiave[2]+ psiave[3]+ psiave[4]+ psiave[5]+ psiave[6]+ psiave[7]+ psiave[8]+ psiave[9])/10;
							
	
							yaw_p = yaw;
							// pitch obtain
							
							//k is the pos of the first comma
							k++;
							CommaPos = k;// Comma's pos +1
							memset(temp, 0, sizeof(temp));
							while(k<18 && (sensor[k]!= 0x2C))
							{
								temp[k-CommaPos] = sensor[k];
								k++;
							}
                            if (cali==0) { calipitch=atof(temp); cali=1;}
							pitch = -1*(atof(temp)-calipitch);
							
							pitchave[9] = pitchave[8];
							pitchave[8] = pitchave[7];
							pitchave[7] = pitchave[6];
							pitchave[6] = pitchave[5];
							pitchave[5] = pitchave[4];
							pitchave[4] = pitchave[3];
							pitchave[3] = pitchave[2];
							pitchave[2] = pitchave[1];
							pitchave[1] = pitchave[0];
							pitchave[0] = pitch;
							pitch = (pitchave[0] + pitchave[1] + pitchave[2]+ pitchave[3]+ pitchave[4]+ pitchave[5]+ pitchave[6]+ pitchave[7]+ pitchave[8]+ pitchave[9])/10;
							
							theta = (pitch - pitch_p)*freq;
							
							thetaave[9] = thetaave[8];
							thetaave[8] = thetaave[7];
							thetaave[7] = thetaave[6];
							thetaave[6] = thetaave[5];
							thetaave[5] = thetaave[4];
							thetaave[4] = thetaave[3];
							thetaave[3] = thetaave[2];
							thetaave[2] = thetaave[1];
							thetaave[1] = thetaave[0];
							thetaave[0] = theta;
							theta = (thetaave[0] + thetaave[1] + thetaave[2]+ thetaave[3]+ thetaave[4]+ thetaave[5]+ thetaave[6]+ thetaave[7]+ thetaave[8]+ thetaave[9])/10;
								
								
							pitch_p = pitch;
							// roll obtain
							k++;
							CommaPos = k;
							memset(temp, 0, sizeof(temp));
							while( k<26 && (sensor[k]!=0x21))
							{
								temp[k-CommaPos] = sensor[k];
								k++;
							}
							if (cali==1) {caliroll=atof(temp); cali=2;}
                            roll = atof(temp)-caliroll;

							rollave[9] = rollave[8];
							rollave[8] = rollave[7];
							rollave[7] = rollave[6];
							rollave[6] = rollave[5];
							rollave[5] = rollave[4];
							rollave[4] = rollave[3];
							rollave[3] = rollave[2];
							rollave[2] = rollave[1];
							rollave[1] = rollave[0];
							rollave[0] = roll;
							roll = (rollave[0] + rollave[1] + rollave[2]+rollave[3]+rollave[4]+rollave[5]+rollave[6]+rollave[7]+rollave[8]+rollave[9])/10;
							
                            phi = (roll - roll_p)*freq;
							
							phiave[9] = phiave[8];
							phiave[8] = phiave[7];
							phiave[7] = phiave[6];
							phiave[6] = phiave[5];
							phiave[5] = phiave[4];
							phiave[4] = phiave[3];
							phiave[3] = phiave[2];
							phiave[2] = phiave[1];
							phiave[1] = phiave[0];
							phiave[0] = phi;
							phi = (phiave[0] + phiave[1] + phiave[2]+ phiave[3]+ phiave[4]+ phiave[5]+ phiave[6]+ phiave[7]+ phiave[8]+ phiave[9])/10;
								
							P = phi;
							Q = theta;
							R = psi;
							roll_p = roll;
							memset(temp, 0, sizeof(temp));//reset the buffer
							k=0;
							break;
							j++;
						}
						// convert float PQR,YPR  to int for UART output
						pp=pp+1;
						yaw_int=(int)floor(yaw*100);
						psi_int=(int)floor(psi*100);
						pitch_int=(int)floor(pitch*100);
						theta_int=(int)floor(theta*100);
						roll_int=(int)floor(roll*100);
						phi_int=(int)floor(phi*100);
						P_int=(int)floor(P*100);
						Q_int=(int)floor(Q*100);
						R_int=(int)floor(R*100);						
					
						// Algorithm begins here
						
						//Compute motor input 
						motor1 = roll*MotorGain1[3] + pitch*MotorGain1[4] + yaw*MotorGain1[5] + P*MotorGain1[0] + Q*MotorGain1[1] + R*MotorGain1[2];
						motor2 = roll*MotorGain2[3] + pitch*MotorGain2[4] + yaw*MotorGain2[5] + P*MotorGain2[0] + Q*MotorGain2[1] + R*MotorGain2[2];
						motor3 = roll*MotorGain3[3] + pitch*MotorGain3[4] + yaw*MotorGain3[5] + P*MotorGain3[0] + Q*MotorGain3[1] + R*MotorGain3[2];
						motor4 = roll*MotorGain4[3] + pitch*MotorGain4[4] + yaw*MotorGain4[5] + P*MotorGain4[0] + Q*MotorGain4[1] + R*MotorGain4[2];
	
						// low and high limit of the throttle in testing
						int limit = 100;//100 is no limit
						int lower = 0;//0 is no limit
						//Calculate motor throttle and RPM
						motor1RPM = motor1*scale + HoverAngularVelocity;
						motor1Throttle = (motor1RPM - modelb) / modelcr;
						if(motor1Throttle>limit)
						{
							motor1Throttle = limit;
						}
						else if(motor1Throttle<=lower)
						{
							motor1Throttle = lower;
						}
						motor2RPM = motor2*scale + HoverAngularVelocity;
						motor2Throttle = (motor2RPM - modelb) / modelcr;
						if(motor2Throttle>limit)
						{
							motor2Throttle = limit;
	
						}
						else if(motor2Throttle<=lower)
						{
							motor2Throttle = lower;
						}
						motor3RPM = motor3*scale + HoverAngularVelocity;
						motor3Throttle = (motor3RPM - modelb) / modelcr;
						if(motor3Throttle>limit)
						{
							motor3Throttle = limit;
						}
						else if(motor3Throttle<=lower)
						{
							motor3Throttle = lower;
						}
						motor4RPM = motor4*scale + HoverAngularVelocity;
						motor4Throttle = (motor4RPM - modelb) / modelcr;
						if(motor4Throttle>limit)
						{
							motor4Throttle = limit;
						}
						else if(motor4Throttle<=lower)
						{
							motor4Throttle = lower;
						}
						// Startup process, stepping up
						if(counter<= 150)
						{
		
						setMotor1(25);
						setMotor2(25);
						setMotor3(25);
						setMotor4(25);
						}
						else if(counter<300)
						{
							setMotor1(32);
							setMotor2(32);
							setMotor3(32);
							setMotor4(32);
						}
						else if(counter<450)
						{
							setMotor1(41);
							setMotor2(41);
							setMotor3(41);
							setMotor4(41);
						}	
						else if(counter ==450)
						{
							caliy = 0;//recalibrate yaw
				
						}
					
						else
						{
							setMotor1(motor1Throttle);
							setMotor2(motor2Throttle);
							setMotor3(motor3Throttle*1.05);
							setMotor4(motor4Throttle);
						
						}
					
					
						
						motor1_int=(int)floor(motor1);
						motor2_int=(int)floor(motor2);
						motor3_int=(int)floor(motor3);
						motor4_int=(int)floor(motor4);
						motor1RPM_int=(int)floor(motor1RPM);
						motor2RPM_int=(int)floor(motor2RPM);
						motor3RPM_int=(int)floor(motor3RPM);
						motor4RPM_int=(int)floor(motor4RPM);

					//	sprintf(print_YPR," yaw:%d pitch:%d roll:%d\t R:%d Q:%d P:%d\tM1:%dM2:%dM3:%dM4:%d\tM1RPM:%dM2RPM:%dM3RPM:%dM4RPM:%d\tcount:%d\n\r",yaw_int,pitch_int,roll_int,R_int,Q_int,P_int,motor1_int,motor2_int,motor3_int,motor4_int,motor1RPM_int,motor2RPM_int,motor3RPM_int,motor4RPM_int,pp);
						//sprintf(print_YPR," yaw:%d pitch:%d roll:%d count:%d\n\r",yaw_int,pitch_int,roll_int,pp);
						sprintf(print_YPR," yaw:%d pitch:%d roll:%d\t R:%d Q:%d P:%d\tM1:%dM2:%dM3:%dM4:%d\tcount:%d\n\r",yaw_int,pitch_int,roll_int,R_int,Q_int,P_int,motor1_int,motor2_int,motor3_int,motor4_int,pp);

						USBUART_1_PutString(print_YPR);
					}
					
                    /* If the last sent packet is exactly maximum packet size, 
                    *  it shall be followed by a zero-length packet to assure the
                    *  end of segment is properly identified by the terminal.
                    */
                    if(count == BUFFER_LEN)
                    {
                        while(USBUART_1_CDCIsReady() == 0u); /* Wait till component is ready to send more data to the PC */ 
                        USBUART_1_PutData(NULL, 0u);         /* Send zero-length packet to PC */
                    }
                }
            }  
            
            
        }
    }   
}
void setMotor1(float throttle)
{
	// PWM at PSOC is clocked to 500kHz,Period = 10000, producing a signal with Freq=50Hz, 0% throttle maps to 5% PWM, 100% Throttle to 10% PWM
	int PWM =  500 +  (int) (throttle*5);
	Motor1_WriteCompare(PWM);
}


void setMotor2(float throttle)
{
	// PWM at PSOC is clocked to 500kHz,Period = 10000, producing a signal with Freq=50Hz, 0% throttle maps to 5% PWM, 100% Throttle to 10% PWM
	int PWM =  500 +  (int) (throttle*5);
	Motor2_WriteCompare(PWM);
}
void setMotor3(float throttle)
{
	// PWM at PSOC is clocked to 500kHz,Period = 10000, producing a signal with Freq=50Hz, 0% throttle maps to 5% PWM, 100% Throttle to 10% PWM
	int PWM =  500 +  (int) (throttle*5);
	Motor3_WriteCompare(PWM);
}

void setMotor4(float throttle)
{
	// PWM at PSOC is clocked to 500kHz,Period = 10000, producing a signal with Freq=50Hz, 0% throttle maps to 5% PWM, 100% Throttle to 10% PWM
	int PWM =  500 +  (int) (throttle*5);
	Motor4_WriteCompare(PWM);
}

/* [] END OF FILE */
