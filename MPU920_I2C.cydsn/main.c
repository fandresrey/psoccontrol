/* ========================================
 *
 * @brief Main file for MPU9250 project.
 *
 * This is the main file to be used with the
 * MPU9250 project. It sets up all the
 * components required for the project.
 *
 * @author Davide Marzorati
 * @date 29 March, 2019
 * ========================================
 */

// Include required header files
#include "project.h"

#include <math.h>

#include "stdio.h"

#if defined(__GNUC__)
/* Add an explicit reference to the floating point printf library */
/* to allow usage of the floating point conversion specifiers. */
/* This is not linked in by default with the newlib-nano library. */
asm(".global _printf_float");
#endif
#define SAMPLE_RATE 500
#define FC 5
#define USBFS_DEVICE (0u)

#define ORDER 140
/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
 * endpoints.
 */
#define USBUART_BUFFER_SIZE (64u)
#define LINE_STR_LENGTH (20u)

float a0, a1, a2, b1, b2, w0, alpha, cosw0, outputVoltage;
float x1 = 0, x2 = 0, yy1 = 0, yy2 = 0;
char8 * parity[] = {
  "None",
  "Odd",
  "Even",
  "Mark",
  "Space"
};
char8 * stop[] = {
  "1",
  "1.5",
  "2"
};
char message[100]; // Message to send over UART
int16 ACCx, ACCy, ACCz;
int led;
float setpoint = -2100; // Valor deseado

float In_ADC; // Valor actual
float DAC_out = 0; // Valor salida
float Out_Control = 0; // Señal de control
float error = 0; // Error actual
float last_error = 0; // Error pasado
float Ant_error = 0; // Error anterior
float D_error = 0; // Error anterior
float Proporcional = 0; // Ganancia proporcial al error
float integral = 0; // Suma de errores
float derivative = 0; // Cambio en el error
float cambio = 0; //variable para tomar muestas 
float last = 0; //asegurar muestra anterior
float now; //asegurar muestra ahora
float Valor_Dac = 0; //asegurar muestra ahora
float sensor;

// Parámetros del controlador
float Kp = 6;
float Ki = 5;
float Kd = 1;
float T = 10;

CY_ISR_PROTO(MPU9250_DR_ISR);
void inicioVolante(){
    int salida = 1;
    int ienc,f=0;
    
    while(salida){
      
            ienc = QuadDec_GetCounter();
      sprintf(message, "enc:%i", ienc); 
     UART_Debug_PutString(message);
    UART_Debug_PutString("\r\n");
              CyDelay(50);
            
            
            
                    if( hall_Read()){
            salida = 0;
            QuadDec_SetCounter(0);
                     left_Write(0);
            rigth_Write(0);
            PWM_1_WriteCompare(0);
            break;
            }else{
            
            }
            
            
            while(ienc <=500){
             left_Write(0);
            rigth_Write(1);
            PWM_1_WriteCompare(50);
                        ienc = QuadDec_GetCounter();
                  sprintf(message, "enc:%i", ienc); 
                 UART_Debug_PutString(message);
                UART_Debug_PutString("\r\n");
      
                   
            if( hall_Read()){
           CyDelay(100);
            QuadDec_SetCounter(0);
                     left_Write(0);
            rigth_Write(0);
            PWM_1_WriteCompare(0);
            ienc = 501 ;
            f=1;
            }else{
            
            }

            
            }
            
            
            
           while(f==0){
             left_Write(1);
            rigth_Write(0);
            PWM_1_WriteCompare(50);
                        ienc = QuadDec_GetCounter();
                  sprintf(message, "enc:%i", ienc); 
                 UART_Debug_PutString(message);
                UART_Debug_PutString("\r\n");
      
                   
            if( hall_Read()){
             CyDelay(800);
            QuadDec_SetCounter(0);
                     left_Write(0);
            rigth_Write(0);
            PWM_1_WriteCompare(0);
          
            f = 1 ;
            }else{
            
            }

            
            }
            
        
        
        
            
                   left_Write(0);
            rigth_Write(0);
            PWM_1_WriteCompare(45);
            
            
            
            
            
        
             
         
            
            
    

    }
    



}
void calculate_output() {

  // Cálculo del error apartir del tiempo de muestreo

  cambio = now - last;
  /// calculo control proporcional

  error = setpoint - outputVoltage;
  last = now; //actualizamos tiempde muestreo
  Proporcional = Kp * error;
  // Cálculo de la integral

  last_error = (error * T) + last_error;
  integral = Ki * last_error;
  // Limitar el valor de la integral para evitar la acumulación de errores
  if (integral > 1 / Ki) {
    integral = 1 / Ki;
  } else if (integral < -1 / Ki) {
    integral = -1 / Ki;
  }
  // Cálculo del derivativo
  D_error = ((error - Ant_error) / T);
  derivative = Kd * D_error;
  // Actualización del error anterior
  Ant_error = error;
  // Cálculo de la salida del controlador
  Out_Control = Proporcional + integral + derivative;

  /* 
       
    
    // Limitar el valor de la salida a la escala del DAC
    if (Out_Control > 4.08) {
        Out_Control = 4.08;
    } else if (Out_Control < -4.08) {
        Out_Control = -4.08;
    }
  
   
  */
}

int main(void) {
  CyGlobalIntEnable; /* Enable global interrupts. */

  left_Write(0);
  rigth_Write(0);
  a0 = 0.5;

  float inputSignal; // Señal de entrada
  float outputSignal; // Señal de salida

  // Coeficientes del filtro FIR
  float coefficients[ORDER + 1] = {
    0.00044104,
    0.00046007,
    0.0004848,
    0.00051576,
    0.0005535,
    0.00059852,
    0.00065132,
    0.00071236,
    0.00078211,
    0.00086097,
    0.00094933,
    0.001,
    0.0012,
    0.0013,
    0.0014,
    0.0015,
    0.0017,
    0.0019,
    0.002,
    0.0022,
    0.0024,
    0.0026,
    0.0028,
    0.0031,
    0.0033,
    0.0036,
    0.0038,
    0.0041,
    0.0044,
    0.0047,
    0.005,
    0.0053,
    0.0056,
    0.0059,
    0.0062,
    0.0066,
    0.0069,
    0.0072,
    0.0076,
    0.0079,
    0.0083,
    0.0086,
    0.009,
    0.0093,
    0.0096,
    0.01,
    0.0103,
    0.0106,
    0.011,
    0.0113,
    0.0116,
    0.0119,
    0.0122,
    0.0125,
    0.0127,
    0.013,
    0.0132,
    0.0135,
    0.0137,
    0.0139,
    0.0141,
    0.0142,
    0.0144,
    0.0145,
    0.0147,
    0.0148,
    0.0148,
    0.0149,
    0.0149,
    0.015,
    0.015,
    0.015,
    0.0149,
    0.0149,
    0.0148,
    0.0148,
    0.0147,
    0.0145,
    0.0144,
    0.0142,
    0.0141,
    0.0139,
    0.0137,
    0.0135,
    0.0132,
    0.013,
    0.0127,
    0.0125,
    0.0122,
    0.0119,
    0.0116,
    0.0113,
    0.011,
    0.0106,
    0.0103,
    0.01,
    0.0096,
    0.0093,
    0.009,
    0.0086,
    0.0083,
    0.0079,
    0.0076,
    0.0072,
    0.0069,
    0.0066,
    0.0062,
    0.0059,
    0.0056,
    0.0053,
    0.005,
    0.0047,
    0.0044,
    0.0041,
    0.0038,
    0.0036,
    0.0033,
    0.0031,
    0.0028,
    0.0026,
    0.0024,
    0.0022,
    0.002,
    0.0019,
    0.0017,
    0.0015,
    0.0014,
    0.0013,
    0.0012,
    0.001,
    0.00094933,
    0.00086097,
    0.00078211,
    0.00071236,
    0.00065132,
    0.00059852,
    0.0005535,
    0.00051576,
    0.0004848,
    0.00046007,
    0.00044104
  };

  float delayLine[ORDER + 1] = {
    0
  };
  uint16 count;
  uint8 buffer[USBUART_BUFFER_SIZE];
  // Start UART component
  UART_Debug_Start();

  CyDelay(500);

  int test;

  // Read WHO AM I register and compare with the expected value

  UART_Debug_PutString(message);

  sprintf(message, "DATA: %.4f", cosw0);
  UART_Debug_PutString("\r\n");

  UART_Debug_PutString(message);
  PWM_1_Start();
  ADC_SAR_Seq_1_Start();
  ADC_SAR_Seq_1_StartConvert();
  QuadDec_Start();
 
  int enc = 0;
  CyDelay(500);

inicioVolante();
  CyDelay(1000);
setpoint=-2;
int test3;

  for (;;) {
    
    
    // while(!ADC_SAR_Seq_1_IsEndConversion(ADC_SAR_Seq_1_RETURN_STATUS));

    /* Get the ADC results for each channel */
    int16_t adcResult0 = ADC_SAR_Seq_1_GetResult16(0) - 260;
    int16_t adcResult1 = ADC_SAR_Seq_1_GetResult16(1) - 260;
    int16_t adcResult2 = ADC_SAR_Seq_1_GetResult16(2) - 310;

    outputSignal = 0;
    delayLine[0] = adcResult1;

    for (int i = 0; i <= ORDER; i++) {
      outputSignal += coefficients[i] * delayLine[i];
    }

    for (int i = ORDER; i > 0; i--) {
      delayLine[i] = delayLine[i - 1];
    }
    int test2 = (outputSignal);
    outputVoltage = outputSignal;
    calculate_output();
    
    test = Out_Control;
   enc = QuadDec_GetCounter();

test3=Out_Control;
    sprintf(message, "AY: %i AZ: %i fir:%i crl:%i enc:%i pwm: %i", 0, 0, test2,test,enc,test3);
    UART_Debug_PutString("\r\n");
    UART_Debug_PutString(message);
    if(enc<350 && enc>-400){
    
      if(Out_Control >= 0){

            left_Write(0);
            rigth_Write(1);


            PWM_1_WriteCompare(fabs(Out_Control)+27);
            CyDelay(1);


            }else{

            left_Write(1);
            rigth_Write(0);
            PWM_1_WriteCompare(fabs(Out_Control)+37);
            CyDelay(1);
            }
    
    }else{
    PWM_1_WriteCompare(0);
    inicioVolante();
    CyDelay(2000);
    }
           

    if (led == 1) {
      led = 0;
      Connection_Led_Write(1);
    } else {
      led = 1;
      Connection_Led_Write(0);
    }
    CyDelayUs(1455);

  }
}

//CY_ISR(MPU9250_DR_ISR)
//{
//
//    if (led == 1)
//    {
//        led = 0;
//        Connection_Led_Write(1);
//    }
//    else
//    {
//        led = 1;
//        Connection_Led_Write(0);
//    }
//
//    // prepare packet
//    static uint8_t data[16];
//    // header
//    CyDelay(1);
//    // read data
//    data[0] = 0xA0;
//    // footer
//    data[13] = 0xC0;
//
//    ACCx = (data[1] << 8) | (data[2]);
//    ACCy = (data[3] << 8) | (data[4]);
//    ACCz = (data[5] << 8) | (data[6]);
//
//    float inputVoltage = ACCy;
//a0=0.08;
//    // Convertir la muestra de entrada a voltaje
// 
//
//    // Calcular la muestra de salida del filtro
//     outputVoltage = (a0 * inputVoltage) + ((1-a0)*outputVoltage);
//    
//    
//calculate_output();
//            
//            
//
//    
//    
//
//
//
//
//    // Escribir la muestra de salida en el DAC
//    int16 out;
//    out = (int16)outputVoltage;
//    data[14] = out;
//    data[15] = (uint8)(out >> 8);
//    // send packet over uart
//int test = (int)round(Out_Control);
//
//    if(Out_Control>65000){
//    Out_Control=65000;
//    }else if(Out_Control<-65000){
//    Out_Control=-65000;
//    }
//

//
//           sprintf(message, "DATAF: %i DATA: %i PID: %i", out,ACCy,test);
//         UART_Debug_PutString("\r\n");
//
//     UART_Debug_PutString(message);
//    
//
//   // USBUART_PutData(data, 16);
//}

/* [] END OF FILE */