#include <18F14K22.h>
#device ADC=10
//*******************************Fuses***************************************\\
//***************************************************************************\\
#FUSES NOWDT                    //No Watch Dog Timer
#FUSES PUT                      //Power Up Timer
#FUSES NOPLLEN                  //
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES NOIESO 
#FUSES MCLR
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NODEBUG                  //No Debug mode for ICD
#FUSES NOBROWNOUT               //No brownout reset
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NOWRT                    //Program memory not write protected
#FUSES RESERVED                 //Used to set the reserved FUSE bits
#fuses INTRC          //Internal RC Osc

#use delay(internal=8000000)
#use rs232(baud=38400,parity=N,xmit=PIN_B7,rcv=PIN_B5,bits=8,stream=RS485,ERRORS) //Comunicacion con Wireless Board
#use rs232(baud=38400,parity=N,xmit=PIN_A0,rcv=PIN_A1,bits=8,stream=Pickit,ERRORS) //Comunicacion con Pickit para configuracion
#use rs232(baud=9600,parity=N,xmit=PIN_C0,rcv=PIN_A2,bits=8,stream=Sensor,ERRORS) //Comunicacion con Pickit para configuracion

#priority rda,ext1,ext2,timer1

#define Led_Status     PIN_C5
#define DRV_485        PIN_B6
#define DRV_Sensor     PIN_B4
#define Pin_Flotador   PIN_C2
#define Relay_Izq      PIN_C5
#define Relay_Der      PIN_C4
//*********************************Variables*********************************\\
//***************************************************************************\\
int8 Blink=0;
int8 Tiempo=0;
int8 Blink_Flotador=0;
int8 Tiempo_Blink=0;
int8 Tiempo_Envio_CMD;  //Tiempo Para el envio del comando status
int8 Num_Serie1;        //Numero de serie de la tarjeta 1er Byte
int8 Metros_Cubicos_Cisterna=0XFE; //sin uso
int8 Numero_de_Cisterna=0XFE;       //sin uso

//*********************Variables para calculos de cisternas******************\\
// Las variables de 8 bits se almacenan en EEprom del micro,
// Las variables de 16 bits se utilizan para realizar los calculos

int16 Distancia_Medida=0;
int8 Distancia_Medida_High=0;
int8 Distancia_Medida_Low=0;
int16 Distancia=0;         //se utiliza para pasar el valor de distancia medida de 16 bits a 2 variables de 8 bits
//la medida del sensor llega en 4 bytes, que indican la distancia milimetros de 300 a 4999
//Las siguientes variables se utilizaron para juntas los 4 bytes para lograr la distancia final en una variable de 16 bits Distancia_Medida
int16 Medida_1;    
int16 Medida_2;
int16 Medida_3;
int16 Medida_4;

int16 Distancia_Vacio;
int8  Distancia_Vacio_High;
int8  Distancia_Vacio_Low;

int16 Distancia_lleno;
int8  Distancia_Lleno_High=0X01; //0X012C = 300 = 30 cm = Distancia minima que es capaz de sensar el ultrasonico
int8  Distancia_Lleno_Low=0X2C;

//***************************************************************************\\
int8 Flotador=0;        //Estado del flotador
int8 Respuesta;         //Bytes de Respuesta de algun CMD recibido, 00= Rechazado, 01= Aceptado
int8 CMD_Ejecutado;     //indica el comando del cual se esta dando respuesta
int8 Segundos=0;        //Temporizador
int8 Estado_Relays=0;   //Byte Para manipular el estado de los relays on board

int32 Calculo1=0;  
int32 Calculo2=0;
int8 Porcentaje=0;
//****************************Variables com serial***************************\\
//int8 Timeout=0;
int8 Char_Recibido_Pickit=0;
int8 Char_Recibido_RS485=0;
int8 Indice_Pickit=0;
int8 Indice_RS485=0;
int8 Indice_Sensor=0;
char Pickitbuff[30];
char RS485buff[30];
char Sensorbuff[10];
int1 F_CMD_Completo_Pickit=0;
int1 F_CMD_Completo_RS485=0;
int1 F_CMD_Completo_Sensor=0;

//*****************************Interrupcion Timer 1**************************\\
// Cada 100 ms
#INT_TIMER1
void  TIMER1_isr(void) 
{
   set_timer1(28036);
   Blink++;


   if(Blink>=10)
   {
      Blink=0;
      output_toggle(Led_Status);
   }
}

//***************Interrupción por serial del pickit***************************\\
#INT_EXT1   
void  EXT1_isr(void) 
{
   Char_Recibido_Pickit=0x00;
  if(kbhit(pickit))
   {
     Char_Recibido_Pickit=fgetc(pickit);                
     Pickitbuff[Indice_Pickit++]=Char_Recibido_Pickit; 
     
     if(Indice_Pickit>29)
     {
         Indice_Pickit=0;
     }
       if (Char_Recibido_Pickit==0X3F)  
         {
            F_CMD_Completo_Pickit=1;
            Tiempo_Blink=1; 
            Blink_Flotador=0;;
            Tiempo=0;
         }                                                                                                               
   }
}
//***************Interrupción por serial del pickit***************************\\
#INT_EXT2   
void  EXT2_isr(void) 
{
  if(kbhit(Sensor))
   {
     Sensorbuff[Indice_Sensor++]=fgetc(Sensor);                
     if(Indice_Sensor>9)
     {
         Indice_Sensor=0;
     }
     if (Sensorbuff[Indice_Sensor-1]==0X0D)  
     {
         F_CMD_Completo_Sensor=1;
         Indice_Sensor=0;
     }  
   }
}
//************************Interrupcion serial 485***************************\\
#INT_RDA
void  RDA_isr(void) 
{
   Char_Recibido_RS485=0x00;
  if(kbhit(RS485))
   {
     Char_Recibido_RS485=fgetc(RS485);                // lo descargo
     RS485buff[Indice_RS485++]=Char_Recibido_RS485;   // lo añado al buffer
     
     if(Indice_RS485>29)
     {
         Indice_RS485=0;
     }
      if (Char_Recibido_RS485==0X3F)  
         {
            F_CMD_Completo_RS485=1;
            Tiempo_Blink=1; 
            Blink_Flotador=0;
            Tiempo=0;
         }                                                                                                          
   }  
}

void Leer_Bytes_de_Config(void)
{
   delay_ms(1);     
   Tiempo_Envio_CMD = read_eeprom (0);
   Num_Serie1 = read_eeprom (1);    //En la localidad 1 se encuentra el 1er byte del numero de serie
   Estado_Relays = read_eeprom(2);
  
   Distancia_Vacio_High = read_eeprom (16);
   Distancia_Vacio_Low = read_eeprom(17);
   Distancia_Lleno_High = read_eeprom (18);
   Distancia_Lleno_Low = read_eeprom (19);

// Convierto variables int8 a int16
   Distancia_Vacio = (Distancia_Vacio_High<<8);
   Distancia_Vacio += Distancia_Vacio_Low;    
   Distancia_lleno = (Distancia_Lleno_High<<8);
   Distancia_lleno += Distancia_Lleno_Low;   
}

//****************Funcion que envia el estado por el Pickit******************\\
void Envia_Estado_Pickit(void)
{
       /*fputc(0X23,Pickit);
       fputc(0X5E,Pickit);
       //fputc(Num_Serie1,Pickit);
       //fputc(0X00,pickit);
       //fputc(Estado_Relays,Pickit);
       //fputc(Flotador,Pickit);
       fputc(Distancia_Medida_High,Pickit);
       fputc(Distancia_Medida_Low,Pickit);
       fputc(Distancia_Vacio_High,Pickit);
       fputc(Distancia_Vacio_Low,Pickit);
       fputc(Distancia_Lleno_High,Pickit);
       fputc(Distancia_Lleno_Low,Pickit);
       fputc(Porcentaje,Pickit);
       //fputc(Metros_Cubicos_Cisterna,Pickit);
       //fputc(Numero_de_Cisterna,Pickit);
       fputc(0X3C,Pickit);
       fputc(0X3F,Pickit);
       */
       fprintf(Pickit,"Distancia medida: %05.3w m.\r\nDistancia vacio: %5.3w m.\r\nDistancia LLeno: %5.3w m.\r\nPorcentaje: %u %%\r\n",Distancia,Distancia_Vacio,DIstancia_Lleno,Porcentaje);
}

//*****************Funcion que envia el estado por el 485*******************\\
void Envia_Estado_RS485(void)
{
       output_high(DRV_485);                                     //Envia   rs-485
       delay_ms(1);
       fputc(0X23,RS485);
       fputc(0X5E,RS485);
       fputc(Num_Serie1,RS485);
       fputc(0X00,RS485);
       fputc(Estado_Relays,RS485);
       fputc(Flotador,RS485);
       fputc(Distancia_Medida_High,RS485);
       fputc(Distancia_Medida_Low,RS485);
       fputc(Distancia_Vacio_High,RS485);
       fputc(Distancia_Vacio_Low,RS485);
       fputc(Distancia_Lleno_High,RS485);
       fputc(Distancia_Lleno_Low,RS485);
       fputc(Porcentaje,RS485);
       fputc(Metros_Cubicos_Cisterna,RS485);
       fputc(Numero_de_Cisterna,RS485);
       fputc(0X3C,RS485);
       fputc(0X3F,RS485);
       delay_ms(3);
       output_low(DRV_485);
       
}

//**************Funcion para cambiar el estado de los relays*****************\\
void Actualiza_Estado_Relays(void)
{
   switch(Estado_Relays)
   {
      case 0X00:
            output_low(Relay_Izq);
            output_low(Relay_Der);
            break;
      case 0x01:
            output_low(Relay_Izq);
            output_high(Relay_Der);
            break;
      case 0x10:
            output_high(Relay_Izq);
            output_low(Relay_Der);
            break;
      case 0x11:
            output_high(Relay_Izq);
            output_high(Relay_Der);
            break;
      default:
            output_low(Relay_Izq);
            output_low(Relay_Der);
   }        
}
//*******************Verifica el estado del flotador*************************\\
void Estado_Flotador(void)
{
//    if(input(Pin_Flotador))  == flotador NC
//    if(!input(Pin_Flotador)) == flotador NA

      if(input(Pin_Flotador))
      {
         Flotador=0x01;          
         //Tiempo_Blink=1;         //Blink cada 200 ms led status
      }
      else 
      {
         Flotador=0x00;
         //Tiempo_Blink=10;     //Blink cada segundo led status
      } 
}
//****************************Lee el estado del ADC**************************\\
//activa el pin DRV_Sensor, para avisarle al sensor ultrasonico que realice una medicion,
//La funcion de interrupcion externa 2 se encarga de recibir el dato enviado por el sensor
void Lee_Distancia(void)      //Lee_ADC
{
   Output_high(DRV_Sensor);
   //fputc(0XAA,Pickit);
   
}

//****************Calcula el porcentaje de agua en cisterna******************\\
//Si se han configurado el valor del adc con cisterna llena y vacia, se puede realizar el
//calculo del porcentaje del valor leido.

void Calcula_Nivel(void)
{   
    if(Distancia_Vacio_High != 0XFF && Distancia_Vacio_Low != 0XFF) //si estan configurados los varores del adc vacio y lleno
    {    
         if(Distancia_Medida >= Distancia_Lleno && Distancia_Medida < Distancia_Vacio)   //si esta dentro del rango configurado
         {
            Calculo1 = (( int32)(Distancia_Vacio -Distancia_Medida)*(int32)(0x64));
            Calculo2 = (( int32)(Distancia_Vacio - Distancia_Lleno));
            Porcentaje = (int8 )(Calculo1/Calculo2);
         }
         else if(Distancia_Medida > Distancia_Vacio)     //Si la distancia medida es mayor a la maxima configurada para Distancia vacia, no ajusta el valor solo manda procentaje 0
           {
               if(Distancia_Medida>0X1380)         //1380h corresponde a el valor de 4.9 mts (4900 mm) de profundidad, ya que cuando el nivel de agua supera al sensor, manda medida maxima de 1388 = 5 m
               {                                   //Por lo que al superar el nivel de agua al sensor , este marcaba 0 %. Cuando estaba repletita
                  Porcentaje=0x64;
               }
               else                                
               {
                  Porcentaje=0X01;
               }
           }
         else if(Distancia_Medida<Distancia_Lleno)
            {
               Porcentaje=0x64;
            }
    }
}

//*****************Funcion que atiende las temporizaciones*******************\\
void Temporizaciones(void)
{

 
}
//****************************************************************************\\
void Verifica_CMD_Pickit(void)
{
   if(F_CMD_Completo_Pickit==1)
   {
      F_CMD_Completo_Pickit=0;
      
       if(Pickitbuff[0]==0X23)      //Tiene llave de inicio 1er byte
       {
         if(Pickitbuff[1]==0X5E)    //Tiene llave de inicio 2do byte
         {
             if(Pickitbuff[2]==0X02) //Es CMD para la wireless Board, solo reeenvia por el aurt RS485
            {
                        output_high(DRV_485);                                     //Envia   rs-485
                        delay_ms(1);
                       for(int i=0;i<Indice_Pickit;i++)
                       {
                           fputc(Pickitbuff[i],RS485);   
                       }
                       delay_ms(3);
                        output_low(DRV_485); 
                       //Indice_Pickit=0;
                       break;
            }
            if(Pickitbuff[2]==0X01) //Es CMD para sensor Board, procede a verificar el comando.
            {
                    // if(Pickitbuff[3] == Num_Serie1)     
                       switch(Pickitbuff[4])  // verifica el byte de comandos [3]
                       {
                       
                           case 0:        //Solicitud del estado del sistema
                                 {
                                 
                                    if(Pickitbuff[5]==0X3C && Pickitbuff[6]==0X3F)
                                    {
                                       Envia_Estado_Pickit();
                                       goto Salida;
                                    }
                                 }
                                 break;
                           case 1:  //Comando para modificar el tiempo entre comandos de status
                                 {
                                   if(Pickitbuff[6]==0X3C && Pickitbuff[7]==0X3F)
                                   {
                                       if(Pickitbuff[5]<0xF0) //0x019= 25 en decimal, multiplicado por 10=250 
                                       {
                                          write_eeprom(0,(Pickitbuff[5]));// el dato recibido esta expresado en segundos 
                                          delay_ms(1);
                                          Tiempo_Envio_CMD = Pickitbuff[5];
                                          Tiempo=0;
                                          Segundos=0;
                                          Respuesta=1;
                                          CMD_Ejecutado=1;
                                       //goto Respuesta_CMD;
                                       }
                                    else
                                       {   
                                        CMD_Ejecutado=1;
                                          Respuesta=0;
                                       //goto Respuesta_CMD;
                                       }
                                   }
                                 }
                                 break;
                            case 2:  //Comando para activar relays onboard
                                 {
                                 if(Pickitbuff[6]==0X3C && Pickitbuff[7]==0X3F)
                                   {
                                       CMD_Ejecutado=2;
                                       Respuesta=0;
                                       if(Pickitbuff[5]==0x00)
                                       {
                                          Estado_Relays=0X00;
                                          Respuesta=1;
                                           write_eeprom(2,0X00);
                                       }
                                       if(Pickitbuff[5]==0X10)
                                       {
                                          Estado_Relays=0X10;
                                          Respuesta=1;
                                           write_eeprom(2,0X10);
                                       }
                                       if(Pickitbuff[5]==0X01)
                                       {
                                          Estado_Relays=0X01;
                                          Respuesta=1;
                                           write_eeprom(2,0X01);
                                       }
                                       if(Pickitbuff[5]==0X11)
                                       {
                                          Estado_Relays=0X11;
                                          Respuesta=1;
                                           write_eeprom(2,0X11);
                                       }
                                    }
                                 }
                                 break;
                                 
                             case 3:  //Comando para establecer el numero de serie
                                {
                                    if(Pickitbuff[6]==0X3C && Pickitbuff[7]==0X3F)
                                   {
                                          CMD_Ejecutado=3;
                                          write_eeprom(1,(Pickitbuff[5]));
                                          Num_Serie1=Pickitbuff[5];
                                          Respuesta=1;
                                    }
                                }
                                 break;
                                 
                              case 4:  //Comando para enviar el valor del offset
                                 {
                                    if(Pickitbuff[9]==0X3C && Pickitbuff[10]==0X3F)
                                   {
                                       CMD_Ejecutado=4;
                                       Respuesta=1;
                                       write_eeprom(16,(Pickitbuff[5]));   //Valor_ADC_Vacio_High
                                       write_eeprom(17,(Pickitbuff[6]));   //Valor_ADC_Vacio_Low
                                       write_eeprom(18,(Pickitbuff[7]));   //Valor_ADC_LLeno_High
                                       write_eeprom(19,(Pickitbuff[8]));   //Valor_ADC_Lleno_Low
                                       Leer_Bytes_de_Config();
                                    }
                                 }
                                 break;
                                 
                              case 5 :
                                 {
                                    if(Pickitbuff[10]==0X3C && Pickitbuff[11]==0X3F)
                                   {
                                          CMD_Ejecutado=5;
                                          Respuesta=1;
                                    }
                                 }
                                 break;
                             default:
                                    Respuesta=0;
                                    CMD_Ejecutado=0;
                       }
            }
         }
       }
       
    //F_CMD_Completo_Pickit=0;  
      
       fputc(0X23,Pickit);
       fputc(0X5E,Pickit);
       fputc(0X01,Pickit);             //indica que es una sensor board
       fputc(Num_Serie1,Pickit);
       /*fputc(Num_Serie2,Pickit);
       fputc(Num_Serie3,Pickit);
       fputc(Num_Serie4,Pickit);
       fputc(Num_Serie5,Pickit);
       fputc(Num_Serie6,Pickit);*/
       fputc(CMD_Ejecutado,Pickit);
       fputc(Respuesta,Pickit);
       fputc(0X3C,Pickit);
       fputc(0X3F,Pickit);
 Salida:
       Respuesta=0;
       CMD_Ejecutado=0;
       Indice_Pickit=0;

   }
}
//***************************************************************************\\
void Verifica_CMD_RS485(void)
{
   if(F_CMD_Completo_RS485==1)
   {
      F_CMD_Completo_RS485=0;
      
       if(RS485buff[0]==0X23)
       {
         if(RS485buff[1]==0X5E)
         {
             if(RS485buff[2]==0X02) //hay que reenviar todo el buffer 485 por el pickit
            {
                       for(int i=0;i<Indice_RS485;i++)
                       {
                           fputc(RS485buff[i],Pickit);   
                       }
                       //Indice_RS485=0;
                       break;
            }
            if(RS485buff[2]==0X01) 
            {
               //if(RS485buff[3] == Num_Serie1 && RS485buff[4]== Num_Serie2 && RS485buff[5]== Num_Serie3 && RS485buff[6]== Num_Serie4 && RS485buff[7]== Num_Serie5 && RS485buff[8]== Num_Serie6)
                  if(RS485buff[3] == Num_Serie1) 
                  
                       switch(RS485buff[4])
                       {
                       
                           case 0:   //Enviar estado por el 485
                               {
                                  if(RS485buff[5]==0X3C && RS485buff[6]==0X3F)
                                    {
                                       Envia_Estado_RS485();   //Envia el estado por el bus 485
                                       goto Salida2;
                                    }
                               }
                                break;
                           
                           case 1:  //Comando para modificar el tiempo entre comandos de status
                                 {
                                 
                                   if(RS485buff[6]==0X3C && RS485buff[7]==0X3F)
                                   {
                                       if(RS485buff[5]<0xF0) //Debera ser menor a 240 segundos
                                       {
                                             write_eeprom(0,(RS485buff[5]));// el dato recibido esta expresado en segundos 
                                             delay_ms(1);
                                             Tiempo_Envio_CMD = RS485buff[5];
                                             Tiempo=0;
                                             Segundos=0;
                                             Respuesta=1;
                                             CMD_Ejecutado=1;
                                             //goto Respuesta_CMD;
                                       }
                                       else
                                       {
                                             Respuesta=0;
                                             CMD_Ejecutado=1;
                                           //goto Respuesta_CMD;
                                       }
                                    }
                                 }
                                 break; 
                            case 2:  //Comando para activar relays onboard
                                 {
                                  if(RS485buff[6]==0X3C && RS485buff[7]==0X3F)
                                  {
                                       CMD_Ejecutado=2;
                                       Respuesta=0;
                                    if(RS485buff[5]==0x00)
                                    {
                                       Estado_Relays=0X00;
                                       Respuesta=1;
                                       write_eeprom(2,0X00);
                                    }
                                    if(RS485buff[5]==0X10)
                                    {
                                       Estado_Relays=0X10;
                                       Respuesta=1;
                                       write_eeprom(2,0X10);
                                    }
                                    if(RS485buff[5]==0X01)
                                    {
                                       Estado_Relays=0X01;
                                       Respuesta=1;
                                       write_eeprom(2,0X01);
                                    }
                                    if(RS485buff[5]==0X11)
                                    {
                                       Estado_Relays=0X11;
                                       Respuesta=1;
                                       write_eeprom(2,0X11);
                                    }
                                  }
                                 }
                                 break;
                            
                             case 3:  //Comando para establecer numero de serie
                                {
                                    if(RS485buff[6]==0X3C && RS485buff[7]==0X3F)
                                    {
                                       CMD_Ejecutado=3;
                                       write_eeprom(1,(RS485buff[5]));
                                       Num_Serie1=RS485buff[5];
                                       Respuesta=1;
                                    }
                                 }
                                 break;
                             case 4:  //Comando para enviar los settings de los valores del adc de la cisterna
                                 {
                                    if(RS485buff[9]==0X3C && RS485buff[10]==0X3F)
                                    {
                                       CMD_Ejecutado=4;
                                       Respuesta=1;
                                       write_eeprom(16,(RS485buff[5]));   
                                       write_eeprom(17,(RS485buff[6]));   
                                       write_eeprom(18,(RS485buff[7]));   
                                       write_eeprom(19,(RS485buff[8]));   
                                       Leer_Bytes_de_Config();
                                    }
                                 }
                                 break;
                              case 5 :
                                 {
                                    if(RS485buff[10]==0X3C && RS485buff[11]==0X3F)
                                    {
                                       CMD_Ejecutado=5;
                                       Respuesta=1;
                                    }
                                 }
                                 break;
                             default:
                                    Respuesta=0;
                                    CMD_Ejecutado=0;
                       }
            }
         }
       }
       output_high(DRV_485);                                     
       delay_ms(1);
       fputc(0X23,RS485);
       fputc(0X5E,RS485);
       fputc(0X01,RS485);  
       fputc(Num_Serie1,RS485);
       fputc(CMD_Ejecutado,RS485);
       fputc(Respuesta,RS485);
       fputc(0X3C,RS485);
       fputc(0X3F,RS485);
       delay_ms(3);
       output_low(DRV_485);  
 Salida2:
       Respuesta=0;
       CMD_Ejecutado=0;
       Indice_RS485=0;
   }
}
//*****************************************************************************
//*****************************************************************************
//***************************************************************************\\
void Verifica_CMD_Sensor(void)
{
   if(F_CMD_Completo_Sensor==1)
   {
      F_CMD_Completo_Sensor=0;
      //fputc(0XBB,Pickit);
      Output_low(DRV_Sensor);
       if(Sensorbuff[0]==0X52 && Sensorbuff[5]==0X0D)
       {
            Medida_1 = ((int16) (Sensorbuff[1]) - 0X30)*1000;
            Medida_2 = ((int16) (Sensorbuff[2]) - 0X30)*100;
            Medida_3 = ((int16) (Sensorbuff[3]) - 0X30)*10;
            Medida_4 = ((int16) (Sensorbuff[4]) - 0X30)*1;
            Distancia_Medida = Medida_1+Medida_2+Medida_3+Medida_4;
            Distancia = Distancia_Medida;
            Distancia_Medida_Low = Distancia;
            Distancia_Medida_High = Distancia>>8;
       }
       else
       {
            fprintf(pickit,"No es un comando valido, no se recibio medida \n\t\r");
       }
   }
}
         
         
/*****************************************************************************\
\*****************************************************************************/
void main()
{
   setup_adc_ports(sAN7);   
   setup_adc(ADC_CLOCK_INTERNAL);
   setup_vref(FALSE);
   setup_spi(SPI_SS_DISABLED);
   enable_interrupts(GLOBAL);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_EXT1);
   enable_interrupts(INT_EXT2);
   enable_interrupts(INT_RDA);
   clear_interrupt(INT_TIMER1);
   clear_interrupt(INT_EXT1);
   clear_interrupt(INT_EXT2);
   clear_interrupt(INT_RDA);
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_8);      //100 ms overflow
   set_timer1(28036);
   output_low(DRV_485);       //habilito el driver 485 para recibir a cualquier momento.
   ext_int_edge(1, H_TO_L);
   ext_int_edge(2, H_TO_L);
   Output_low(DRV_Sensor);
 
//*******************Verificación de bytes de configuracion******************\\
   delay_ms(10);     //tiempo requerido para poder trabajar con le eeprom
   Leer_Bytes_de_Config();
   
   if(Tiempo_Envio_CMD==0XFF) //si no se ha configurado el tiempo pone por default el valor de 5 segundos
   {
      write_eeprom(0,1);
      Tiempo_Envio_CMD=1;
   }
   if(Num_Serie1==0XFF) //si no se ha configurado numero de serie, se pone por default 1
   {
      write_eeprom(1,1);
      Num_Serie1=1;
   }
    if(Distancia_Vacio_High==0XFF && Distancia_Lleno_High==0XFF) //si no se ha configurado los niveles de adc máximo y mínimo, porcentaje toma valor de 0XFE
   {
      Porcentaje = 0xFE;
   }
   if(Estado_Relays==0XFF)
   {
      Estado_Relays = 0X00;
      write_eeprom(2,0);
   }
   Tiempo_Blink=10; 
   
   puts("Iniciando",pickit);
   
   while(true)
   {
      Temporizaciones();  
      //Actualiza_Estado_Relays();
      //Verifica_CMD_Pickit();
      //Verifica_CMD_RS485();
      //Verifica_CMD_Sensor();
   }
}
