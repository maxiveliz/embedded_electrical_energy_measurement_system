//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


//HardwareX

//Embedded Electrical Energy Measurement System Based on the M90E36A to Acquire and Process High-Frequency Features in Household Appliances.

//Maximiliano E. Véliz (mveliz@campus.ungs.edu.ar)
//Gustavo E. Real (greal@campus.ungs.edu.ar)



// Conditional compilations
// If there is no Wifi board, comment out the following line
//#define S_WIFI

#define S_M90E36A
//--------------------------------------------------------------------------------------------------

// If you want to test E2, CAN communication, DMA_M90, M90 DMA mode commands
// uncomment the following line


#define TEST_VARIOS

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

 
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
#include <Wire.h>             //Library for I2C bus handling
#include <SPI.h>              //Library for SPI bus handling
#include <SD.h>               //Library for SD/MMC memory management
#include <SerialFlash.h>      //Flash memory management library 
#include <DueTimer.h>         //Library for setting timer-controlled tasks
#include <LiquidCrystal.h>    //Library for 6-pin display management
#include "variant.h"          //Generic definition library for due_can
#include <due_can.h>          //Library for CAN bus operation
#include <M90E36A.h>          //M90E36A Variable and Register Definitions

//--------------------------------------------------------------------------------------------------
//Datos del equipo
#define proyecto "PROJECT: Embedded Electrical Energy Measurement System Based on the M90E36A to Acquire and Process High-Frequency Features in Household Appliances."    
#define equipo   "EQUIPMENT:    Arcane Meter"
#define vers     "VERS.:     1.DMA"
#define fecha    "DATE:     2023"
//--------------------------------------------------------------------------------------------------


#define PWM2 2 //Pin corresponding to Open Drain digital output 0
#define PWM3 3 //Pin corresponding to digital output Open Drain 1
#define PWM4 4 //Pin corresponding to digital output Open Drain 2
#define PWM5 5 //Pin corresponding to digital output Open Drain 3
#define PWM6 6 //Pin corresponding to digital output to Relay 1
#define PWM7 7 //Pin corresponding to digital output to Relay 2
#define PWM8 8 //Pin corresponding to the digital output to Relay 3
#define PWM9 9 //Pin corresponding to the digital output to Relay 4
#define PWM10 10 //Pin corresponding to the generic digital output 1
#define PWM11 11 //Pin corresponding to the generic digital output 2
#define PWM12 12 //Pin corresponding to generic digital output 3
#define PWM13 13 //Pin corresponding to the generic digital output 4

#define DIN0 29 //Pin corresponding to digital input DIN0
#define DIN1 27 //Pin corresponding to digital input DIN1
#define DIN2 25 //Pin corresponding to the digital input DIN2
#define DIN3 23 //Pin corresponding to digital input DIN3
#define DIN4 22 //Pin corresponding to the digital input DIN4
#define DIN5 24 //Pin corresponding to digital input DIN5
#define DIN6 26 //Pin corresponding to digital input DIN6
#define DIN7 28 //Pin corresponding to the digital input DIN7

#define WRA1 36 //Pin to validate gain inAmp 1 (Channel 09)
#define WRA2 37 //Pin to validate gain inAmp 2 (Channel 10)
#define WRA3 38 //Pin to validate gain inAmp 3 (Channel 11)
#define WRA4 39 //Pin to validate gain inAmp 4 (Channel 12)
#define G_0 40 //Pin 0 for inAmp gain selection (Low Bit)
#define G_1 41 //Pin 1 for inAmp gain selection (High bit)

#define TST 45 //TST for indicating power on of the SADyC32 board (Pin 45)
#define rs 46 //Pin definitions for the display in 4-pin data configuration + En + Rs
#define en 47
#define d4 50
#define d5 51
#define d6 52
#define d7 53

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define WP1 30 //Output that handles the WRITE PROTECT protect of the EEprom-I2C memory: 24AA1025 (on new boards).

#define WP 32 //Output handling the WRITE PROTECT protect of the Flash-SPI memory: S25FL127S.
#define HOLD 33 //Output that handles the HOLD of the Flash-SPI memory: S25FL127S 
#define CS_FL 31 //Output that handles the CHIP SELECT of the Flash-SPI memory: S25FL127S

#define CS_SD 34 //Output handling the CHIP SELECT of the SD memory (new board --> 34 ; Mikroe --> 47)
#define WP_SD 35 //Input for detecting the SD memory Write Protect
#define CD 44 //Input for detecting presence of the SD CHIP

#define RS485_DIR 43 //Output handling the RS232 communication address
#define RS485_Rx LOW //Value for the RS485-tranceiver to be in receive mode
#define RS485_Tx HIGH //Value for the RS485-tranceiver to be in transmit mode

#define MAX_RESOL 12 //Maximum resolution of the converter

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//Definitions for CAN communication
//Function definitions
#define CAN_ERR 0x01
#define CAN_DO 0x04
#define CAN_SET 0x08
#define CAN_QRY 0x0C
#define CAN_ACK 0x10
#define CAN_POST 0x14
#define CAN_HB 0x18

//Device addresses
#define CAN_BROADCAST 0x00            //Address for messages to all
#define CAN_MASTER_ADDRESS 0x01
#define CAN_GEN_EOLICO 0x02
#define CAN_CONV_DCAC 0x03
#define CAN_VOL_INERCIA 0x04

//Generics and masks
#define CAN_TRANSMISION_A
#define CAN_TRANSMISION_DE
#define MAX_CAN_FRAME_DATA_LEN 8
#define CANSTB 48                     //Defines the operating mode of the TJA1040 CAN Transeiver - 0 --> Normal Mode || 1 --> StadBy Mode
#define DISPO_ID_MASK 0x1F            //Mask for decoding the source device (Sender) in the CAN Packet Identifier
#define MENSAJE_ID_MASK 0x3F          //Mask to decode the message type from in the CAN Packet Identifier

//Definitions of Variable Types
#define CAN_DIGITAL_OUT 0x00
#define CAN_DIGITAL_IN  0x01
#define CAN_ANALOG_OUT  0x02
#define CAN_ANALOG_IN  0x03
#define CAN_MOD 0x04
#define CAN_ANA 0x05
#define CAN_INAMP 0x06
#define CAN_AINAMP 0x07
#define CAN_PWM 0x08
#define CAN_ECHO 0x09
#define CAN_RTC 0x0A
#define CAN_PAR 0x0B
#define CAN_SRST 0x0C
#define CAN_U_A 0x10
#define CAN_U_B 0x11
#define CAN_U_C 0x12
#define CAN_I_A 0x13
#define CAN_I_B 0x14
#define CAN_I_C 0x15
#define CAN_I_N1 0x16
#define CAN_PA_A 0x17
#define CAN_PA_B 0x18
#define CAN_PA_C 0x19
#define CAN_PA_TOT 0x1A
#define CAN_PR_A 0x1B
#define CAN_PR_B 0x1C
#define CAN_PR_C 0x1D
#define CAN_PR_TOT 0x1E
#define CAN_PS_A 0x1F
#define CAN_PS_B 0x20
#define CAN_PS_C 0x21
#define CAN_PS_TOT 0x22
#define CAN_FP_A 0x23
#define CAN_FP_B 0x24
#define CAN_FP_C 0x25
#define CAN_FP_TOT 0x26
#define CAN_THDU_A 0x27
#define CAN_THDU_B 0x28
#define CAN_THDU_C 0x29
#define CAN_THDI_A 0x2A
#define CAN_THDI_B 0x2B
#define CAN_THDI_C 0x2C
#define CAN_FREC 0x2D
#define CAN_TEMP 0x2E
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//State machine definitions
#define LIBRE false                   
#define OCUPADO true

//Definitions for receiving logic via serial ports
#define ESCAPE 0x1B           //Escape --> 0x1B - Set to 0x63 "c" only for console tests that do not support "0x1B".
#define IN_END 0x0D           //End of messages
#define TIME_OUT 500000       //Time-out of reception
#define VECT_END 20           //Receive buffer vector size 
#define LARGO_ARCH 35         //Maximum file length
#define NO_SERIAL 255         //Communication no port
#define SERIAL_0 0            //Serial communication 0 (Monitor - Arduino board micro USB terminal)
#define SERIAL_1 1            //Serial 1 communication (RS232)
#define SERIAL_2 2            //Serial communication 2 (RS485)  
#define SERIAL_3 3            //Serial 3 communication (WiFi)
#define SERIAL_4 4            //Native USB communication
#define delta_B_S0 115200     //Monitor/Arduino bandwidth in baud (bits/sec)- Max. tested speed: 460.800
#define delta_B_S1 115200     //RS232 bandwidth in baud rate (bits/sec)- Maximum speed not verified
#define delta_B_S2 19200      //RS485 bandwidth in baud rate (bits/sec)- Maximum speed not verified
#define delta_B_WiFi 115200   //WiFi bandwidth in baud (bits/sec)- Maximum unverified speed
#define delta_B_USB 0         //Bandwidth of Native USB (Arduino) in baud (bits/sec) - Tested maximum speed: 2000000
#define conCR true
#define sinCR false
#define comillas 0x22
#define grados 0xF8

//Software reset definitions
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

//WatchDog can range from (1/256)sec to 16sec. This is set in WDV and WDD. I.e. --> (1/256*WDV) in seconds  
#define WDTO_1S WDT_MR_WDRSTEN | WDT_MR_WDRPROC | WDT_MR_WDV(256) | WDT_MR_WDD(256)

//--------------------------------------------------------------------------------------------------
//SPISettings SPI bus for SD/MMC memory management
//SPISettings Spi_mem_param(50000000, MSBFIRST, SPI_MODE0);
//--------------------------------------------------------------------------------------------------
//Definitions needed to use the uC as SPI Slave
#define largo_buff_DMA 4096
#define tope_buff_DMA largo_buff_DMA-1
#define s_Buff1 0
#define s_Buff2 1
#define Deshabilitada 0
#define Habilitada 1
byte buff_DMA1[largo_buff_DMA];
byte buff_DMA2[largo_buff_DMA];
volatile uint16_t punt_buff_DMA1;
volatile uint16_t punt_buff_DMA2;
volatile bool semaforo_DMA;
volatile bool transm_USB;

//--------------------------------------------------------------------------------------------------
const byte mem_ad = 0xa0;                         // EEProm-I2C memory base address: 24AA1025    
const unsigned long MEM_FIN_REG = 130559;         //End of EEProm for registration || 130559 (final 512 bytes are reserved)
const byte rtc_ad = 0x68;                         //RTC-I2C address: DS1307
const byte rtc_m_st = 0x08;                       //RTC ram start address
char separador = ';';                             //Separator for excel
char LF = 0x0A;                                   //Line feed
char CR = 0x0D;                                   //Carriage return

//Defining the places in E2 to store configuration registers, passwords, etc.
const unsigned long e2_RegConfig = MEM_FIN_REG + 1;               //Beginning address of the last 512 bytes of E2 (130560<->130943), for constants and set-points
                                                                  //Remainder of E2 (128 bytes) allocated to M90E36A (130944<->131071) so cannot be used for others
const unsigned long e2_CanMasterSlave = e2_RegConfig;             //One byte for CAN Master address
const unsigned long e2_CanDirMasterSlave = e2_CanMasterSlave+1;   //One byte for CAN Slave address
const unsigned long e2_RedWifi = e2_CanDirMasterSlave+1;          //32 bytes for WiFi Network Name
const unsigned long e2_WifiPass = e2_RedWifi + 32;                //32 bytes for WiFi network password
//E2 free from "e2_free" up to and including 130943
//const unsigned long e2_Libre = e2_WifiPass + 32

//volatile => are in RAM and not in registers. Very important in interrupts.

typedef enum {                        //Serial Receiving Machine Status Definition
  e_comando,
  e_parseo
}estadosRS_e;
estadosRS_e estadosRS = e_espera;

bool comm0_flag;                      //Communication in progress indicator in serial 0
bool mem_full;                        //EEProm memory full indicator
bool rtc_flag;
volatile bool Start_flag;             //Flag of the start of the time frames
volatile bool Stop_flag;              //Flag for termination of time recording
volatile bool Wifi_ini;               //Indicator that the WiFi board is initialised (True --> initialised)
volatile bool Sd_ini;                 //Indicator that the SD memory is initialised (True --> initialised)
volatile bool Sf_ini;                 //Indicator that Serial Flash memory is initialised (True --> initialised)
volatile bool CAN0_ini;               //Indicator that CAN0 communication is initialised (True --> initialised)
volatile bool M90E36A_ini;            //Indicator that the board with the M90E36A is initialised (True --> initialised)
volatile bool estadoLedTest;          //Indicates the status of the indicator LED
volatile bool ComandoNoEjecutado_flag;//Indicates whether a command could have been executed || "1" --> not executed
//------
char inChar;                          //Variable storing the serial input character
//------
volatile byte modo;                   //Defines the working mode of the "see specifications" plate
volatile byte cantCan_analog1;        //Number of analogue automation channels
byte contCan_analog1;                 //Idem for the counter
volatile byte cantCan_analog2;        //Number of analogue instrumentation channels
byte contCan_analog2;                 //Idem for the counter
volatile byte reg_flag;               //Registration in progress indicator
byte auxbuffer[VECT_END];             //Vector for serial reception characters
byte k_g;                             //Counter for correction characters
volatile byte pin1, pin2;             //Variable for uC pin handling in routines
volatile byte p_Serial;               //Indicates that serial is in use || 255 = none
volatile byte p_SerialTmp;            //Indicates that serial is in use for timed events || 255 = none
//------
int comando;                          //Variable where the entered command is saved
int i;
int vect_canales1[8];                 //Vector for data storage of the analogue automation channels
int vect_canales2[4];                 //Vector for data storage of analogue instrumentation channels
unsigned int wifiSendTimeout;         //Wifi standby time-out
unsigned int mask_Can1;               //Value for assigning and controlling the channels in the ADC
unsigned int mask_Can2;               //Value for assigning and controlling the channels on the ADC (in-Amp)
volatile unsigned int tiempo_reg;     //Stores the registration time 
long delta_B;                         //Variable that loads the speed according to the communication used
unsigned long comm0_out;              //Variable to be decremented for serial time-out
unsigned long t_Muestro_Min;          //Minimum sampling time
unsigned long pumemConst;             //Pointer to E2 memory of constants in the last 1024 bytes (130,048 --> 131,072)
volatile unsigned long pumem;         //Pointer to register memory E2
//------
String inString = "";                 //String for serial reception       
//------
char archivo[LARGO_ARCH];             //File Format: 8.3
//------
CAN_FRAME canBuffer_In;               //Input buffer for CAN communication
CAN_FRAME canBuffer_Out;              //Output buffer for CAN communication
byte CAN_MASTER;                      //Define which type of CAN device is the present programme
byte CAN_SLAVE_ADDRESS;               //Address of any slave

//Definiciones varias
bool parpadeoFlag = false;
const char ACK = 0x06;
const char NAK = 0x15;
bool emu_TST = false;
char archivoBack[LARGO_ARCH];         //To save the file name for different purposes
char ensayo[LARGO_ARCH];              //For files on SD

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------- INITIALIZATION -------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
void setup() {
//Initialisation of pins of the micro ---------------------------

  pinMode(TST, OUTPUT); digitalWrite(TST, HIGH);        //TST is turned on during initialisation

  pinMode(PWM2, OUTPUT); digitalWrite(PWM2, LOW);
  pinMode(PWM3, OUTPUT); digitalWrite(PWM3, LOW);
  pinMode(PWM4, OUTPUT); digitalWrite(PWM4, LOW);
  pinMode(PWM5, OUTPUT); digitalWrite(PWM5, LOW);
  pinMode(PWM6, OUTPUT); digitalWrite(PWM6, LOW);
  pinMode(PWM7, OUTPUT); digitalWrite(PWM7, LOW);
  pinMode(PWM8, OUTPUT); digitalWrite(PWM8, LOW);
  pinMode(PWM9, OUTPUT); digitalWrite(PWM9, LOW);
  pinMode(PWM10, OUTPUT); digitalWrite(PWM10, LOW);              
  pinMode(PWM11, OUTPUT); digitalWrite(PWM11, LOW);              
  pinMode(PWM12, OUTPUT); digitalWrite(PWM12, LOW);
  pinMode(PWM13, OUTPUT); digitalWrite(PWM13, LOW);

  pinMode(WRA1, OUTPUT); digitalWrite(WRA1, LOW);
  pinMode(WRA2, OUTPUT); digitalWrite(WRA2, LOW);
  pinMode(WRA3, OUTPUT); digitalWrite(WRA3, LOW);
  pinMode(WRA4, OUTPUT); digitalWrite(WRA4, LOW);
  pinMode(G_0, OUTPUT); digitalWrite(G_0, LOW);
  pinMode(G_1, OUTPUT); digitalWrite(G_1, LOW);

  pinMode(M90_dmaCTRL, OUTPUT); digitalWrite(M90_dmaCTRL, LOW);
  
  pinMode(WP1, OUTPUT); digitalWrite(WP1, HIGH);
  
  pinMode(CS_FL, OUTPUT); digitalWrite(CS_FL, HIGH);
  pinMode(HOLD, OUTPUT); digitalWrite(HOLD, HIGH);
  pinMode(WP, OUTPUT); digitalWrite(WP, HIGH);

  pinMode(CS_SD, OUTPUT); digitalWrite(CS_SD, HIGH);
  pinMode(WP_SD, INPUT);
  pinMode(CD, INPUT);

  pinMode(RS485_DIR, OUTPUT);               //Management of the RS485 communication address
  digitalWrite(RS485_DIR, RS485_Rx);        //Prepared for reception via RS485

  pinMode(CANSTB, OUTPUT);                  //Output to set mode of CAN Tranceiver
  digitalWrite(CANSTB, LOW);                //Normal mode

  pinMode(DIN0, INPUT);
  pinMode(DIN1, INPUT);
  pinMode(DIN2, INPUT);
  pinMode(DIN3, INPUT);
  pinMode(DIN4, INPUT);
  pinMode(DIN5, INPUT);
  pinMode(DIN6, INPUT);
  pinMode(DIN7, INPUT);

  //Network analyser chip select chip (Microchip/Atmel M90E36A)
  pinMode(CS_M90, OUTPUT);
  digitalWrite(CS_M90, HIGH);

//Fin pin initialisation ----------------------------------

  estadosRS = e_espera;                    //Serial Receive State Machine Start Point

  Serial.begin(delta_B_S0);                //Serial communication (TX0, RX0) Arduino Monitor via USB-ProgrammingPort connector

  Serial1.begin(delta_B_S1);               //Serial communication (TX1, RX1) RS232 via DB9 male connector
  Serial2.begin(delta_B_S2);                //RS485 communication (TX2, RX2)
  Serial3.begin(delta_B_WiFi);              //WiFi Communication (TX3, RX3) (ESP8266)
  SerialUSB.begin(delta_B_USB);             //Communication via native USB on Arduino Due board

  Serial.println();
  Serial.println(F("Inicializando..."));
  Serial.println();

  if (Can0.begin(CAN_BPS_250K)) {
    Serial.println("CAN0 Inicializado");
    CAN0_ini = true;
  }
  else {
    Serial.println("CAN0 initialization (sync) ERROR");
    CAN0_ini = false;
  }

  Can0.watchFor(CAN_TRANSMISION_DE);  

  Wire.begin();
  SPI.begin();
  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(proyecto);
  lcd.setCursor(0, 1);
  lcd.print(vers);
  delay(3000);
  lcd.clear();
  lcd.print(F("Initializing..."));
  delay(2000);
  lcd.clear();
  
//Inicialización memoria SD
  lcd.setCursor(0, 0);
  if (!SD.begin(CS_SD)) {
    Serial.println(F("SD card not present, not formatted or failed."));
    lcd.print(F("SD: No"));
    Sd_ini = false;
  } else {
    Serial.println(F("SD card initialized."));
    lcd.print(F("SD: Si"));
    Sd_ini = true;
  }

//Inicialización memoria Flash ----------------------------------  
  if (!SerialFlash.begin(CS_FL)) { 
    Serial.println(F("Flash memory cannot be accessed via SPI."));
    Sf_ini = false; 
  } else {
    Serial.println(F("Flash found and initialised."));
    Sf_ini = true;
  }

//Seteo de la referencia y modo de funcionamiento del ADC -------
  analogReference(AR_DEFAULT);
  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;    //To convert to approx. 6uS instead of 40uS.
  REG_ADC_MR = (REG_ADC_MR & 0xFFFFFF0F) | 0x00000080;    //Mode: free running without shutdown of Core ADC between conversions (page 1333 of the Sam3X manual).
//  REG_ADC_MR |= 0xC0;                                   //<-- Before - Mode: free running with Core ADC shutdown between conversions (page 1333 of the Sam3X manual). 
  mask_Can1 = 0x00FF;                                     //Default
  mask_Can2 = 0x3C00;                                     //Default
  analogReadResolution(MAX_RESOL);
  analogWriteResolution(MAX_RESOL);

  rtc_write(rtc_m_st + 7, 0x00);                          //Stores in clock start code

  ini_var();                                              //Initialisation of all board variables and states
  set_ADC_Chan();                                         //Setting in the ADC of the chosen channels 
  REG_ADC_CR=0x02;                                        //Beginning of the conversion 

//Inicialización de la placa Wifi.
#ifdef S_WIFI
  parpadeoFlag = true;
  int reintento = 3;
  do{
    if(parpadeoFlag == true) {
      lcd.setCursor(0, 1);
      lcd.print(F("WF:   "));
    }
    parpadeoFlag = !parpadeoFlag;
    lcd.setCursor(0, 1);
    if(ini_Wifi()){                                         
      Serial.println(F("WiFi inicializado"));
      lcd.print(F("WF: Si"));
      Wifi_ini = true;
    }else{
      Serial.println(F("Wifi no inicializado"));
      lcd.print(F("WF: No"));
      Wifi_ini = false;
    }
    reintento--;
  }while((Wifi_ini == false) && (reintento >0));
#else
  Wifi_ini = false;
  Serial.println(F("Wifi no presente"));
  Serial.println();
#endif

//Armado de "archivoBack" con el RTC
  get_time();
  String s="";
  s = s + char(auxbuffer[9]) + char(auxbuffer[10]) + char(auxbuffer[7])+ char(auxbuffer[8])+ char(auxbuffer[1])+ char(auxbuffer[2]) + char(auxbuffer[3])+ char(auxbuffer[4]);
  s.toCharArray(archivoBack,sizeof(archivoBack));
  
//Inicialización del M90E36A
#ifdef S_M90E36A
  M90E36A_ini = false;
  if(e2prom_read(e2_M90_Config_OK) == 0x01){
    IniSet_Config_E2();                       // Load respective registers into RAM from E2
  }
  if(e2prom_read(e2_M90_Adj_OK) == 0x01){
    IniSet_Adj_E2();                          //Loading the respective registers into RAM from E2
  }
  if((e2prom_read(e2_M90_Cal_OK) == 0x01) && (e2prom_read(e2_M90_Harm_OK) == 0x01)){
    IniSet_Cal_E2();                          //Loading the respective registers into RAM from E2
    IniSet_Harm_E2();                        //Loading the respective registers into RAM from E2
  }

  iniReg_M90_Ram();                           //Loading the respective registers into the M90E36A from RAM
  ini_Temperatura();                          //Initialising the temperature sensor of the M90E36A
  ini_AmplifDft();                            //Initialisation of amplifications for DFT Analysis

  MaxCompArm = e2prom_read(e2_M90_MaxCompArm);
  if(MaxCompArm < 2 || MaxCompArm > 32) MaxCompArm = 10;

  regHR_flag = e2prom_read(e2_M90_regHR);
  if(regHR_flag == 1) {
    write_M90_1R(DFT_CTRL, 0x0001);     //Start the DFT module for the calculation of harmonics per channel 
  } else {
    regHR_flag = 0;
    write_M90_1R(DFT_CTRL, 0x0000);     //For the DFT module for the calculation of harmonics per channel 
  }

  regTodo_flag = e2prom_read(e2_M90_regTodo);
  if(regTodo_flag == 1) {
    write_M90_1R(DFT_CTRL, 0x0001);     //Start the DFT module for the calculation of harmonics per channel  
  } else {
    regTodo_flag = 0;
    write_M90_1R(DFT_CTRL, 0x0000);     //For the DFT module for the calculation of harmonics per channel
  }

  Serial.println(F("M90E36A inicializado."));
  M90E36A_ini = true;

//If you were logging before the M90E36A parameter start-up, prepare the system for further logging.
  if(registrandoM90_flag == 1 && reg_flag == 0) {
    get_time();
    sprintf(ensayo,"%s%s%d%d",archivoBack,".0", (auxbuffer[5] - 48), (auxbuffer[6] - 48));
    Timer4.attachInterrupt(Reg_M90_Ram);                        //Timer para registro en RAM
    Timer4.start(tiempo_reg * 1000);
  }
#else
  M90E36A_ini = false;
  Serial.println(F("Placa con M90E36A no presente"));
  Serial.println();
#endif
//End of initialisation M90E36A

//If you were recording analogue channels before the start-up, prepare the system for further recording.
  if(reg_flag == 1 && registrandoM90_flag == 0) {
    graba_ini_E2();                                       //Store on EEProm headed by feed-back
    Timer4.attachInterrupt(Registro_Temp_E2);             //Timer for registration in E2
    Timer4.start(tiempo_reg * 1000000);
  }

#ifdef S_WDOG
  WDT_Enable(WDT, WDTO_1S);
  Serial.println(F("W-Dog activado"));
#else
  Serial.println(F("W-Dog no activado"));
  Serial.println();
#endif

 //Setup DMA_M90E36A
  punt_buff_DMA1 = 0;
  punt_buff_DMA2 = 0;
  semaforo_DMA = s_Buff1;

  ComandoNoEjecutado_flag = 0;

  titilaTST(6, 125);                                       //End initialization                                                    

  enEspera();

  SerialUSB.println("...................");
  
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------- START OF MAIN PROGRAMME  ---------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
void loop() {
/*  
#ifdef S_WDOG
  WDT_Restart(WDT);                   
#endif

  if(comm0_flag==true){                                                       //Time-out serial
    comm0_out--;
    if (comm0_out==0){
      comm0_flag = false;
      p_Serial = NO_SERIAL;
      digitalWrite(TST, LOW);
      estadosRS=e_espera;
    }
  }

  if (SerialUSB.available() > 0){                                             //Native USB communication
    USB_serialEvent();
  }
 
  if(Can0.available() > 0) {                                                  //Receiving frames via CAN Bus
    Can0.read(canBuffer_In);
    if(CAN_MASTER == 0){
//      Serial.println("-----------------");
//      Serial.println(canBuffer_In.id);
//      Serial.println(canBuffer_In.data.low);
//      Serial.println(canBuffer_In.data.high);
//      Serial.println("-----------------");
      if((((byte)(canBuffer_In.data.low >> 24) == CAN_SLAVE_ADDRESS) && (((byte)canBuffer_In.id & DISPO_ID_MASK) == CAN_MASTER_ADDRESS)) || (((byte)canBuffer_In.id & DISPO_ID_MASK) == CAN_BROADCAST)) {    
//        Serial.print(F("CAN mensaje recibido: "));
//        Serial.print(canBuffer_In.data.low);
//        Serial.print(canBuffer_In.data.high);
//        Serial.println();
        if(CanMessParsing() == false){
          Serial.println(F("Error en analisis del mensaje CAN"));
        }
      }
    } else {
      if((byte)(canBuffer_In.data.low >> 24) == CAN_MASTER_ADDRESS) {    
//        Serial.print(F("CAN mensaje recibido: "));
//        Serial.print(canBuffer_In.data.low);
//        Serial.print(canBuffer_In.data.high);
//        Serial.println();
        if(CanMessParsing() == false){
          Serial.println(F("Error en analisis del mensaje CAN"));
        }
      }
    }
  }

  if(check_M90 == true){
    check_M90 = false;
    serialOutputTmp(F("Chequeando integridad de registros del M90E36A"), 0, conCR);
    ChEf_EfVal();
    serialOutputTmp(F("Fin chequeo integridad de registros del M90E36A"), 0, conCR);
  }

  if(regM90_flag == true){
//    unsigned long milisegundos = millis();
    regM90_flag = false;
    emu_TST = !emu_TST;
    digitalWrite(TST, emu_TST);
    registra_M90();
//    Serial.println(millis()-milisegundos);    
  }

  if(regHR_flag == 1 && ((read_M90_1R(DFT_CTRL) & 0x0001) == 0x0000)){
//    unsigned long milisegundos = millis();
    emu_TST = !emu_TST;
    digitalWrite(TST, emu_TST);
    registra_M90_HRI();
    write_M90_1R(DFT_CTRL, 0x0001);      //Start the DFT module for the calculation of harmonics per channel. 
//    Serial.println(millis()-milisegundos);    
  }

  if(regTodo_flag == 1 && ((read_M90_1R(DFT_CTRL) & 0x0001) == 0x0000)){
//    unsigned long milisegundos = millis();
    emu_TST = !emu_TST;
    digitalWrite(TST, emu_TST);
    registra_M90_Todo();
    write_M90_1R(DFT_CTRL, 0x0001);     //Start the DFT module for the calculation of harmonics per channel.  
//    Serial.println(millis()-milisegundos);    
  }

  if(buff_Lleno == true){
//    unsigned long milisegundos1 = millis();
    buff_Lleno = false;
    if(!graboRam_SD(ensayo, BuffMedicionesMax)){                                          //Copy data from RAM to SD Memory
      serialOutputTmp("", 0, conCR);
      serialOutputTmp(F("Error al abrir "), 0, sinCR);
      serialOutputTmp(ensayo, 0, conCR);
    }
//    Serial.println(millis()-milisegundos1);    
  }
*/
  if(transm_USB == Habilitada){
    transm_USB = Deshabilitada;
    if(semaforo_DMA == s_Buff2){
      SerialUSB.write((byte*)&buff_DMA1, largo_buff_DMA);
    } else {
      SerialUSB.write((byte*)&buff_DMA2, largo_buff_DMA);
    }
  }

}

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------- ROUTINES --------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that flashes the TST led. it receives the times and the time of each time.
void titilaTST(int veces, unsigned long tiempo){
  int i;
   
  for(i=1;i<=veces;i++){
    digitalWrite(TST,HIGH);
    delay(tiempo);
    digitalWrite(TST,LOW);
    delay(tiempo);
  }
}

//--------------------------------------------------------------------------------------------------
//Mensaje de sistema en espera
void enEspera(void){

  if(M90E36A_ini == true || CAN0_ini == true){
    //Placa inicializada y lista para trabajar
    Serial.println();
    Serial.println(F("Sistema listo y en espera..."));
  } else {
    Serial.println();
    Serial.println(F("Modo comando..."));
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of all variables for a correct execution of the program
void ini_var(void){
  pumem = 0;
  pumem = pumem + (unsigned long)rtc_read(rtc_m_st + 1);
  pumem = pumem + ((unsigned long)rtc_read(rtc_m_st + 2) << 8);  
  pumem = pumem + ((unsigned long)rtc_read(rtc_m_st + 3) << 16);  
  pumem = pumem + ((unsigned long)rtc_read(rtc_m_st + 4) << 24);  
  if(pumem > MEM_FIN_REG) pumem = 0;
    
  cantCan_analog1 = rtc_read(rtc_m_st + 5);
  if(cantCan_analog1 < 1 || cantCan_analog1 > 8) cantCan_analog1 = 8;  

  cantCan_analog2 = rtc_read(rtc_m_st + 6);
  if(cantCan_analog2 < 1 || cantCan_analog2 > 4) cantCan_analog2 = 4;  

  reg_flag = rtc_read(rtc_m_st + 8);
  if(reg_flag > 1) reg_flag = 0;

  check_M90 = false;
  DMA_flag = false;
//  SPI_RD_flag = false;
//  DMA_Off_Flag = false;
  registrandoM90_flag = e2prom_read(e2_M90_registrando);
  if(registrandoM90_flag > 1) registrandoM90_flag = 0;

  tiempo_reg = 0;
  tiempo_reg = tiempo_reg + (unsigned int)rtc_read(rtc_m_st + 9);
  tiempo_reg = tiempo_reg + ((unsigned int)rtc_read(rtc_m_st + 10) << 8);  
  if(reg_flag == 1 && (tiempo_reg < 1 || tiempo_reg > 3600)) tiempo_reg = 15;
  if(registrandoM90_flag == 1 && (tiempo_reg < 333 || tiempo_reg > 60000)) tiempo_reg = 1000;
  
  modo = rtc_read(rtc_m_st + 11);  
  if(modo < 1 || modo > 5) modo = 1;

  contCan_analog1 = 0;
  contCan_analog2 = 0;
  inString = "";
  comm0_flag = false;
  Start_flag = false;
  Stop_flag = false;  
  p_Serial = NO_SERIAL;

  //Inicialización CAN ------------------------------------------------------------------ 
  CAN_MASTER = e2prom_read(e2_CanMasterSlave);
  CAN_SLAVE_ADDRESS = e2prom_read(e2_CanDirMasterSlave);
//  CAN_MASTER = 0x00;
//  CAN_SLAVE_ADDRESS = 0x04;
//  CAN_MASTER = 1;
//  CAN_SLAVE_ADDRESS = 1;
  Serial.println(F("------------------------"));
  Serial.print(F("CAN_MASTER=        "));
  Serial.println(CAN_MASTER);
  if(!CAN_MASTER){
    Serial.print(F("CAN_SLAVE_ADDRESS= "));
    Serial.println(CAN_SLAVE_ADDRESS);
  }else {
    Serial.print(F("CAN_MASTER_ADDRESS= "));
    Serial.println(CAN_MASTER_ADDRESS);
  }
  Serial.println(F("------------------------"));
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that sets the chosen channels in the ADC to sweep only those channels.
void set_ADC_Chan(){
  if((modo == 1) || (modo == 2)) {
    mask_Can1 = (0x00FF << (8-cantCan_analog1)) & 0x00FF;
    REG_ADC_WPMR = 0x41444300;                  //Disables the WP of the ADC configuration registers - WPEN=0
    REG_ADC_CHER = mask_Can1;                   //Channels enabled  
    REG_ADC_CHDR = ~mask_Can1;                  //Channels disabled 
    REG_ADC_WPMR = 0x41444301;                  //Enable WP of the ADC configuration registers - WPEN=1
    REG_ADC_WPMR = 0x00000001;                  //WPEN access key
  }
  if((modo == 3) || (modo == 4)) {
    mask_Can2 = (0x3C00 >> (4-cantCan_analog2)) & 0x3C00;
    REG_ADC_WPMR = 0x41444300;                  //Disable the WP of the ADC configuration registers - WPEN=0
    REG_ADC_CHER = mask_Can2;                   //Channels enabled 
    REG_ADC_CHDR = ~mask_Can2;                  //Channels disabled  
    REG_ADC_WPMR = 0x41444301;                  //Enable WP of the ADC configuration registers - WPEN=1 
    REG_ADC_WPMR = 0x00000001;                  //WPEN access key 
  }
}
//--------------------------------------------------------------------------------------------------
  
//--------------------------------------------------------------------------------------------------
//Routine that reads the ADC in sequence
//Each time it executes it reads all set channels

void adc_Srv(){
  if((modo == 1) || (modo == 2)) {
    while((ADC->ADC_ISR & mask_Can1)!= mask_Can1);           //Wait until all conversions are complete. Synchronism. 
    for(contCan_analog1=0;contCan_analog1<cantCan_analog1;contCan_analog1++){
      vect_canales1[contCan_analog1] = ADC->ADC_CDR[7 - contCan_analog1];
    }
  }
  if((modo == 3) || (modo == 4)) {
    while((ADC->ADC_ISR & mask_Can2)!= mask_Can2);           //Wait until all conversions are complete. Synchronism.  
    for(contCan_analog2=0;contCan_analog2<cantCan_analog2;contCan_analog2++){
      vect_canales2[contCan_analog2] = ADC->ADC_CDR[10 + contCan_analog2];
    }
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------

void  set_t_muestreo_Min(){
  if(modo == 1 || modo == 2){
    t_Muestro_Min = (unsigned long)(20000000*cantCan_analog1/delta_B);
  }
  if(modo == 3 || modo == 4){
    t_Muestro_Min = (unsigned long)(20000000*cantCan_analog2/delta_B);
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Serial event receiving data via RS232 from the Arduino board (Terminal)
//Validates the reception of a command

void serialEvent() {
    inChar = Serial.read();
    if (p_Serial == NO_SERIAL || p_Serial == SERIAL_0){
      p_Serial = SERIAL_0;
      delta_B = delta_B_S0;
      in_Comando();
    }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Serial event by which data is received via RS232 (Board's own)
//Validates the reception of a command

void serialEvent1() {
    inChar = Serial1.read();
    if (p_Serial == NO_SERIAL || p_Serial == SERIAL_1){
      p_Serial = SERIAL_1;
      delta_B = delta_B_S1;
      in_Comando();
    }  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Serial event by which data are received via RS485

void serialEvent2() {
    inChar = Serial2.read();
    if (p_Serial == NO_SERIAL || p_Serial == SERIAL_2){
      p_Serial = SERIAL_2;
      delta_B = delta_B_S2;
      in_Comando();
    }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Serial event through which data is received via WiFi (ESP8266 board)

void serialEvent3() {
    inChar = Serial3.read();
    if (p_Serial == NO_SERIAL || p_Serial == SERIAL_3){
      p_Serial = SERIAL_3;
      delta_B = delta_B_WiFi;
      if(Wifi_ini){
        in_Comando();
      }else{
        Serial.print(inChar);
        p_Serial = NO_SERIAL;
      }
    }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Serial event receiving data via native USB

void USB_serialEvent() {
    inChar = SerialUSB.read();
    if (p_Serial == NO_SERIAL || p_Serial == SERIAL_4){
      p_Serial = SERIAL_4;
      delta_B = delta_B_USB;
      in_Comando();
    }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//State machine interpreting the sequence of a command via RS232
//Receives each data in sequence until it finds the end of the command
//Or, the format is: --> ESC Command CR
//If the command is not completed or does not exist, it is timed out.
//If the command exists it executes accordingly
void in_Comando(void){
    comm0_out = TIME_OUT;
    switch(estadosRS){
     case e_espera:
       if(inChar==ESCAPE){
         comm0_flag = true;
         digitalWrite(TST, HIGH);
         estadosRS = e_comando;
       } else {
         p_Serial = NO_SERIAL;
       }
     break;
     case e_comando:
        //Chequeo ASCII de numeros, mayúsculas y minúsculas
       if((inChar>47 && inChar<58) || (inChar>64 && inChar<91) || (inChar>96 && inChar<123)){
         comando = inChar;
         k_g=0;
         estadosRS = e_parseo;
       } else {
         estadosRS = e_espera;
         digitalWrite(TST, LOW);
         comm0_flag = false;
         p_Serial = NO_SERIAL;
       }
     break;
     case e_parseo:
        if (parsing()==LIBRE){
          if(ComandoNoEjecutado_flag == 1){
            serialOutput(F(""), 0, conCR);
            serialOutput(F("Comando no ejecutado"), 0, conCR);
            serialOutput(F(""), 0, conCR);
            ComandoNoEjecutado_flag = 0;
          }
           estadosRS = e_espera;
           digitalWrite(TST, LOW);
           comm0_flag = false;
           p_Serial = NO_SERIAL;
        }
     break;
    }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Parseo of the command in question
bool parsing(void){
  int estado_parsing;
  unsigned long aux;
  String auxString = "";
  
  estado_parsing = OCUPADO;
  switch(comando){
    case '0':                                //Transmits the list of available commands - ( ESC 0 CR)
      help();
      inString = "";
      estado_parsing = LIBRE;
      break;       
#ifdef TEST_VARIOS
    case '1':                               // Records an 8-bit data from E2 and reads it - ( ESC 1 Address CR Data ) - Address --> 000000 to 131071
      if (recibo232() == true) {            //Data --> 000 to 255
        unsigned long direccionE2 = auxbuffer[1] * 100000 + auxbuffer[2] * 10000 + auxbuffer[3] * 1000 + auxbuffer[4] * 100 + auxbuffer[5] * 10 + auxbuffer[6]; 
        byte DataE2 = auxbuffer[7] * 100 + auxbuffer[8] * 10 + auxbuffer[9];
        if(direccionE2 <= 131071){
          Serial.println(F("GRABACION EN E2:"));
          Serial.print(F("Dir:  "));
          Serial.println(direccionE2);
          Serial.print(F("Dato: "));
          Serial.println(DataE2);
          e2prom_write(direccionE2, DataE2);
          Serial.println();
          Serial.println(F("LECTURA DE E2:"));
          Serial.print(F("Dir:  "));
          Serial.println(direccionE2);
          Serial.print(F("Dato: "));
          Serial.println(e2prom_read(direccionE2));
        } else {
          ComandoNoEjecutado_flag = 1;
        }
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case '2':                               //RS232 communication test - ( ESC 2 Char CR ) - Char => any ASCII character
      if (recibo232_a() == true) {
        serialOutput(archivo, 0, conCR);
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case '3':                               //CAN communication test (ESC 3 CR) - Parameter input via monitor
      test_Can_Send();
      inString = "";
      estado_parsing = LIBRE;
      break;
    case '4':                               //CAN communication test in binary - ( ESC 4 CR value ) - Value => 00xxxxxx|000xxxxxx|B0|B1|B2|B3|B4|B5|B6|B7|
      test_Can_Send_Bin();
      inString = "";
      estado_parsing = LIBRE;       
      break;
    case '5':                               //A 16-bit data is read from E2 - ( ESC 5 CR Address ) - Address --> 000000 to 131071
      if (recibo232() == true) {
        unsigned long direccionE2 = auxbuffer[1] * 100000 + auxbuffer[2] * 10000 + auxbuffer[3] * 1000 + auxbuffer[4] * 100 + auxbuffer[5] * 10 + auxbuffer[6]; 
        Serial.print(F("Add:  "));
        Serial.print(direccionE2);
        Serial.print(F(" - "));
        Serial.println(direccionE2+1);
        Serial.print(F("Data: 0x"));
        Serial.println(e2Read_16b(direccionE2), HEX);
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case '6':                               // Saves a fixed data to a file in SD memory - ( ESC Q File CR ) - File format --> 8.3 max.
      if (recibo232_a() == true) {          // without special characters and in lower case.
        grabo_dato_sd(archivo, 'a');
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case '7':                                                       //M90E36A in DMA Mode - ( ESC 7 Qty_Channels CR ) || Qty_Channels = 1 to 5
      if (recibo232() == true) {
          aux=inString.toInt();
          if((aux>=1) && (aux<=5)) Set_DMA_M90((uint8_t)aux);
          inString = "";
          estado_parsing = LIBRE;       
      }
      break;
    case '8':                                                       //Disable DMA on the M90E36A - ( ESC 8 CR )
      digitalWrite(M90_dmaCTRL,LOW);
      delay(8000);
      softwareReset(); 
//      inString = "";
//      estado_parsing = LIBRE;       
      break;
#endif
    case 'A':                               //Receive number of analogue channels - ( ESC A Quantity CR) - Quantity => 1 to 8
      if (recibo232() == true) {
          aux=inString.toInt();
          if(aux<1 || aux>8) {aux = 1;}
          cantCan_analog1 = aux;
          rtc_write((rtc_m_st + 5), cantCan_analog1);
          ini_var();
          set_ADC_Chan();
          inString = "";
          estado_parsing = LIBRE;       
      }
      break;
    case 'B':                               //Receives number of analogue channels type inAmp - ( ESC B Quantity CR) - Quantity => 1 to 4
      if (recibo232() == true) {
          aux=inString.toInt();
          if(aux<1 || aux>4) {aux = 1;}
          cantCan_analog2 = aux;
          rtc_write((rtc_m_st + 6), cantCan_analog2);
          ini_var();
          set_ADC_Chan();
          inString = "";
          estado_parsing = LIBRE;       
      }
      break;
    case 'C':                               //Setting of the amplification of the inAmp - (ESC C Input CR Amplification) - Input => 09 to 12 - Amplification => 0 to 3
      if (recibo232() == true) {
        byte in_inAmp, amplif;
        in_inAmp = auxbuffer[1] * 10 + auxbuffer[2];
        amplif = auxbuffer[3];  
        set_inAmp(in_inAmp, amplif);
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'D':                               //Downloading data from EEProm - ( ESC D CR )
      parada();
      descarga_E2();
      rtc_write(rtc_m_st + 7, 0x02);        //Save on watch download code
      ini_var();                            //Commented until final plaque is available
      inString = "";
      estado_parsing = LIBRE;       
      break;
    case 'E':                               //Receives the working mode of the equipment - ( ESC E CR Mode ) - Mode => 1 to 5
      if (recibo232() == true) {
          aux=inString.toInt();
          if(aux<1 || aux>5) {aux = 1;}
          modo = aux;
          rtc_write((rtc_m_st + 11), modo);
          ini_var();
          set_ADC_Chan();
          inString = "";
          estado_parsing = LIBRE;       
      }
      break;
    case 'F':                               //Save a file name for SD memory - ( ESC F CR file )
      if (recibo232_a() == true) {          //8.3 characters maximum, no special characters and lower case
        strcpy(archivoBack,archivo);
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'G':                               //Deleted EEProm - ( ESC G CR )
      if(modo == 2 || modo == 4) {
        borrado_E2();
      }
      inString = "";
      estado_parsing = LIBRE;
      break;
    case 'H':                                //Setting of the real time clock by I2c - ( ESC H hhmmssDDMMAAAA CR )
      if (recibo232() == true) {
        Set_time();
        inString = "";
        estado_parsing = LIBRE;
      }      
      break;
    case 'I':                               //Set Master/Slave and slave address - (ESC I Master/Slave Dir_Master/Slave CR) 
      if (recibo232() == true) {            //Master/Slave --> 1 / 0 || Dir_Master/Slave --> 001 / 002 a 255
        set_Can_E2();
        ini_var();
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'J':                               // Query the IP of the board - ( ESC J CR )
      serialOutput(F("Mi IP es: "), 0, sinCR);
      serialOutput(getDirIp(), 0, conCR);
      inString = "";
      estado_parsing = LIBRE;
      break;
    case 'L':                               //View files in SD memory - ( ESC L CR)
      listaArcchivosSD();
      inString = "";
      estado_parsing = LIBRE;
    break;
    case 'M':                               //PWM generation - ( ESC M Pin Duty CR) - Pin => 10 to 13 - Duty => 0000 to 4095
      if (recibo232() == true) {
        set_pwm();
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'N':                               //Generation of analogue signals in DACs - ( ESC N Pin Steps CR ) - Pin => 00 to 01 - Steps => 0000 to 4095
      if (recibo232() == true) {
        set_dac();
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'O':                               // transmits data from a file in SD memory - ( ESC OR CR file ) - File format --> 8.3 max.
      if (recibo232_a() == true) {          // without special characters and in lower case.
        leo_arch_sd(archivo);
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'P':                                //Stop device modes - ( ESC P CR )
      parada();
      inString = "";
      estado_parsing = LIBRE;
      break;
    case 'R':                               //Timed recording of the set digital and analogue channels - ( ESC R Time CR ) - Time => 1 to 3600 seconds
      if (recibo232() == true) {
          tiempo_reg = (unsigned int)inString.toInt();
          if(modo == 2 || modo == 4) {
            if(tiempo_reg>=1 && tiempo_reg<=3600 && registrandoM90_flag == 0 && reg_flag == 0 && regHR_flag == 0 && regTodo_flag == 0) {
              parada();
              rtc_write(rtc_m_st + 7, 0x01);                              //Stores in clock start code
              rtc_write(rtc_m_st + 9, byte(tiempo_reg>>0));               //save on clock time recording
              rtc_write(rtc_m_st + 10, byte(tiempo_reg>>8));             //save on clock time recording
              reg_flag = 1;
              rtc_write((rtc_m_st + 8), reg_flag);                        //Saves in clock register flag
              graba_ini_E2();                                             //Store in EEProm log start header
              Timer4.attachInterrupt(Registro_Temp_E2);                 //Timer for registration in E2
              Timer4.start(tiempo_reg * 1000000);
            } else {
              ComandoNoEjecutado_flag = 1;          
            }
          }
          inString = "";
          estado_parsing = LIBRE;       
      }
      break;
    case 'S':                               //Seteo de la salida digital - (ESC S Salida Estado CR) - Salida => 02 a 09 - Estado 0 o 1
      if (recibo232() == true) {
        set_saldig();
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'T':                               //Transmisión temporizada de los canales analógicos seteados - ( ESC T Tiempo CR ) - Tiempo => "t_Muestreo_Min" a 10000000 microsegundos || Tiempo=0 -> una transmisión
      if (recibo232() == true) {            //(En los datos analógicos se transmite primero el byte alto y luego el bajo) || t_Muestreo_Min -->  mínimo tiempo de muestreo de acuerdo al ancho de banda disponible
          aux=(unsigned long)inString.toInt();
          if(aux<=10000000 && modo != 0 && registrandoM90_flag == 0 && reg_flag == 0 && regHR_flag == 0 && regTodo_flag == 0) {
            p_SerialTmp = p_Serial;
            if(aux==0){
              Transm_Temp();  
            }else{
              set_t_muestreo_Min();
              if((modo ==1 || modo == 3) && aux >= t_Muestro_Min) {
                Timer3.attachInterrupt(Transm_Temp);                      //Timer de transmisiones temporizadas
                Timer3.start(aux);
              }
            }
          } else {
            ComandoNoEjecutado_flag = 1;       
          }
          inString = "";
          estado_parsing = LIBRE;
      }
      break;
    case 'U':                               //Transmits the status of the digital channels - (ESC U CR)
      serialOutput("", leo_can_dig(), sinCR);
      inString = "";
      estado_parsing = LIBRE;       
      break;
    case 'W':                               //Set Network and Password for WiFi - (ESC W Network 0xFF Pass 0xFF CR) - Network and Pass max. 15 ASCII characters each
      if (recibo232_a() == true) {
        set_Net_Pas();
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'X':                               //Wifi Network and Key Query - (ESC X CR)
      leo_Net_Pas();
      inString = "";
      estado_parsing = LIBRE;
      break;
    case 'Z':                               //Reset by sotware - ( ESC Z CR )
      softwareReset();
      break;
    case 'a':                               //Enables status check of M90E36A registers - (ESC to CR)
      p_SerialTmp = p_Serial;
      Timer1.attachInterrupt(Check_M90E36A);
      Timer1.start(CONTROL_TIME);
      inString = "";
      estado_parsing = LIBRE;
      break;
    case 'b':                               //Calibration of the M90E36A
      Calib_M90();
      inString = "";
      estado_parsing = LIBRE;
      break;       
    case 'd':                                 //Reads a range of records - ( ESC d Reg_ini Reg_fin CR ) - Records --> 000 to 465 in decimal
      if (recibo232() == true) {
        uint16_t reg_ini = auxbuffer[1] * 100 + auxbuffer[2] * 10 + auxbuffer[3];  
        uint16_t reg_fin = auxbuffer[4] * 100 + auxbuffer[5] * 10 + auxbuffer[6];  
        read_M90_rango(reg_ini, reg_fin);
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'e':                               //Transmits device data - ( ESC and CR ) - Responds with basic project data
      datos_equipo();
      inString = "";
      estado_parsing = LIBRE;       
      break;
    case 'f':                               //M90E36A variables timed recording - ( ESC f Time CR ) - Time => 333ms to 60000ms
      if (recibo232() == true) {
        tiempo_reg = (unsigned int)inString.toInt();
        if(tiempo_reg>=333 && tiempo_reg<=60000 && registrandoM90_flag == 0 && reg_flag == 0 && regHR_flag == 0 && regTodo_flag == 0) {
          if(preparo_Registro() == true){
            Timer4.attachInterrupt(Reg_M90_Ram);                       //Timer for RAM logging
            Timer4.start(tiempo_reg * 1000);
          }
        } else {
          ComandoNoEjecutado_flag = 1;
        }
        inString = "";
        estado_parsing = LIBRE;       
      }
      break;
    case 'g':                               //Timed recording of the harmonic components in % of the M90E36A - ( ESC g MaxCompArm CR ) - MaxCompArm => 2 to 32
      if (recibo232() == true) {
        MaxCompArm = (uint8_t)inString.toInt();
        if(MaxCompArm >= 2 && MaxCompArm <= 32 && registrandoM90_flag == 0 && reg_flag == 0 && regHR_flag == 0 && regTodo_flag == 0) {
          if(preparo_RegistroHR() == true){
            write_M90_1R(DFT_CTRL, 0x0001);     //Start the DFT module for the calculation of harmonics per channel       
          }
        } else {
          ComandoNoEjecutado_flag = 1;
        }
        inString = "";
        estado_parsing = LIBRE;       
      }
      break;
    case 'h':                               //Transmits day and time of the RTC - ( ESC h CR ) - Responds with day and time of the device
      get_time();
      prt_time();
      inString = "";
      estado_parsing = LIBRE;       
      break;
    case 'i':                              // Timed recording of the variables and harmonic components in % of the M90E36A - ( ESC i MaxCompArm CR ) - MaxCompArm => 2 to 32
      if (recibo232() == true) {
        MaxCompArm = (uint8_t)inString.toInt();
        if(MaxCompArm >= 2 && MaxCompArm <= 32 && registrandoM90_flag == 0 && reg_flag == 0 && regHR_flag == 0 && regTodo_flag == 0) {
          if(preparo_RegistroTodo() == true){
            write_M90_1R(DFT_CTRL, 0x0001);     //Start the DFT module for the calculation of harmonics per channel      
          }
        } else {
          ComandoNoEjecutado_flag = 1;
        }
        inString = "";
        estado_parsing = LIBRE;       
      }
      break;
    case 'r':                               //Read a register - ( ESC r CR Register ) - Register --> 000 to 465 in decimal
      if (recibo232() == true) {
        uint16_t registro = auxbuffer[1] * 100 + auxbuffer[2] * 10 + auxbuffer[3];  
        Serial.println();
        Serial.println(F("------------------------------"));
        Serial.println(F("Lectura de un registro"));
        Serial.print(F("Registro 0x"));
        Serial.print(registro,HEX);
        Serial.print(F(" -->  0x"));
        Serial.println(read_M90_1RC(registro),HEX);
        Serial.println(F("------------------------------"));
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 't':                              //Reads and transmits voltage, current, active and reactive power registers - ( ESC t CR )
      Transmite_VI_PQS();
      inString = "";
      estado_parsing = LIBRE;
      break;
    case 'u':                               //Transmits the harmonic components in % of the M90E36A - MaxCompArm => 2 to 32 - ( ESC u MaxCompArm CR )
      if (recibo232() == true) {
        MaxCompArm = (uint8_t)inString.toInt();
        if(MaxCompArm >= 2 && MaxCompArm <= 32 && registrandoM90_flag == 0 && reg_flag == 0 && regHR_flag == 0 && regTodo_flag == 0) {
          write_M90_1R(DFT_CTRL, 0x0001);       //Arranca el módulo DFT para el cálculo de armónicas por canal       
          while((read_M90_1R(DFT_CTRL) & 0x0001) == 1){;}
          Transmite_HRV();
          Transmite_HRI();
        } else {
          ComandoNoEjecutado_flag = 1;
        }
        inString = "";
        estado_parsing = LIBRE;       
      }      
      break;
    case 'w':                              //Write a record - ( ESC w Record data CR ) - Record --> 000 to 465 in decimal || Data --> 00000 to 65535 in decimal
      if (recibo232() == true) {
        uint16_t registro = auxbuffer[1] * 100 + auxbuffer[2] * 10 + auxbuffer[3];  
        uint16_t dato = auxbuffer[4] * 10000 + auxbuffer[5] * 1000 + auxbuffer[6] * 100 + auxbuffer[7] * 10 + auxbuffer[8];
        write_M90_1RC(registro, dato);
        inString = "";
        estado_parsing = LIBRE;
      }
      break;
    case 'z':
      write_M90_1R(SoftReset, RESET_M90);
      delay(500);
      iniReg_M90_Ram();
      inString = "";
      estado_parsing = LIBRE;
      break;
    default:
      serialOutput(F("Comando no reconocido"), 0, conCR);    
      inString = "";
      estado_parsing = LIBRE;
      break;
  }
  return estado_parsing;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Transmite la ayuda de comando
void help(void){
  serialOutput("", 0, conCR);
  serialOutput(F("Command list ESC:"), 0, conCR);
  serialOutput("", 0, conCR);
  serialOutput(F("-> (ESC 0 CR ) - Command help"), 0, conCR);
#ifdef TEST_VARIOS
  serialOutput(F("-> (ESC 1 Address Data CR) -  Write and read in the EEprom - Address => 000000 a 131071 - Data => 000 a 255"), 0, conCR);
  serialOutput(F("-> (ESC 2 char CR) - RS232 Communication test  - char --> any character ASCII"), 0, conCR);
  serialOutput(F("-> (ESC 3 CR) - CAN communication test - input of parameters via serial monitor"), 0, conCR);
  serialOutput(F("-> (ESC 4 value CR) - CAN communication test in binary - value => 00xxxxxx|000xxxxxx|B0|B1|B2|B3|B4|B5|B6|B7|"), 0, conCR);
  serialOutput(F("-> (ESC 5 Address CR) - Reads from E2 a 16 bit data | Dirección --> 000000 a 131071"), 0, conCR);
  serialOutput(F("-> (ESC 6 Archive CR ) - Record a fixed data in a file in SD memory. - file format --> 8.3 max"), 0, conCR);
  serialOutput(F("-> (ESC 7 number of channels CR) - Activate DMA mode of the M90E36A with 1, 2, 3, 4, 5 channels - 1 voltage and up to 4 current "), 0, conCR);
  serialOutput(F("-> (ESC 8 CR) - Disables DMA mode of the M90E36A"), 0, conCR);
#endif
  serialOutput(F("-> (ESC A Number CR) - Receives number of analog channels - Number => 1 a 8"), 0, conCR);
  serialOutput(F("-> (ESC B Number CR) - Receives number of current analog channels - Number => 1 a 4"), 0, conCR);
  serialOutput(F("-> (ESC C Input Amplification CR) - Setting of the amplification of the inAmp - Input => 09 a 12 - Amplification => 0 a 3"), 0, conCR);
  serialOutput(F("-> (ESC D CR ) -  Downloading data from the EEProm"), 0, conCR);
  serialOutput(F("-> (ESC E Modo CR ) - Working mode - mode => 1 a 5"), 0, conCR);
  serialOutput(F("-> (ESC F Archivo CR ) - Save a file name for SD memory"), 0, conCR);
  serialOutput(F("->                      8.3 characters maximum ,no special characters and lowercase"), 0, conCR);
  serialOutput(F("-> (ESC G CR ) - Clear EEProm"), 0, conCR);
  serialOutput(F("-> (ESC H hhmmssDDMMAAAA CR )"), 0, conCR);
  serialOutput(F("-> (ESC I Master/Slave Dir_Master/Slave CR) - set Master/Slave and slave address"), 0, conCR);
  serialOutput(F("->                                            Master/Slave --> 1 / 0 || Dir_Master/Slave --> 001 / 002 a 255"), 0, conCR);
  serialOutput(F("-> (ESC J CR ) - Query ip of the board"), 0, conCR);
  serialOutput(F("-> (ESC L CR) - View files in the SD memory"), 0, conCR);
  serialOutput(F("-> (ESC M Pin Duty CR) - PWM generation - Pin => 10 al 13 - Duty => 000 a 255"), 0, conCR); 
  serialOutput(F("-> (ESC N Pin Steps CR - Analogue signal generation in DACs - Pin => 00 al 01 - Steps => 0000 a 4095)"), 0, conCR);
  serialOutput(F("-> (ESC O Archivo CR ) - Transmits data of a file in SD memory - File format --> 8.3 max"), 0, conCR);
  serialOutput(F("->                       (without special characters and in lower case)"), 0, conCR);
  serialOutput(F("-> (ESC P CR ) - Stop"), 0, conCR);
  serialOutput(F("-> (ESC R Time CR ) - Timed recording of set digital and analogue channels - Time => 1 a 3600 segundos)"), 0, conCR);
  serialOutput(F("-> (ESC S Output State CR) - Setting the digital output - output => 02 a 09 - State 0 o 1)"), 0, conCR);
  serialOutput(F("-> (ESC T Time CR ) - Timed transmission of preset analogue channels - "), 0, conCR);
  serialOutput(F("->                      Time => t_Muestreo_Min a 10000000 microsegundos || Time=0 -> una transmisión"), 0, conCR);
  serialOutput(F("->                       In analogue data, the high byte is transmitted first and then the low byte || "), 0, conCR);
  serialOutput(F("->                      t_sampling_Min -->  minimum sampling time according to available bandwidth"), 0, conCR);
  serialOutput(F("-> (ESC U CR) - Transmits the status of digital channels"), 0, conCR);
  serialOutput(F("-> (ESC W Red 0xFF Pass 0xFF CR) - Network and WiFi Password Setting - Network and Pass maximum 15 ASCII characters each"), 0, conCR);
  serialOutput(F("-> (ESC X CR ) - Query the stored Network and Wifi Password"), 0, conCR);
  serialOutput(F("-> (ESC Z CR) - Reset por sotware"), 0, conCR);
  serialOutput(F("-> (ESC a CR) - Enables timed integrity check of the M90E36A registers"), 0, conCR);
  serialOutput(F("-> (ESC b CR) - Calibration of the M90E36A"), 0, conCR);
  serialOutput(F("-> (ESC d reg1 reg2 CR) - Reads a range of records from the M90E36A || reg1 y reg2 --> 000 a 465 in decimal"), 0, conCR);
  serialOutput(F("-> (ESC e CR ) - Transmits equipment data - Respond with the basic data of the project)"), 0, conCR);
  serialOutput(F("-> (ESC f time CR ) - Timed recording of the M90E36A variables - Time => 333ms a 60000ms"), 0, conCR);
  serialOutput(F("-> (ESC g maxharmcomp CR ) - Timed recording of the harmonic components in % of the M90E36A - maxharmcomp => 2 a 32"), 0, conCR);
  serialOutput(F("-> (ESC h CR ) - Transmits day and time of the RTC  -  Respond with day and time of equipment"), 0, conCR);
  serialOutput(F("-> (ESC i MaxCompArm CR ) - Timed recording of variables and harmonic components in % of the M90E36A"), 0, conCR);
  serialOutput(F("    MaxCompharm => 2 a 32"), 0, conCR);
  serialOutput(F("-> (ESC r Registro CR) - Read a record from the M90E36A || Register --> 000 a 465 en decimal"), 0, conCR);
  serialOutput(F("-> (ESC t CR) - Transmits the measured values of voltages, currents, powers, etc. of the M90E36A"), 0, conCR);
  serialOutput(F("-> (ESC u Maxcompharm CR ) - Transmits harmonic components in % of the M90E36A - MaxcompArm => 2 a 32"), 0, conCR);
  serialOutput(F("-> (ESC w Register CR) - Write a record of the M90E36A || Register --> 000 a 465 in decimal || Data --> 00000 a 65535"), 0, conCR);
  serialOutput(F("-> (ESC z CR) - Reset and reset of the M90E36A"), 0, conCR);
  serialOutput("", 0, conCR);
}

//--------------------------------------------------------------------------------------------------
//Recibe un dato en inChar y llena la cadena inString y el vector auxbuffer hasta recibir IN_END (ox0d)
//Solo recibe caracteres alfanuméricos, los demas los descarta (auxbuffer comienza en [1])
bool recibo232(void){
  bool s = false;

  if (isDigit(inChar)) {
    inString += (char)inChar;
    if(k_g < VECT_END){
      k_g++;
      inChar = inChar - 48;
      auxbuffer[k_g] = inChar;
    }
  }
  if (inChar == IN_END) {
    s = true;
  }
  return s;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Receives a data in inChar and fills the file vector until IN_END (ox0d) is received.
//Receives any character and puts null-terminator at the end (file starts at [0])
bool recibo232_a(void){
  bool s = false;

  if(k_g < LARGO_ARCH){
    archivo[k_g] = inChar;
    k_g++;
  }

  if (inChar == IN_END) {
    k_g--;
    archivo[k_g] = (byte)NULL;
    s = true;
  }
  return s;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Generic serial output. Sends a String or a binary data, with or without CR/LF.
//If data="" send the binary, otherwise send data. Also newLine is either withCR or withoutCR.


void serialOutput(String data, byte binario, bool newLine) {

  switch (p_Serial) {
    case SERIAL_0:
      if(data.length()!=0) {
        Serial.print(data);
      } else {
        Serial.write(binario);
      }
      if (newLine == true) { Serial.println(); }
      break;
    case SERIAL_1:
      if(data.length()!=0) {
        Serial1.print(data);
      } else {
        Serial1.write(binario);
      }
      if (newLine == true) { Serial1.println(""); }
      break;
    case SERIAL_2:
      digitalWrite(RS485_DIR, RS485_Tx);
      if(data.length()!=0) {
        Serial2.print(data);
      } else {
        Serial2.write(binario);
      }
      if (newLine == true) { Serial2.println(""); }
      digitalWrite(RS485_DIR, RS485_Rx);
      break;
    case SERIAL_3:
      wifiSendTimeout = 1000;
      if(data.length()!=0) {
        envioWifi(data);
      } else {
        envioWifi_Bin(binario);                     
      }
      if (newLine == true) { envioWifi("\r\n"); }
      delay(10);
      break;
    case SERIAL_4:
      if(data.length()!=0) {
        SerialUSB.print(data);
      } else {
        SerialUSB.write(binario);
      }
      if (newLine == true) { SerialUSB.println(""); }
      break;
      default:
        ;
      break;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Generic serial output. A String or a binary data is sent, with or without CR/LF
//for timed events.
//If data="" it sends the binary, otherwise it sends data. Also newLine is either withCR or withoutCR.
void serialOutputTmp(String data, byte binario, bool newLine) {

  switch (p_SerialTmp) {
    case SERIAL_0:
      if(data.length()!=0) {
        Serial.print(data);
      } else {
        Serial.write(binario);
      }
      if (newLine == true) { Serial.println(); }
      break;
    case SERIAL_1:
      if(data.length()!=0) {
        Serial1.print(data);
      } else {
        Serial1.write(binario);
      }
      if (newLine == true) { Serial1.println(""); }
      break;
    case SERIAL_2:
      digitalWrite(RS485_DIR, RS485_Tx);
      if(data.length()!=0) {
        Serial2.print(data);
      } else {
        Serial2.write(binario);
      }
      if (newLine == true) { Serial2.println(""); }
      digitalWrite(RS485_DIR, RS485_Rx);
      break;
    case SERIAL_3:
      wifiSendTimeout = 1000;
      if(data.length()!=0) {
        envioWifi(data);
      } else {
        envioWifi_Bin(binario);                  
      }
      if (newLine == true) { envioWifi("\r\n"); }
      delay(10);
      break;
    case SERIAL_4:
      if(data.length()!=0) {
        SerialUSB.print(data);
      } else {
        SerialUSB.write(binario);
      }
      if (newLine == true) { SerialUSB.println(""); }
      break;
    default:
      ;
      break;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//----------------------- CAN Driver ---------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//Setting the device as Master or Slave 
void set_Can_E2(void){
  byte aux;
  
  e2prom_write(e2_CanMasterSlave, auxbuffer[1]);
  aux = auxbuffer[2] * 100 + auxbuffer[3] * 10 + auxbuffer[4];
  e2prom_write(e2_CanDirMasterSlave, aux);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Decoding of the message received by CAN Bus
bool CanMessParsing(void){
  bool resultado = true;
  unsigned int funcion;

  funcion = (byte)(canBuffer_In.id >> 5);
  Serial.print("funcion: ");
  Serial.println(funcion);

  switch(funcion){
    case CAN_ERR:
      if(CAN_ERR_EXEC() == false){
        Serial.print(F("Error en la ejecucion del ERROR"));
      }
      break;
    case CAN_DO:
      Serial.println("CAN_DO");
      if(CAN_DO_EXEC() == false){
        Serial.print(F("Error en la ejecucion de la ACCION"));
      }
      break;
    case CAN_SET:
      if(CAN_SET_EXEC() == false){
        Serial.print(F("Error en la ejecucion de la CONFIGURACION"));
      }
      break;
    case CAN_QRY:
      if(CAN_QRY_EXEC() == false){
        Serial.print(F("Error en la ejecucion de la CONSULTA"));
      }
      break;
    case CAN_ACK:
      if(CAN_ACK_EXEC() == false){
        Serial.print(F("Error en la ejecucion del ACK"));
      }
      break;
    case CAN_POST:
      if(CAN_POST_EXEC() == false){
        Serial.print(F("Error en la ejecucion de la ANUNCIO"));
      }
      break;
    case CAN_HB:
      if(CAN_HB_EXEC() == false){
        Serial.print(F("Error en la ejecucion de la PRESENCIA"));
      }
      break;
    default:
      resultado = false;
      break;
  }
  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
bool CAN_ERR_EXEC(void){
  bool resultado = true;

  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
bool CAN_DO_EXEC(void){
  bool resultado = true;
  byte variable;

  variable = (byte)(canBuffer_In.data.low >> 16);
  Serial.print("Variable: "); 
  Serial.println(variable); 
  
  switch(variable){
    case CAN_DIGITAL_OUT:                                                 //Setting a digital output channel
      Serial.println(F("DO-DIGITAL_OUT"));
      auxbuffer[2] = (byte) (canBuffer_In.data.low >> 8);
      auxbuffer[3] = (byte) canBuffer_In.data.low; 
      Serial.println(auxbuffer[2]);
      Serial.println(auxbuffer[3]);
      if(auxbuffer[2] > 1 && auxbuffer[2] < 10 && auxbuffer[3] < 2 ) set_saldig();
    break;
    case CAN_ANALOG_OUT:                                                  //Setting an analogue output channel
      byte pinDac;
      unsigned int valor;
      Serial.println(F("DO-ANALOG_OUT"));
      pinDac = ((byte)(canBuffer_In.data.low >> 8)) + 66;
      Serial.println(pinDac);
      if(pinDac > 65 && pinDac < 68){
        valor = (((byte)canBuffer_In.data.low) * 256) + ((byte)(canBuffer_In.data.high >> 24));
        Serial.print(valor);
        analogWrite(pinDac, valor);
      }
    break;
    case CAN_PWM:                                                          //PWM output setting
      byte pinPwm;
      unsigned int dtyCicle;
      Serial.println(F("DO-PWM"));
      pinPwm = (byte)(canBuffer_In.data.low >> 8);
      if(pinPwm > 9 && pinPwm < 14){               //Pin13 (led) and pins 0 and 1 (Rx0 and Tx0) are not used.
        dtyCicle = (byte)canBuffer_In.data.low;
        analogWrite(pinPwm, dtyCicle);
      }else{
        resultado = false;
      }
    break;
    case CAN_PAR:                                         //Stop equipment modes
      Serial.println(F("DO-PARADA"));
      parada();
    break;
    case CAN_SRST:                                        //Remotely resetting the equipment
      Serial.println(F("DO-RESET"));
      softwareReset();
    break;
    default:
      resultado = false;
    break;
  }
  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
bool CAN_SET_EXEC(void){
  bool resultado = true;
  byte variable, aux_dato, aux1_dato;

  variable = (byte)(canBuffer_In.data.low >> 16);
  Serial.print("Variable: "); 
  Serial.println(variable); 
  
  switch(variable){
    case CAN_ANA:                                       //Setting of the number of analogue automation channels
      Serial.println(F("SET-ANA"));
      aux_dato = (byte)(canBuffer_In.data.low >> 8);
      Serial.println(aux_dato);
      if(aux_dato>=1 && aux_dato<=8) {
        cantCan_analog1 = aux_dato;
        rtc_write((rtc_m_st + 5), cantCan_analog1);
        ini_var();
        set_ADC_Chan();
      } else {
        resultado = false;        
      }
    break;
    case CAN_INAMP:                                     //Setting of the number of analogue instrumentation channels
      Serial.println(F("SET-INAMP"));
      aux_dato = (byte)(canBuffer_In.data.low >> 8);
      Serial.println(aux_dato);
      if(aux_dato>=1 && aux_dato<=4){
        cantCan_analog2 = aux_dato;
        rtc_write((rtc_m_st + 6), cantCan_analog2);
        ini_var();
        set_ADC_Chan();
      }else {
        resultado = false;
      }
    break;
    case CAN_AINAMP:                                     //Setting the amplification of the instrumentation channels
      Serial.println(F("SET-AINAMP"));
      aux_dato = (byte)(canBuffer_In.data.low >> 8);
      aux1_dato = (byte)(canBuffer_In.data.low);
      Serial.println(aux_dato);
      Serial.println(aux1_dato);
      if(set_inAmp(aux_dato, aux1_dato) == false){
        resultado = false;
      }
    break;
    case CAN_MOD:                                       //Setting the working mode of the board
      Serial.println(F("MOD"));
      aux_dato = (byte)(canBuffer_In.data.low >> 8);
      Serial.println(aux_dato);
      if(aux_dato>=1 && aux_dato<=4){
        modo = aux_dato;
        rtc_write((rtc_m_st + 11), modo);
        ini_var();
        set_ADC_Chan();
      }else{
        resultado = false;
      }
    break;
    case CAN_RTC:                                       //Setting the RTC (hhmmss || DDMMAA)
      Serial.println(F("SET-RTC"));
      if(rtc_flag == false) {
        rtc_flag = true;      
        auxbuffer[1] = (byte)(canBuffer_In.data.low >> 8);
        auxbuffer[2] = (byte)(canBuffer_In.data.low);
        auxbuffer[3] = (byte)(canBuffer_In.data.high >> 24);
        auxbuffer[4] = (byte)(canBuffer_In.data.high >> 16);
        auxbuffer[5] = (byte)(canBuffer_In.data.high >> 8);
        auxbuffer[6] = (byte)(canBuffer_In.data.high);
      }else{
        rtc_flag = false;      
        auxbuffer[7] = (byte)(canBuffer_In.data.low >> 8);
        auxbuffer[8] = (byte)(canBuffer_In.data.low);
        auxbuffer[9] = (byte)(canBuffer_In.data.high >> 24);
        auxbuffer[10] = (byte)(canBuffer_In.data.high >> 16);
        auxbuffer[11] = (byte)2;
        auxbuffer[12] = (byte)0;
        auxbuffer[13] = (byte)(canBuffer_In.data.high >> 8);
        auxbuffer[14] = (byte)(canBuffer_In.data.high);
        if(Set_time() == false) {resultado = false;}
      }
    break;
    default:
      resultado = false;
    break;
  }
  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
bool CAN_QRY_EXEC(void){
  uint8_t data[8]  = {0,0,0,0,0,0,0,0};
  uint32_t id_mensaje = 0;
  bool resultado = true;
  byte variable, canal_CAN;
  uint16_t aux;
  
  variable = (byte)(canBuffer_In.data.low >> 16);
  Serial.print("Variable: "); 
  Serial.println(variable); 

  id_mensaje = (uint32_t)(CAN_ACK << 5) | CAN_SLAVE_ADDRESS;          //Responds with an ACK message and the requested value
  data[0] = CAN_MASTER_ADDRESS;
  data[1] = variable;
  
  switch(variable){
    case CAN_DIGITAL_IN:                                              //Request to read digital channels. Reply by ACK
      Serial.println(F("QRY-DIGITAL_IN"));
      data[2] = leo_can_dig();                                        //All digital signals in a single byte
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_ANALOG_IN:                                               //Request to read an analogue channel. ACK reply
      canal_CAN = (byte)(canBuffer_In.data.low >> 8);
      Serial.print("Canal: "); 
      Serial.println(canal_CAN); 
      Serial.println(F("QRY-ANALOG_IN"));
      data[2] = canal_CAN;                      
      adc_Srv();
      if(((modo == 1) || (modo == 2)) && (canal_CAN < cantCan_analog1)){
        data[3] = (byte)(vect_canales1[canal_CAN] >> 8);    //Analogue channel high end    
        data[4] = (byte)vect_canales1[canal_CAN];           //Analogue channel low end
        can_Send_8B(id_mensaje, data);
      } else if(((modo == 3) || (modo == 4)) && (canal_CAN < cantCan_analog2)) {
        data[3] = (byte)(vect_canales2[canal_CAN] >> 8);    //High end of inAmp channel     
        data[4] = (byte)vect_canales2[canal_CAN];           //Lower part of the inAmp channel
        can_Send_8B(id_mensaje, data);
      } else {
        resultado = false;
      }
    break;
    case CAN_RTC:                                                     //Request to read the RTC. Response by ACK
      Serial.println(F("QRY-RTC"));
      get_time();
      for(int h=1;h<=6;h++){
        data[h+1] = auxbuffer[h] - 48;
      }
      can_Send_8B(id_mensaje, data);
      delay(1);
//      id_mensaje = (((uint32_t)CAN_ACK) << 5) | CAN_SLAVE_ADDRESS; 
//      data[0] = CAN_MASTER_ADDRESS;
//      data[1] = CAN_RTC;
      for(int h=7;h<=12;h++){
        data[h-5] = auxbuffer[h] - 48;
      }
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_U_A:                                                   //Request to read UrmsA from M90E36A. Response by ACK
      Serial.println(F("QRY-U_A"));
      aux = read_M90_1RC(UrmsA);
      data[2] = (byte)(aux >> 8);                                   //High-high part of the variable     
      data[3] = (byte)(aux);                                        //High-low part of the variable
      aux = read_M90_1RC(UrmsALSB);
      data[4] = (byte)(aux >> 8);                                   // Low-high part of the variable    
      data[5] = (byte)(aux);                                        //Low-low part of the variable
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_U_B:                                                   
      Serial.println(F("QRY-U_B"));
      aux = read_M90_1RC(UrmsB);
      data[2] = (byte)(aux >> 8);                                     
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(UrmsBLSB);
      data[4] = (byte)(aux >> 8);                                       
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_U_C:                                                   
      Serial.println(F("QRY-U_C"));
      aux = read_M90_1RC(UrmsC);
      data[2] = (byte)(aux >> 8);                                   
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(UrmsCLSB);
      data[4] = (byte)(aux >> 8);                                       
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_I_A:                                                   
      Serial.println(F("QRY-I_A"));
      aux = read_M90_1RC(IrmsA);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(IrmsALSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_I_B:                                                   
      Serial.println(F("QRY-I_B"));
      aux = read_M90_1RC(IrmsB);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(IrmsBLSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_I_C:                                                   
      Serial.println(F("QRY-I_C"));
      aux = read_M90_1RC(IrmsC);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(IrmsCLSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_I_N1:                                                  
      Serial.println(F("QRY-I_C"));
      aux = read_M90_1RC(IrmsN1);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PA_A:                                                  
      Serial.println(F("QRY-PA_A"));
      aux = read_M90_1RC(PmeanA);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(PmeanALSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PA_B:                                                  
      Serial.println(F("QRY-PA_B"));
      aux = read_M90_1RC(PmeanB);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(PmeanBLSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PA_C:                                                  
      Serial.println(F("QRY-PA_C"));
      aux = read_M90_1RC(PmeanC);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(PmeanCLSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PA_TOT:                                                
      Serial.println(F("QRY-PA_TOT"));
      aux = read_M90_1RC(PmeanT);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(PmeanTLSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PR_A:                                                  
      Serial.println(F("QRY-PR_A"));
      aux = read_M90_1RC(QmeanA);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                       
      aux = read_M90_1RC(QmeanALSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PR_B:                                                 
      Serial.println(F("QRY-PR_B"));
      aux = read_M90_1RC(QmeanB);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(QmeanBLSB);
      data[4] = (byte)(aux >> 8);                                       
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PR_C:                                                  
      Serial.println(F("QRY-PR_C"));
      aux = read_M90_1RC(QmeanC);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(QmeanCLSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PR_TOT:                                                
      Serial.println(F("QRY-PR_TOT"));
      aux = read_M90_1RC(QmeanT);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(QmeanTLSB);
      data[4] = (byte)(aux >> 8);                                       
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PS_A:                                                  
      Serial.println(F("QRY-PS_A"));
      aux = read_M90_1RC(SmeanA);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(SmeanALSB);
      data[4] = (byte)(aux >> 8);                                       
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PS_B:                                                  
      Serial.println(F("QRY-PS_B"));
      aux = read_M90_1RC(SmeanB);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(SmeanBLSB);
      data[4] = (byte)(aux >> 8);                                     
      data[5] = (byte)(aux);                                       
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PS_C:                                                  
      Serial.println(F("QRY-PS_C"));
      aux = read_M90_1RC(SmeanC);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(SmeanCLSB);
      data[4] = (byte)(aux >> 8);                                      
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_PS_TOT:                                                
      Serial.println(F("QRY-PS_TOT"));
      aux = read_M90_1RC(SAmeanT);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      aux = read_M90_1RC(SAmeanTLSB);
      data[4] = (byte)(aux >> 8);                                       
      data[5] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_FP_A:                                                  
      Serial.println(F("QRY-FP_A"));
      aux = read_M90_1RC(PFmeanA);
      data[2] = (byte)(aux >> 8);                                   
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_FP_B:                                                  
      Serial.println(F("QRY-FP_B"));
      aux = read_M90_1RC(PFmeanB);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_FP_C:                                                 
      Serial.println(F("QRY-FP_C"));
      aux = read_M90_1RC(PFmeanC);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_FP_TOT:                                                
      Serial.println(F("QRY-FP_TOT"));
      aux = read_M90_1RC(PFmeanT);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_THDU_A:                                                
      Serial.println(F("QRY-THDU_A"));
      aux = read_M90_1RC(THDNUA);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_THDU_B:                                               
      Serial.println(F("QRY-THDU_B"));
      aux = read_M90_1RC(THDNUB);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_THDU_C:                                                
      Serial.println(F("QRY-THDU_C"));
      aux = read_M90_1RC(THDNUC);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_THDI_A:                                                
      Serial.println(F("QRY-THDI_A"));
      aux = read_M90_1RC(THDNIA);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_THDI_B:                                                
      Serial.println(F("QRY-THDI_B"));
      aux = read_M90_1RC(THDNIB);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_THDI_C:                                                
      Serial.println(F("QRY-THDI_C"));
      aux = read_M90_1RC(THDNIC);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_FREC:                                                  
      Serial.println(F("QRY-FREC"));
      aux = read_M90_1RC(M90_Freq);
      data[2] = (byte)(aux >> 8);                                      
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    case CAN_TEMP:                                                  
      Serial.println(F("QRY-TEMP"));
      aux = read_M90_1RC(M90_Temp);
      data[2] = (byte)(aux >> 8);                                       
      data[3] = (byte)(aux);                                        
      can_Send_8B(id_mensaje, data);
    break;
    default:
      resultado = false;
    break;
  }
  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
/// ACKs can come empty or with data from a QRY response.
//Specific procedure for function #### MASTER #### only for TESTING

bool CAN_ACK_EXEC(void){
  bool resultado = true;
  byte variable, valor_dig, canal_analog, caracter;
  unsigned int valor_analog;
  float M90_Var;
   
  variable = (byte)(canBuffer_In.data.low >> 16);
  Serial.print("Variable: "); 
  Serial.println(variable); 
  
  switch(variable){
    case CAN_DIGITAL_IN:                                          //Receiving the digital channels as MASTER
      Serial.println("ACK-DIGITAL_IN");
      valor_dig = (byte) (canBuffer_In.data.low >> 8);
      Serial.print(F("Entradas digitales: "));
      Serial.println(valor_dig, BIN);
    break;
    case CAN_ANALOG_IN:                                           //Reception of an analogue channel as MASTER
      Serial.println("ACK-ANALOG_IN");
      canal_analog = (byte) (canBuffer_In.data.low >> 8);
      Serial.print(F("Canal_analog: "));
      Serial.println(canal_analog);
      valor_analog = (((unsigned int)(canBuffer_In.data.low & 0x000000ff)) << 8) | (unsigned int)(canBuffer_In.data.high >> 24);
      Serial.print(F("Steps_analog: "));
      Serial.println(valor_analog);
    break;
    case CAN_U_A:                                                 //Receiving Ua channel as MASTER
      Serial.println("ACK-CAN_U_A");
      M90_Var = 0.01 * (uint16_t)(canBuffer_In.data.low & 0x0000ffff) + (uint16_t)(canBuffer_In.data.high >> 24) / 256;
      Serial.print(F("UA(V): "));
      Serial.println(M90_Var);
    break;
    case CAN_U_B:                                                 
      Serial.println("ACK-CAN_U_B");
      M90_Var = 0.01 * (uint16_t)(canBuffer_In.data.low & 0x0000ffff) + (uint16_t)(canBuffer_In.data.high >> 24) / 256;
      Serial.print(F("UB(V): "));
      Serial.println(M90_Var);
    break;
    case CAN_U_C:                                                 
      Serial.println("ACK-CAN_U_C");
      M90_Var = 0.01 * (uint16_t)(canBuffer_In.data.low & 0x0000ffff) + (uint16_t)(canBuffer_In.data.high >> 24) / 256;
      Serial.print(F("UC(V): "));
      Serial.println(M90_Var);
    break;
    case CAN_I_A:                                                
      Serial.println("ACK-CAN_I_A");
      M90_Var = 0.001 * (uint16_t)(canBuffer_In.data.low & 0x0000ffff) + (uint16_t)(canBuffer_In.data.high >> 24) / 256;
      Serial.print(F("IA(A): "));
      Serial.println(M90_Var);
    break;
    case CAN_I_B:                                                 
      Serial.println("ACK-CAN_I_B");
      M90_Var = 0.001 * (uint16_t)(canBuffer_In.data.low & 0x0000ffff) + (uint16_t)(canBuffer_In.data.high >> 24) / 256;
      Serial.print(F("IB(A): "));
      Serial.println(M90_Var);
    break;
    case CAN_I_C:                                                
      Serial.println("ACK-CAN_I_C");
      M90_Var = 0.001 * (uint16_t)(canBuffer_In.data.low & 0x0000ffff) + (uint16_t)(canBuffer_In.data.high >> 24) / 256;
      Serial.print(F("IC(A): "));
      Serial.println(M90_Var);
    break;
    case CAN_ECHO:                                                //Reception of echo of an ASCII character as MASTER
      Serial.println("ACK-ECHO");
      caracter = (byte)(canBuffer_In.data.low >> 8);
      Serial.print(F("Caracter recibido: "));
      Serial.println(caracter);
    break;
    case CAN_RTC:                                                 //Reception of the RTC in binary as MASTER
      Serial.println(F("ACK-RTC"));
      if(rtc_flag == false) {
        rtc_flag = true;      
        auxbuffer[1] = (byte)(canBuffer_In.data.low >> 8);
        auxbuffer[2] = (byte)(canBuffer_In.data.low);
        auxbuffer[3] = (byte)(canBuffer_In.data.high >> 24);
        auxbuffer[4] = (byte)(canBuffer_In.data.high >> 16);
        auxbuffer[5] = (byte)(canBuffer_In.data.high >> 8);
        auxbuffer[6] = (byte)(canBuffer_In.data.high);
      }else{
        rtc_flag = false;      
        auxbuffer[7] = (byte)(canBuffer_In.data.low >> 8);
        auxbuffer[8] = (byte)(canBuffer_In.data.low);
        auxbuffer[9] = (byte)(canBuffer_In.data.high >> 24);
        auxbuffer[10] = (byte)(canBuffer_In.data.high >> 16);
        auxbuffer[11] = (byte)2;
        auxbuffer[12] = (byte)0;
        auxbuffer[13] = (byte)(canBuffer_In.data.high >> 8);
        auxbuffer[14] = (byte)(canBuffer_In.data.high);
        Serial.print(F("Lectura reloj: "));
        for(int h=1;h<=14;h++) {Serial.print(auxbuffer[h]);}
        Serial.println();
      }
    break;
    default:
      resultado = false;
    break;
  }
  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
bool CAN_POST_EXEC(void){
  bool resultado = true;

  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
bool CAN_HB_EXEC(void){
  uint8_t data[8]  = {0,0,0,0,0,0,0,0};
  uint32_t id_mensaje = 0;
  bool resultado = true;
  byte variable;

  variable = (byte)(canBuffer_In.data.low >> 16);
  Serial.print("Variable: "); 
  Serial.println(variable); 
  
  switch(variable){
    case CAN_ECHO:
      Serial.println(F("HB-ECHO"));
      id_mensaje = (((uint32_t)CAN_ACK) << 5) | CAN_SLAVE_ADDRESS; 
      data[0] = CAN_MASTER_ADDRESS;
      data[1] = CAN_ECHO;
      data[2] = (byte)(canBuffer_In.data.low >> 8);
      can_Send_8B(id_mensaje, data);
      Serial.println(id_mensaje);
      Serial.println(data[0]);
      Serial.println(data[1]);
      Serial.println(data[2]);
    break;
    default:
      resultado = false;
    break;
  }
  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Sends a data packet via CAN Bus - To be entered:
//Message ID - priority - 8 bytes of data.
//Priority ranges from 0 --> 15.- Lower number means higher priority.

void can_Send_8B(uint32_t id_mensaje, uint8_t *data){

  canBuffer_Out.id = id_mensaje;
  canBuffer_Out.length = MAX_CAN_FRAME_DATA_LEN;
  canBuffer_Out.extended = false;
  canBuffer_Out.data.low = (((uint32_t)data[0]) << 24) | (((uint32_t)data[1]) << 16) | (((uint32_t)data[2]) << 8) | (uint32_t)data[3];
  canBuffer_Out.data.high = (((uint32_t)data[4]) << 24) | (((uint32_t)data[5]) << 16) | (((uint32_t)data[6]) << 8) | (uint32_t)data[7];
  Can0.sendFrame(canBuffer_Out);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that sends a CAN packet according to input data by Monitor communication.
//Input is divided into the 2 ID fields and the 8 data bytes

void test_Can_Send(void) {
  uint32_t CAN_MSG_L = 0;
  uint32_t CAN_MSG_H = 0;
  uint8_t id_mensaje = 0;
  uint8_t id_dispo = 0;

  Serial.println();

  Serial.println("Enter message identifier and <cr>");
  while (Serial.available() == 0);
  while (Serial.available() > 0) {
    id_mensaje = Serial.parseInt();
  }

  Serial.println("Enter device identifier and <cr>");
  while (Serial.available() == 0);
  while (Serial.available() > 0) {
    id_dispo = Serial.parseInt();
  }

  Serial.println("Enter 4 numbers and <cr>");
  while (Serial.available() == 0);
  while (Serial.available() > 0) {
    CAN_MSG_L = Serial.parseInt();
  }

  Serial.println("Enter another 4 numbers and <cr>");
  while (Serial.available() == 0);
  while (Serial.available() > 0) {
    CAN_MSG_H = Serial.parseInt();
  }

  Serial.print("Message_ID= ");
  Serial.println(id_mensaje);
  Serial.print("ID_Dispo_Origin=  ");
  Serial.println(id_dispo);
  Serial.print("L-data to be sent= ");
  Serial.println(CAN_MSG_L);
  Serial.print("H-data to be sent= ");
  Serial.println(CAN_MSG_H);

  canBuffer_Out.id = (uint32_t)((id_mensaje << 5) | id_dispo);
  canBuffer_Out.length = MAX_CAN_FRAME_DATA_LEN;
  canBuffer_Out.extended = false;
  canBuffer_Out.data.low = CAN_MSG_L;
  canBuffer_Out.data.high = CAN_MSG_H;
  if (Can0.sendFrame(canBuffer_Out) == 1) Serial.println(F("Frame enviado por Can0"));
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that sends a CAN packet according to RS232 input data.
///Input comes as a single packet with the 10 bytes in binary

void test_Can_Send_Bin(void){
  uint32_t CAN_MSG_L = 0;
  uint32_t CAN_MSG_H = 0;
  uint8_t id_mensaje = 0;
  uint8_t id_dispo = 0;
  int i = 1;

  auxbuffer[i] = inChar;                  //Comes from the command decoding state machine
  while (Serial.available() == 0);
  while(Serial.available() > 0) {
    i++;
    auxbuffer[i] = (byte) Serial.read();
    Serial.write(auxbuffer[i]);
    if(auxbuffer[i] == IN_END) break;
    while (Serial.available() == 0);
  }
  
  id_mensaje = (int)auxbuffer[1];
  id_dispo = (int)auxbuffer[2];
  CAN_MSG_L = (((uint32_t)auxbuffer[3] << 24) | ((uint32_t)auxbuffer[4] << 16) | ((uint32_t)auxbuffer[5] << 8) | ((uint32_t)auxbuffer[6]));
  CAN_MSG_H = (((uint32_t)auxbuffer[7] << 24) | ((uint32_t)auxbuffer[8] << 16) | ((uint32_t)auxbuffer[9] << 8) | ((uint32_t)auxbuffer[10]));

  Serial.print("ID_Mensaje= ");
  Serial.println(id_mensaje);
  Serial.print("ID_Dispo_Origen= ");
  Serial.println(id_dispo);
  Serial.print("Datos-L a enviar= ");
  Serial.println(CAN_MSG_L);
  Serial.print("Datos-H a enviar= ");
  Serial.println(CAN_MSG_H);

  canBuffer_Out.id = (uint32_t)((id_mensaje << 5) | id_dispo);
  canBuffer_Out.length = MAX_CAN_FRAME_DATA_LEN;
  canBuffer_Out.extended = false;
  canBuffer_Out.data.low = CAN_MSG_L;
  canBuffer_Out.data.high = CAN_MSG_H;
  if (Can0.sendFrame(canBuffer_Out) == 1) Serial.println(F("Frame enviado por Can0"));  
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//------------------- End CAN driver ---------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//For all working modes, timers and interrupt decouplers

void parada(void){
  Timer1.stop();
  Timer1.detachInterrupt();
  Timer2.stop();
  Timer2.detachInterrupt();
  Timer3.stop();
  Timer3.detachInterrupt();
  Timer4.stop();
  Timer4.detachInterrupt();
  detachInterrupt(digitalPinToInterrupt(DIN0)); 
  detachInterrupt(digitalPinToInterrupt(DIN7)); 
  detachInterrupt(digitalPinToInterrupt(pin1)); 
  detachInterrupt(digitalPinToInterrupt(pin2));

  if(reg_flag == 1){
    reg_flag = 0;
    rtc_write((rtc_m_st + 8), reg_flag);
  }

  if(registrandoM90_flag == 1){
    registrandoM90_flag = 0;
    e2prom_write(e2_M90_registrando, registrandoM90_flag);
    if(!graboRam_SD(ensayo, numeroMedWrite)){                                          //Clear the buffer in RAM by passing the data to SD
      serialOutputTmp("", 0, conCR);
      serialOutputTmp(F("Error al abrir "), 0, sinCR);
      serialOutputTmp(ensayo, 0, conCR);
    }
  }

  if(regHR_flag == 1){
    regHR_flag = 0;
    e2prom_write(e2_M90_regHR, regHR_flag);
    write_M90_1R(DFT_CTRL, 0x0000);                                                    //For the DFT module for the calculation of harmonics per channel 
    if(!graboRam_SD(ensayo, numeroMedWrite)){                                          //Clear the buffer in RAM by passing the data to SD
      serialOutputTmp("", 0, conCR);
      serialOutputTmp(F("Error al abrir "), 0, sinCR);
      serialOutputTmp(ensayo, 0, conCR);
    }
  }
  
  if(regTodo_flag == 1){
    regTodo_flag = 0;
    e2prom_write(e2_M90_regTodo, regTodo_flag);
    write_M90_1R(DFT_CTRL, 0x0000);                                                    //For the DFT module for the calculation of harmonics per channel 
    if(!graboRam_SD(ensayo, numeroMedWrite)){                                          //Clear the buffer in RAM by passing the data to SD
      serialOutputTmp("", 0, conCR);
      serialOutputTmp(F("Error al abrir "), 0, sinCR);
      serialOutputTmp(ensayo, 0, conCR);
    }
  }
  
  flushAll();
  interrupts();
  digitalWrite(TST,LOW);

  serialOutputTmp("", 0, conCR);
  serialOutputTmp("", 0, conCR);
  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that flushes all serial buffers
void flushAll(void){
  while(Serial.available()) Serial.read();
  while(Serial1.available()) Serial1.read();
  while(Serial2.available()) Serial2.read();
  while(Serial3.available()) Serial3.read();
  while(SerialUSB.available()) SerialUSB.read();
  Serial.flush();
  Serial1.flush();
  Serial2.flush();
  Serial3.flush();
  SerialUSB.flush();
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Transmission of the basic data of the equipment via the corresponding channel.

void datos_equipo(void){
  serialOutput(proyecto, 0, conCR);
  serialOutput(equipo, 0, conCR);
  serialOutput(vers, 0, conCR);
  serialOutput(fecha, 0, conCR);
  serialOutput(F("CAN_SLAVE_ADDRESS= "), 0, sinCR);
  serialOutput(String(CAN_SLAVE_ADDRESS),0 , conCR);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Transmits via RS232 the values of all digital channels and the analogue channels read.
//analogue channels read. First it copies the analogue channel vector
//a temporary data freeze mode
 
void Transm_Temp(void) {
  byte aux;
  int i;

  adc_Srv();

  if((modo == 1) || (modo == 2)) {
    for(i=0;i<cantCan_analog1;i++){
      aux = byte(vect_canales1[i] >> 8);    
//      Serial.write(aux);    
//      Serial.write(vect_canales1[i]);
      serialOutputTmp("", aux, sinCR);    
      serialOutputTmp("", vect_canales1[i], sinCR);
    }
  }
  if((modo == 3) || (modo == 4)) {
    for(i=0;i<cantCan_analog2;i++){
      aux = byte(vect_canales2[i] >> 8);    
//      Serial.write(aux);    
//      Serial.write(vect_canales2[i]);
      serialOutputTmp("", aux, sinCR);    
      serialOutputTmp("", vect_canales2[i], sinCR);
    }
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that writes a data to the EEProm and saves the new pumem in the clock ram.
//Data --> data to be written - "pumem" is assumed to point to the first free value
void graba_dato_E2(byte data) {

  if(mem_full == 0){
      e2prom_write(pumem, data);
      pumem++;
      if(pumem >= MEM_FIN_REG) {
        mem_full = 1;
        parada();
      }
      rtc_write(rtc_m_st + 1, byte(pumem>>0));
      rtc_write(rtc_m_st + 2, byte(pumem>>8));
      rtc_write(rtc_m_st + 3, byte(pumem>>16));
      rtc_write(rtc_m_st + 4, byte(pumem>>24));
  }
}

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that clears the EEProm and initialises the register pointer "pumem" to zero
void borrado_E2(void) {
    parada();
    e2prom_erase();
    pumem = 0;
    rtc_write(rtc_m_st + 1, byte(pumem>>0));
    rtc_write(rtc_m_st + 2, byte(pumem>>8));
    rtc_write(rtc_m_st + 3, byte(pumem>>16));
    rtc_write(rtc_m_st + 4, byte(pumem>>24));
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that saves the register initialisation data to the EEProm
void graba_ini_E2(void) {
  int i;
  char buf[6];

  sprintf(buf,"%04d",tiempo_reg);
  get_time();
  graba_dato_E2(0xff);
  graba_dato_E2(0xff);
  for(i=1;i<=10;i++){
    graba_dato_E2(auxbuffer[i]);
  }
  graba_dato_E2(0x32);
  graba_dato_E2(0x30);
  graba_dato_E2(auxbuffer[11]);
  graba_dato_E2(auxbuffer[12]);
  for(i=0;i<=3;i++){
    graba_dato_E2(buf[i]);
  }
  graba_dato_E2(modo);
  graba_dato_E2(cantCan_analog1);
  graba_dato_E2(cantCan_analog2);
  graba_dato_E2(0x08);
  graba_dato_E2(rtc_read(rtc_m_st + 7));
  graba_dato_E2(0xff);
  graba_dato_E2(0xff);
}

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Timed EEProm (I2C) register routine of set channels and mode
//Pumem points to first free memory location
void Registro_Temp_E2(void) {
  int i;

  serialOutput(F("Rutina de registro"), 0, conCR);

  adc_Srv();

  graba_dato_E2(leo_can_dig());                       //Record status of the 8 digital channels

  if(modo == 2) {
    for(i=0;i<cantCan_analog1;i++){
      graba_dato_E2(byte(vect_canales1[i]));       //Record values of the set analogue channels
      graba_dato_E2(byte(vect_canales1[i] >> 8));
    }
  }
  if(modo == 4) {
    for(i=0;i<cantCan_analog2;i++){
      graba_dato_E2(byte(vect_canales2[i]));       //Record values of the set inAmp analogue channels
      graba_dato_E2(byte(vect_canales2[i] >> 8));
    }
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that reinitialises the system
void softwareReset(void){
  serialOutput("", 0, conCR);
  serialOutput(F("Software reset activado..."), 0, conCR);
  lcd.clear();
  lcd.print(F("Software reset"));
  lcd.setCursor(0, 1);
  lcd.print(F("activado..."));
  delay(1000);
  REQUEST_EXTERNAL_RESET;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Function transmitting the IP of the board connected to Wifi
String getDirIp(void){
  char c;
  int flag = 0;
  String aux = "";

  Serial3.println("AT+CIPSTA?");
  delay(500);
  while(Serial3.available()){
    c = Serial3.read();
    if(c == comillas) flag++;
    if(flag == 1 && c != comillas) aux = aux + String(c);
  }
  return aux;
}

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Allows to set the amplification of the inAmps. Analogue input ((in_inAmp) 09 to 12)
//with amplification ((amplif) 0 to 3)

bool set_inAmp( byte in_inAmp, byte amplif){
  bool resultado = true;
  if((in_inAmp > 8 && in_inAmp < 13) && amplif < 4){
    in_inAmp = in_inAmp + 27;               
    switch(amplif){                                         //Amplification setting
      case 0:
        digitalWrite(G_0,LOW);
        digitalWrite(G_1,LOW);
      break;
      case 1:
        digitalWrite(G_0,HIGH);
        digitalWrite(G_1,LOW);
      break;
      case 2:
        digitalWrite(G_0,LOW);
        digitalWrite(G_1,HIGH);
      break;
      case 3:
        digitalWrite(G_0,HIGH);
        digitalWrite(G_1,HIGH);
      break;
    }
    digitalWrite(in_inAmp,LOW);                             //Negative pulse to take the amplification as appropriate
    delayMicroseconds(1);    
    digitalWrite(in_inAmp,HIGH);
  }else{
    resultado = false;
  }
  return resultado;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Allows to set a digital output pin ((pinDig) from 02 to 09) to ((status) 0 or 1)

void set_saldig(void){
  byte pinDig;
  bool estado;

  pinDig = auxbuffer[2];
  if(pinDig > 1 && pinDig < 10){                //Pin13 (led) and pins 0 and 1 (Rx0 and Tx0) are not used.
    estado = auxbuffer[3];  
    digitalWrite(pinDig, estado);
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Allows to set a PWM on the selected pin ((pinPwm) from 10 to 13) with the Duty-Cicle (dtyCicle)

void set_pwm(void){
  byte pinPwm;
  unsigned int dtyCicle;

  pinPwm = auxbuffer[1] * 10 + auxbuffer[2];
  if(pinPwm > 9 && pinPwm < 14){               //Pin13 (led) and pins 0 and 1 (Rx0 and Tx0) are not used
    dtyCicle = auxbuffer[3] * 100 + auxbuffer[4] * 10 + auxbuffer[5];  
    analogWrite(pinPwm, dtyCicle);
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Sets the selected DAC ((pinDac) from 00 to 01) with a specific value ((Value) from 0000 to 4095)

void set_dac(void){
  byte pinDac;
  unsigned int valor;

  pinDac = auxbuffer[2] + 66;
  if(pinDac > 65 && pinDac < 68){
    valor = auxbuffer[3] * 1000 + auxbuffer[4] * 100 + auxbuffer[5] * 10 + auxbuffer[6];  
    analogWrite(pinDac, valor);
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that reads the digital input channels and unifies them into a single byte "digi_in".
byte leo_can_dig(void){
  byte digi_in;
  
  digi_in = 0x00 | byte(digitalRead(DIN0))
                 | (byte(digitalRead(DIN1)) << 1)
                 | (byte(digitalRead(DIN2)) << 2)
                 | (byte(digitalRead(DIN3)) << 3)
                 | (byte(digitalRead(DIN4)) << 4)
                 | (byte(digitalRead(DIN5)) << 5)
                 | (byte(digitalRead(DIN6)) << 6)
                 | (byte(digitalRead(DIN7)) << 7);
  return digi_in;
}

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that downloads the data recorded on the EEProm and transmits it via RS232.
void descarga_E2(void){
  unsigned long aux_p;

  serialOutput("", byte(pumem>>24), sinCR);
  serialOutput("", byte(pumem>>16), sinCR);
  serialOutput("", byte(pumem>>8), sinCR);
  serialOutput("", byte(pumem>>0), sinCR);

  for(aux_p=0;aux_p<pumem;aux_p++){
    serialOutput("", byte(e2prom_read(aux_p)), sinCR);
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                          Routines for WiFi communication
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of the WiFi communication with the server on port 80
bool ini_Wifi(void){
  char c;
  int i = 0;
  int index = 0;
  String ordenes[]= {  
    "ATE0",
    "AT+CWAUTOCONN=0",
    "AT+CWQAP",
    "AT+CWJAP_CUR=\"",    //Beginning of the AT command to connect to the WiFi network with "network_name" and "password"
    "AT+CIPSTA?",
    "AT+CIPMUX=1",
    "AT+CIPSERVER=1,80",
    "END"                                                // END" --> To recognise the end of AT commands
  };
  String s ="";
  char nombre_Red[32] = "";
  char clave[32] = "";

  for(i=0;i<=31;i++){
    nombre_Red[i] = e2prom_read(e2_RedWifi+i);
    clave[i] = e2prom_read(e2_WifiPass+i);
  }

  i = 0;
  while(ordenes[index] != "END"){  
    if(index == 3) {
      Serial3.println(ordenes[index]+String(nombre_Red)+"\",\""+String(clave)+"\"");
      delay(7500);
    } else {
      Serial3.println(ordenes[index]);
    }
    delay(500);
    index++;
//    s="";
    while(Serial3.available()){
      c = Serial3.read();
      Serial.print(c);
      if(c != '\n') s = s + c;
      else s="";
      if (s.startsWith("OK")) i++;
    }
    Serial.println("....................");
  }  
  if(i >= 14) {
    return true;
  }else{
    return false;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Send character string over RS232 with ESP8266 board
void envioWifi(String output){
  char aux1[] = ">";
  char aux2[] = "SEND OK";
  
  Serial3.print("AT+CIPSEND=0,");               // AT+CIPSEND=0, num
  Serial3.println(output.length());
  if(Serial3.find(aux1)){                        // If we receive the message request
    Serial3.println(output);                      //Here goes the string to the WIFI 
    delay(10);
    while (Serial3.available() > 0 ){ 
      if(Serial3.find(aux2)) break;        // Look for the OK in the wifi response.
    }
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Send a binary data via RS232 with the ESP8266 board.
void envioWifi_Bin(byte dato){
  char aux1[] = ">";
  char aux2[] = "SEND OK";
  
  Serial3.print("AT+CIPSEND=0,");               // AT+CIPSEND=0, num
  Serial3.println(1);
  if(Serial3.find(aux1)){                        // If we receive the message request
    Serial3.write(dato);
    Serial3.println(); 
    delay(10);
    while (Serial3.available() > 0 ){ 
      if(Serial3.find(aux2)) break;        // Look for the OK in the wifi response
    }
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Receives a line from the ESP8266 board

String leo_Wifi(){
  String s = "" ;
  if (Serial3.available()){
    char c = Serial3.read();
    while ( c != '\n' ){            //Until the character is intro
      s = s + c;
      delay(25);
      c = Serial3.read();
    }
  }
  return(s) ;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Writes the name of Red and its password in the clock ram.
void set_Net_Pas(void){
  int i=0,j=0;

  while(archivo[i] != 0xff){
    e2prom_write(e2_RedWifi+i,archivo[i]);
    i++;
  }
  e2prom_write(e2_RedWifi+i,0x00);
  
  i++;
  while(archivo[i] != 0xff){
    e2prom_write(e2_WifiPass+j,archivo[i]);
    i++;
    j++;
  }
  e2prom_write(e2_WifiPass+j,0x00);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Read the Network name and its Wifi access key from e2
void leo_Net_Pas(void){
  int i=0;
  char nombre_Red[32] = "";
  char clave[32] = "";

  for(i=0;i<=31;i++){
    nombre_Red[i] = e2prom_read(e2_RedWifi+i);
    clave[i] = e2prom_read(e2_WifiPass+i);
  }
  serialOutput(F("Red Wifi: "), 0, sinCR);
  serialOutput(nombre_Red, 0, conCR);
  serialOutput(F("Password: "), 0, sinCR);
  serialOutput(clave, 0, conCR);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                        SD memory routines
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that adds data to a file in the SD memory.
//file --> name of the file where you want to save the data.
//data --> data to save

bool grabo_dato_sd(char *archivo, byte data){

  File dataFile = SD.open(archivo, FILE_WRITE);

  if (dataFile) {
    dataFile.write(data);
    dataFile.close();
    return true;
  }
  else {
    serialOutput("Error al abrir ", 0, sinCR);
    serialOutput(archivo, 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that adds a string to a file in the SD memory.
//file --> name of the file where you want to save the data.
//dataString --> srting to save to

bool graboStringSd(char *archivo, String dataString){
  File dataFile = SD.open(archivo, FILE_WRITE);

  if (dataFile) {
    dataFile.print(dataString);
    dataFile.close();
    return true;
  }
  else {
    serialOutput("Error al abrir ", 0, sinCR);
    serialOutput(archivo, 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that reads a file from the SD memory and sends it via RS232.
//file --> name of the file where you want to save the data.

bool leo_arch_sd(char *archivo){
  
  File dataFile = SD.open(archivo, FILE_READ);

  if (dataFile) {
    while (dataFile.available()) {
      serialOutput("", dataFile.read(), sinCR);
    }
    dataFile.close();
    serialOutput("", 0, conCR);
    return true;

  } else {
    serialOutput(archivo, 0, sinCR);
    serialOutput(F(" no existe"), 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------
void listaArcchivosSD(void){
  File dataFile = SD.open("/");
  if (dataFile) {
    printDirectory(dataFile, 0);
    dataFile.close();
  } else {
    serialOutput(F("Unable to access data files in SD memory"), 0, conCR);
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that shows the files that are in the SD memory.
void printDirectory(File dir, int numTabs) {

  serialOutput("", 0, conCR);
  while (true) {
    File entry =  dir.openNextFile();
    if (!entry) {
      break;                                                        //There are no more files
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      serialOutput(String('\t'), 0, sinCR);
    }

    if (entry.isDirectory()) {
//      serialOutput(entry.name(), 0, sinCR);                                //For subdirectories
//      serialOutput("/", 0, conCR);
//      printDirectory(entry, numTabs + 1);
    } else {
      serialOutput(String(entry.name()), 0, sinCR);
      serialOutput(String("\t\t\t"), 0, sinCR);                     //Files have size, directories do not.
      serialOutput(String(entry.size()), 0, sinCR);
      serialOutput(String(" bytes"), 0, conCR);
    }
    entry.close();
  }
}

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                     FLASH (SPI) Memory Routines - S25FL127S
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that adds data to a file in Flash memory.
//file --> name of the file where you want to save the data. If it does not exist, it creates it.
//data --> data to save

bool grabo_dato_flash(char *archivo, byte data){
  char buff[256];

  buff[0] = data;
  if (SerialFlash.exists(archivo) == false) {
        SerialFlash.create(archivo, 256);
  }

  SerialFlashFile S_File = SerialFlash.open(archivo);
  if (S_File) {
    S_File.write(buff, 1);
    S_File.close();
    return true;
  } else {
    serialOutput(F("Error al abrir "), 0, sinCR);
    serialOutput(archivo, 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that reads a file from the Flash memory and sends it via RS232.
//file --> name of the file where you want to save the data.
//cant_data --> amount of data to read (0 --> all)
bool leo_arch_flash(char *archivo){

  if (SerialFlash.exists(archivo)) {
    SerialFlashFile S_File = SerialFlash.open(archivo);
  
    if (S_File) {
      serialOutput(archivo, 0, conCR);
      char buffer[256];
      S_File.read(buffer, 256);
      serialOutput(buffer, 0, sinCR);
      S_File.close();
      return true;
    } else {
      serialOutput(F("Error al abrir "), 0, sinCR);
      serialOutput(archivo, 0, conCR);
      return false;
    }
  } else {
    serialOutput(archivo, 0, sinCR);
    serialOutput(F(" no existe"), 0, conCR);
    return false;
  }    
}
 
//--------------------------------------------------------------------------------------------------
//Rutina que borra toda la Flash

void borro_flash(void) {

  SerialFlash.eraseAll();
  
  while (SerialFlash.ready() == false) {
     // Loop de espera hasta que se borre. Puede tardar de 30 seg. a 2 min.
  }
}
 
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                          EEPROM Driver - (I2C) - 24AA1025
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that writes a data to the E2
//e2addr --> full 4-byte memory bank pointer address
//data --> data to be stored

void e2prom_write(unsigned long e2_addr, byte data){
  byte memaddl, memaddh,i; 

  i = (byte)((e2_addr & 0x00010000) >> 13);
  i = i | mem_ad;
  i = i >> 1;
  memaddh = (byte)((e2_addr & 0x0000ff00)>> 8);
  memaddl = (byte)(e2_addr & 0x000000ff);
#ifdef S_P_NUEVA
  digitalWrite(WP1, LOW);                 
#endif
  Wire.beginTransmission(i);
  Wire.write(memaddh);                
  Wire.write(memaddl);                
  Wire.write(data);                
  Wire.endTransmission(true);
  delay(10);    
#ifdef S_P_NUEVA
  digitalWrite(WP1, HIGH);                 
#endif
}  
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine that reads a data from E2
//e2addr --> full address of pointer to memory bank in 4 bytes
//Returns byte read

byte e2prom_read(unsigned long e2_addr){
  byte memaddl, memaddh, i, c; 
  
  i = (byte)((e2_addr & 0x00010000) >> 13);
  i = i | mem_ad;
  i = i >> 1;
  memaddh = (byte)((e2_addr & 0x0000ff00)>> 8);
  memaddl = (byte)(e2_addr & 0x000000ff);
  Wire.beginTransmission(i);
  Wire.write(memaddh);                
  Wire.write(memaddl);                
  Wire.endTransmission(false);    
  Wire.requestFrom((int)i, (int)1); 
  while(!Wire.available()) {delay(1);}
  c = Wire.read();
  return c;    
}  
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine storing a 16-bit data in E2
void e2Write_16b(unsigned long direccion, uint16_t dato){
  e2prom_write(direccion, (byte)dato);
  e2prom_write(direccion+1, (byte)(dato >> 8));
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine reading a 16-bit data from E2
uint16_t e2Read_16b(unsigned long direccion){
  uint16_t dato;

  dato = e2prom_read(direccion) | ((e2prom_read(direccion+1)) << 8);
  return dato;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//E2 memory fast deletion

void e2prom_erase(void){
  byte memaddl, memaddh, i, data, k; 
  unsigned long e2_addr;
  
  data = 0xff;
  e2_addr = 0;
#ifdef S_P_NUEVA
  digitalWrite(WP1, LOW);                 
#endif
  do{
    i = (byte)((e2_addr & 0x00010000) >> 13);
    i = i | mem_ad;
    i = i >> 1;
    memaddh = (byte)((e2_addr & 0x0000ff00)>> 8);
    memaddl = (byte)(e2_addr & 0x000000ff);
    Wire.beginTransmission(i);
    Wire.write(memaddh);                
    Wire.write(memaddl);
    for(k=1;k<=64;k++){                
       Wire.write(data);
       e2_addr++;                
    }
    Wire.endTransmission(true);    
    delay(10);
  }while(e2_addr < MEM_FIN_REG);
#ifdef S_P_NUEVA
  digitalWrite(WP1, HIGH);                 
#endif
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                           Rutinas para RTC (I2C) - DS1307
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Setting the clock
//If not possible, return "false" but "true".
//Input format --> hhmmssDDMMAAAAAA

bool Set_time(void){
  byte i, j, k, m;
  
   //hhmmssddmmaaaa  invierto ssmmhhddmmaaaa
   k = auxbuffer[1];
   auxbuffer[1] = auxbuffer[5];
   auxbuffer[5] = k;
   k = auxbuffer[2];
   auxbuffer[2] = auxbuffer[6];
   auxbuffer[6] = k;
   k = 0;
   do{
      k++;
      i = k;
      i++;
      m = auxbuffer[k] * 10;
      m = m + auxbuffer[i];
      switch(k){
         //ss
         case 1:
            if(m > 59) return false;
         break;
         //mm
         case 3:
            if(m > 59) return false;
         break;
            //hh
         case 5:
            if(m > 23) return false;
         break;
            //dd
         case 7:
            if(m > 31) return false;
            if(m == 0) return false;
         break;
            //mm
         case 9:
            if(m > 12) return false;
            if(m == 0) return false;
         break;
            //año 20
         case 11:
            if(m != 20) return false;
         break;
            //año aa
         case 13:
            if(m > 99) return false;
//            if(m < 0) return false;
         break;
      }
      k++;
   }while(k < 14);
   k = 0;
   do{
      k++;
      i = auxbuffer[k];
      i = rotl_b(i, 4);
      k++;
      m = auxbuffer[k];
      auxbuffer[k] = i | m;
   }while(k <= 13);

    //program to ds1307
   //SETTING SS MM MM HH DD MM
   k = 0;
   i = rtc_ad;
   m = 0x80;
   Wire.beginTransmission(i);
   Wire.write(k);                
   Wire.write(m);                
   Wire.endTransmission(true);    
   j = 4;
   k = 1;
   i = rtc_ad;
   do{
      m = auxbuffer[j];
      Wire.beginTransmission(i);
      Wire.write(k);                
      Wire.write(m);                
      Wire.endTransmission(true);    
      j++;
      j++;
      k++;
      if(k == 3) k++;
      if(k == 6){
         j++;
         j++;
      }
   }while(k <= 6);
   k = 0;
   m = auxbuffer[2];
   Wire.beginTransmission(i);
   Wire.write(k);                
   Wire.write(m);                
   Wire.endTransmission(true);    
   return true;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// ds1307 is read
//Format --> SS MM HH DD MM YYYYY

void get_time(void){
  byte j, k, i, m;

  j = 1;
  k = 0x00;
  i = rtc_ad;
  Wire.beginTransmission(i);
  Wire.write(k);                
  Wire.endTransmission(false);    
  Wire.requestFrom((int)i, (int)7, (int)true); 
  do{
     m = Wire.read();
     if(k != 3){
         i = m & 0x0f;
         i = i + 48;
         m = rotr_b(m, 4);
         m = m & 0x0f;
         m = m + 48;
         auxbuffer[j] = m;
         j++;
         auxbuffer[j] = i;
         j++;
     }
     k++;
  }while(k < 6);
   m = Wire.read();
   i = m & 0x0f;
   i = i + 48;
   m = rotr_b(m, 4);
   m = m & 0x0f;
   m = m + 48;
   auxbuffer[j] = m;
   j++;
   auxbuffer[j] = i;
//hhmmssddmmaaaa  invierto ssmmhhddmmaaaa
   k = auxbuffer[1];
   auxbuffer[1] = auxbuffer[5];
   auxbuffer[5] = k;
   k = auxbuffer[2];
   auxbuffer[2] = auxbuffer[6];
   auxbuffer[6] = k;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
///Transmits via RS232 current day and time
//Comes in auxbuffer[] as: hhmmssddmmaa
void prt_time(void){
   byte i, k;
   
   k = 0;
   do{
      k++;
      i = k;
      i++;
      if(k == 11){
        serialOutput("20", 0, sinCR);
      }
      serialOutput(String(char(auxbuffer[k])), 0, sinCR);
      serialOutput(String(char(auxbuffer[i])), 0, sinCR);
      switch(k){
         case 1:
            serialOutput(":", 0, sinCR);
            break;
         case 3:
            serialOutput(":", 0, sinCR);
            break;
         case 5:
            serialOutput(" ", 0, sinCR);
            break;
         case 7:
            serialOutput("/", 0, sinCR);
            break;
         case 9:
            serialOutput("/", 0, sinCR);
            break;
      }
      k++;
   }while(k <= 11);
   serialOutput("", 0, conCR);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Write data to the RTC 
//rtc_addr --> internal address of the registers and memory of the RTC
//data --> data to be stored

void rtc_write(byte rtc_addr, byte data){
  byte i; 
  
  i = rtc_ad;
  Wire.beginTransmission(i);
  Wire.write(rtc_addr);                
  Wire.write(data);                
  Wire.endTransmission(true);    
}  
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads a data from the RTC
//rtc_addr --> internal address of the registers and memory of the RTC
//data --> data to read

byte rtc_read(byte rtc_addr){
  byte i, c; 
  
  i = rtc_ad;
  Wire.beginTransmission(i);
  Wire.write(rtc_addr);                
  Wire.endTransmission(false);    
  Wire.requestFrom((int)i, (int)1, (int)true); 
  c = Wire.read();
  return c;    
}  
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//-------------------------------- Driver M90E36A --------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reading a record from M90E36A
uint16_t read_M90_1R(uint16_t registro_var16){
  uint16_t read_var16;

  registro_var16 = registro_var16 | 0x8000;   //Accommodate to be a read (1st bit set to "1")
  SPI.beginTransaction(Spi_M90_param);
  digitalWrite(CS_M90, LOW);
  SPI.transfer16(registro_var16);      
  read_var16 = SPI.transfer16(0x0000);      
  digitalWrite(CS_M90, HIGH);
  SPI.endTransaction();
  return(read_var16);

}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reading a record from M90E36A
uint16_t read_M90_1RC(uint16_t registro_var16){
  uint16_t read_var16c, read_control;

  read_var16c = read_M90_1R(registro_var16);
  read_control = read_M90_1R(LastSPIData);
  if(read_var16c != read_control){
    do{
      read_var16c = read_control;
      read_control = read_M90_1R(LastSPIData);
    } while(read_var16c != read_control);
  }
  return(read_var16c);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Writing a record of the M90E36A
void write_M90_1R(uint16_t registro_var16, uint16_t dato16){

  registro_var16 = registro_var16 & 0x7fff;   //Annotation to be a write (1st bit set to "0")
//        Serial.println();
//        Serial.println(F("------------------------------"));
//        Serial.println(F("Writing a register"));
//        Serial.print(F("Register 0x"));
//        Serial.println(registro_var16,HEX);
//        Serial.print(F("Dato 0x"));
//        Serial.println(dato16,HEX);
//        Serial.println(F("------------------------------"));
  SPI.beginTransaction(Spi_M90_param);
  digitalWrite(CS_M90, LOW);
  SPI.transfer16(registro_var16);      
  SPI.transfer16(dato16);      
  digitalWrite(CS_M90, HIGH);
  SPI.endTransaction();
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Writing a record of the M90E36A with control by LastSPIData
void write_M90_1RC(uint16_t registro_var16, uint16_t dato16){
  uint16_t read_control;
  
  do{
    write_M90_1R(registro_var16, dato16);
    read_control = read_M90_1R(LastSPIData);
  }while(dato16 != read_control);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Read a range of records from the M90E36A
void read_M90_rango(uint16_t registro_inicial, uint16_t registro_final){
  uint16_t i,j,k;

  Serial.println();
  Serial.println(F("------------------------------"));
  j=registro_inicial;
  for(i=0;i<=(registro_final-registro_inicial);i++){
    k = read_M90_1RC(j);
    Serial.print(F("Registro 0x"));
    Serial.print(j,HEX);
    Serial.print(F(" --> 0x"));
    Serial.println(k,HEX);
    j++;
    delay(1);
  }
  Serial.println(F("------------------------------"));
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Verify the effectiveness of the value of the configuration registers.
//Should be checked periodically
void ChEf_EfVal(void){
  if(read_M90_1RC(ConfigStart) != 0x8765){
    IniReg_Config_Ram();
  }else{
    if((read_M90_1RC(SysStatus0) & 0x4000) != 0) {IniReg_Config_Ram();}
  }
  if(read_M90_1RC(CalStart) != 0x8765){
    IniReg_Cal_Ram();
  }else{
    if((read_M90_1RC(SysStatus0) & 0x1000) != 0) {IniReg_Cal_Ram();}
  }
  if(read_M90_1RC(HarmStart) != 0x8765){
    IniReg_Harm_Ram();
  }else{
    if((read_M90_1RC(SysStatus0) & 0x0400) != 0) {IniReg_Harm_Ram();}
  }
  if(read_M90_1RC(AdjStart) != 0x8765){
    IniReg_Adj_Ram();
  }else{
    if((read_M90_1RC(SysStatus0) & 0x0100) != 0) {IniReg_Adj_Ram();}
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory
void iniReg_M90_Ram(void){ 
  IniReg_Config_Ram();
  IniReg_Cal_Ram();
  IniReg_Harm_Ram();
  IniReg_Adj_Ram();
  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory 
void IniReg_Config_Ram(void){
  Serial.println();
  Serial.println("Cargando registros FUNCTION CONFIGURATION");
  write_M90_1RC(ConfigStart, CALIBRACION);

  write_M90_1RC(PLconstH, Set_PLconstH);
  write_M90_1RC(PLconstL, Set_PLconstL);

  write_M90_1RC(MMode0, Set_MMode0);
  write_M90_1RC(MMode1, Set_MMode1);

  write_M90_1RC(PStartTh, Set_PStartTh);
  write_M90_1RC(QStartTh, Set_QStartTh);
  write_M90_1RC(SStartTh, Set_SStartTh);
  write_M90_1RC(PPhaseTh, Set_PPhaseTh);
  write_M90_1RC(QPhaseTh, Set_QPhaseTh);
  write_M90_1RC(SPhaseTh, Set_SPhaseTh);

  uint16_t aux = read_M90_1RC(CS_0);
  write_M90_1RC(CS_0, aux);
  write_M90_1RC(ConfigStart,OPERACION);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory 
void IniReg_Cal_Ram(void){
  Serial.println();
  Serial.println("Loading ENERGY METERING CALIBRATION records");
  write_M90_1RC(CalStart, CALIBRACION);

  write_M90_1RC(PoffsetA, Set_PoffsetA);
  write_M90_1RC(QoffsetA, Set_QoffsetA);
  write_M90_1RC(PoffsetB, Set_PoffsetB);
  write_M90_1RC(QoffsetB, Set_QoffsetB);
  write_M90_1RC(PoffsetC, Set_PoffsetC);
  write_M90_1RC(QoffsetC, Set_QoffsetC);

  write_M90_1RC(PQGainA, Set_PQGainA);
  write_M90_1RC(PhiA, Set_PhiA);
  write_M90_1RC(PQGainB, Set_PQGainB);
  write_M90_1RC(PhiB, Set_PhiB);
  write_M90_1RC(PQGainC, Set_PQGainC);
  write_M90_1RC(PhiC, Set_PhiC);

  uint16_t aux = read_M90_1RC(CS_1);
  write_M90_1RC(CS_1, aux);
  write_M90_1RC(CalStart,OPERACION);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory 
void IniReg_Harm_Ram(void){
  Serial.println();
  Serial.println("Loading records FUNDAMENTAL/ARMONIC ENERGY METERING CALIBRATION");
  write_M90_1RC(HarmStart, CALIBRACION);

  write_M90_1RC(PoffsetAF, Set_PoffsetAF);
  write_M90_1RC(PoffsetBF, Set_PoffsetBF);
  write_M90_1RC(PoffsetCF, Set_PoffsetCF);

  write_M90_1RC(PGainAF, Set_PGainAF);
  write_M90_1RC(PGainBF, Set_PGainBF);
  write_M90_1RC(PGainCF, Set_PGainCF);

  uint16_t aux = read_M90_1RC(CS_2);
  write_M90_1RC(CS_2 , aux);
  write_M90_1RC(HarmStart,OPERACION);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory 
void IniReg_Adj_Ram(void){
  Serial.println();
  Serial.println("Loading MEASUREMENT VALUE CALIBRATION records");
  write_M90_1RC(AdjStart, CALIBRACION);

  write_M90_1RC(UoffsetA, Set_UoffsetA);
  write_M90_1RC(UoffsetB, Set_UoffsetB);
  write_M90_1RC(UoffsetC, Set_UoffsetC);

  write_M90_1RC(IoffsetA, Set_IoffsetA);
  write_M90_1RC(IoffsetB, Set_IoffsetB);
  write_M90_1RC(IoffsetC, Set_IoffsetC);

  write_M90_1RC(UgainA, Set_UgainA);
  write_M90_1RC(UgainB, Set_UgainB);
  write_M90_1RC(UgainC, Set_UgainC);

  write_M90_1RC(IgainA, Set_IgainA);
  write_M90_1RC(IgainB, Set_IgainB);
  write_M90_1RC(IgainC, Set_IgainC);

  write_M90_1RC(IgainN, Set_IgainN);
  write_M90_1RC(IoffsetN, Set_IoffsetN);

  uint16_t aux = read_M90_1RC(CS_3);
  write_M90_1RC(CS_3, aux);
  write_M90_1RC(AdjStart,OPERACION);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of variables and registers in RAM from the ATM90E36A's RAM 
void Load_Ram_M90(void){ 
  Load_Ram_Config();
  Load_Ram_Cal();
  Load_Ram_Harm();
  Load_Ram_Adj();
  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of variables in Ram from ATM90E36A
void Load_Ram_Config(void){
  Serial.println();
  Serial.println("Loading into RAM the FUNCTION CONFIGURATION registers");

  Set_PLconstH = read_M90_1RC(PLconstH);
  Set_PLconstL = read_M90_1RC(PLconstL);

  Set_MMode0 = read_M90_1RC(MMode0);
  Set_MMode1 = read_M90_1RC(MMode1);

  Set_PStartTh = read_M90_1RC(PStartTh);
  Set_QStartTh = read_M90_1RC(QStartTh);
  Set_SStartTh = read_M90_1RC(SStartTh);
  Set_PPhaseTh = read_M90_1RC(PPhaseTh);
  Set_QPhaseTh = read_M90_1RC(QPhaseTh);
  Set_SPhaseTh = read_M90_1RC(SPhaseTh);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory 
void Load_Ram_Cal(void){
  Serial.println();
  Serial.println("Loading the ENERGY METERING CALIBRATION records into RAM");

  Set_PoffsetA = read_M90_1RC(PoffsetA);
  Set_QoffsetA = read_M90_1RC(QoffsetA);
  Set_PoffsetB = read_M90_1RC(PoffsetB);
  Set_QoffsetB = read_M90_1RC(QoffsetB);
  Set_PoffsetC = read_M90_1RC(PoffsetC);
  Set_QoffsetC = read_M90_1RC(QoffsetC);

  Set_PQGainA = read_M90_1RC(PQGainA);
  Set_PhiA = read_M90_1RC(PhiA);
  Set_PQGainB = read_M90_1RC(PQGainB);
  Set_PhiB = read_M90_1RC(PhiB);
  Set_PQGainC = read_M90_1RC(PQGainC);
  Set_PhiC = read_M90_1RC(PhiC);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory
void Load_Ram_Harm(void){
  Serial.println();
  Serial.println("Loading FUNDAMENTAL/ARMONIC ENERGY METERING CALIBRATION records into RAM");

  Set_PoffsetAF = read_M90_1RC(PoffsetAF);
  Set_PoffsetBF = read_M90_1RC(PoffsetBF);
  Set_PoffsetCF = read_M90_1RC(PoffsetCF);

  Set_PGainAF = read_M90_1RC(PGainAF);
  Set_PGainBF = read_M90_1RC(PGainBF);
  Set_PGainCF = read_M90_1RC(PGainCF);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers from RAM memory 
void Load_Ram_Adj(void){
  Serial.println();
  Serial.println("Loading MEASUREMENT VALUE CALIBRATION records into RAM");

  Set_UoffsetA = read_M90_1RC(UoffsetA);
  Set_UoffsetB = read_M90_1RC(UoffsetB);
  Set_UoffsetC = read_M90_1RC(UoffsetC);

  Set_IoffsetA = read_M90_1RC(IoffsetA);
  Set_IoffsetB = read_M90_1RC(IoffsetB);
  Set_IoffsetC = read_M90_1RC(IoffsetC);

  Set_UgainA = read_M90_1RC(UgainA);
  Set_UgainB = read_M90_1RC(UgainB);
  Set_UgainC = read_M90_1RC(UgainC);

  Set_IgainA = read_M90_1RC(IgainA);
  Set_IgainB = read_M90_1RC(IgainB);
  Set_IgainC = read_M90_1RC(IgainC);

  Set_IgainN = read_M90_1RC(IgainN);
  Set_IoffsetN = read_M90_1RC(IoffsetN);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of variables in RAM, equivalent to ATM90E36A registers, from the E2
void iniSet_M90_E2(void){ 
  IniSet_Config_E2();
  IniSet_Cal_E2();
  IniSet_Harm_E2();
  IniSet_Adj_E2();
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers in RAM from E2
void IniSet_Config_E2(void){
  Serial.println();
  Serial.println("Loading RAM with FUNCTION CONFIGURATION registers of the E2");

  Set_PLconstH = e2Read_16b(e2_PLconstH);
  Set_PLconstL = e2Read_16b(e2_PLconstL);

  Set_MMode0 = e2Read_16b(e2_MMode0);
  Set_MMode1 = e2Read_16b(e2_MMode1);

  Set_PStartTh = e2Read_16b(e2_PStartTh);
  Set_QStartTh = e2Read_16b(e2_QStartTh);
  Set_SStartTh = e2Read_16b(e2_SStartTh);
  Set_PPhaseTh = e2Read_16b(e2_PPhaseTh);
  Set_QPhaseTh = e2Read_16b(e2_QPhaseTh);
  Set_SPhaseTh = e2Read_16b(e2_SPhaseTh);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers in RAM from E2
void IniSet_Cal_E2(void){
  Serial.println();
  Serial.println("Loading RAM with ENERGY METERING CALIBRATION records from E2");

  Set_PoffsetA = e2Read_16b(e2_PoffsetA);
  Set_QoffsetA = e2Read_16b(e2_QoffsetA);
  Set_PoffsetB = e2Read_16b(e2_PoffsetB);
  Set_QoffsetB = e2Read_16b(e2_QoffsetB);
  Set_PoffsetC = e2Read_16b(e2_PoffsetC);
  Set_QoffsetC = e2Read_16b(e2_QoffsetC);

  Set_PQGainA = e2Read_16b(e2_PQGainA);
  Set_PhiA = e2Read_16b(e2_PhiA);
  Set_PQGainB = e2Read_16b(e2_PQGainB);
  Set_PhiB = e2Read_16b(e2_PhiB);
  Set_PQGainC = e2Read_16b(e2_PQGainC);
  Set_PhiC = e2Read_16b(e2_PhiC);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers in RAM from E2
void IniSet_Harm_E2(void){
  Serial.println();
  Serial.println("Loading RAM with FUNDAMENTAL/ARMONIC ENERGY METERING CALIBRATION records from E2");

  Set_PoffsetAF = e2Read_16b(e2_PoffsetAF);
  Set_PoffsetBF = e2Read_16b(e2_PoffsetBF);
  Set_PoffsetCF = e2Read_16b(e2_PoffsetCF);

  Set_PGainAF = e2Read_16b(e2_PGainAF);
  Set_PGainBF = e2Read_16b(e2_PGainBF);
  Set_PGainCF = e2Read_16b(e2_PGainCF);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Initialisation of ATM90E36A variables and registers in RAM from E2
void IniSet_Adj_E2(void){
  Serial.println();
  Serial.println("Loading RAM with MEASUREMENT VALUE CALIBRATION records from E2");

  Set_UoffsetA = e2Read_16b(e2_UoffsetA);
  Set_UoffsetB = e2Read_16b(e2_UoffsetB);
  Set_UoffsetC = e2Read_16b(e2_UoffsetC);

  Set_IoffsetA = e2Read_16b(e2_IoffsetA);
  Set_IoffsetB = e2Read_16b(e2_IoffsetB);
  Set_IoffsetC = e2Read_16b(e2_IoffsetC);

  Set_UgainA = e2Read_16b(e2_UgainA);
  Set_UgainB = e2Read_16b(e2_UgainB);
  Set_UgainC = e2Read_16b(e2_UgainC);

  Set_IgainA = e2Read_16b(e2_IgainA);
  Set_IgainB = e2Read_16b(e2_IgainB);
  Set_IgainC = e2Read_16b(e2_IgainC);

  Set_IgainN = e2Read_16b(e2_IgainN);
  Set_IoffsetN = e2Read_16b(e2_IoffsetN);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Save of ATM90E36A variables and registers in E2 
void SaveE2_M90_Ram(void){ 
  SaveE2_Config_Ram();
  SaveE2_Cal_Ram();
  SaveE2_Harm_Ram();
  SaveE2_Adj_Ram();
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Save of ATM90E36A variables and registers in E2  
void SaveE2_Config_Ram(void){
  Serial.println();
  Serial.println("Saving in E2 the FUNCTION CONFIGURATION registers in RAM");

   e2Write_16b(e2_PLconstH, Set_PLconstH);
   e2Write_16b(e2_PLconstL, Set_PLconstL);

  e2Write_16b(e2_MMode0, Set_MMode0);
  e2Write_16b(e2_MMode1, Set_MMode1);

  e2Write_16b(e2_PStartTh, Set_PStartTh);
  e2Write_16b(e2_QStartTh, Set_QStartTh);
  e2Write_16b(e2_SStartTh, Set_SStartTh);
  e2Write_16b(e2_PPhaseTh, Set_PPhaseTh);
  e2Write_16b(e2_QPhaseTh, Set_QPhaseTh);
  e2Write_16b(e2_SPhaseTh, Set_SPhaseTh);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Save of ATM90E36A variables and registers in E2 
void SaveE2_Cal_Ram(void){
  Serial.println();
  Serial.println("Storing in E2 the ENERGY METERING CALIBRATION registers in RAM");

  e2Write_16b(e2_PoffsetA, Set_PoffsetA);
  e2Write_16b(e2_QoffsetA, Set_QoffsetA);
  e2Write_16b(e2_PoffsetB, Set_PoffsetB);
  e2Write_16b(e2_QoffsetB, Set_QoffsetB);
  e2Write_16b(e2_PoffsetC, Set_PoffsetC);
  e2Write_16b(e2_QoffsetC, Set_QoffsetC);

  e2Write_16b(e2_PQGainA, Set_PQGainA);
  e2Write_16b(e2_PhiA, Set_PhiA);
  e2Write_16b(e2_PQGainB, Set_PQGainB);
  e2Write_16b(e2_PhiB, Set_PhiB);
  e2Write_16b(e2_PQGainC, Set_PQGainC);
  e2Write_16b(e2_PhiC, Set_PhiC);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Save of ATM90E36A variables and registers in E2  
void SaveE2_Harm_Ram(void){
  Serial.println();
  Serial.println("Storing in E2 the FUNDAMENTAL/ARMONIC ENERGY METERING CALIBRATION registers of the RAM");

  e2Write_16b(e2_PoffsetAF, Set_PoffsetAF);
  e2Write_16b(e2_PoffsetBF, Set_PoffsetBF);
  e2Write_16b(e2_PoffsetCF, Set_PoffsetCF);

  e2Write_16b(e2_PGainAF, Set_PGainAF);
  e2Write_16b(e2_PGainBF, Set_PGainBF);
  e2Write_16b(e2_PGainCF, Set_PGainCF);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Save of ATM90E36A variables and registers in E2
void SaveE2_Adj_Ram(void){
  Serial.println();
  Serial.println("Storing in E2 the MEASUREMENT VALUE CALIBRATION registers of the RAM");

  e2Write_16b(e2_UoffsetA, Set_UoffsetA);
  e2Write_16b(e2_UoffsetB, Set_UoffsetB);
  e2Write_16b(e2_UoffsetC, Set_UoffsetC);

  e2Write_16b(e2_IoffsetA, Set_IoffsetA);
  e2Write_16b(e2_IoffsetB, Set_IoffsetB);
  e2Write_16b(e2_IoffsetC, Set_IoffsetC);

  e2Write_16b(e2_UgainA, Set_UgainA);
  e2Write_16b(e2_UgainB, Set_UgainB);
  e2Write_16b(e2_UgainC, Set_UgainC);

  e2Write_16b(e2_IgainA, Set_IgainA);
  e2Write_16b(e2_IgainB,  Set_IgainB);
  e2Write_16b(e2_IgainC, Set_IgainC);

  e2Write_16b(e2_IgainN, Set_IgainN);
  e2Write_16b(e2_IoffsetN, Set_IoffsetN);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calibration routine of all registers of the M90E36A
void Calib_M90(void){
  Load_Ram_M90();
  Ing_ConfigRegs();
  Ing_AmplifDft();
  Ing_AdjRegs();
  Ing_CalRegs();
  Ing_HarmRegs();
  Serial.println(F("END OF CALIBRATION PROCEDURE"));
  Serial.println();
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine to enter the configuration registers CONFIGURATION REGISTERS of the M90E36A
void Ing_ConfigRegs(void){
  uint32_t ingreso = 0, aux;
  uint16_t aux1;
  byte inCaracter = 0;

  Serial.println();
  Serial.println(F("DO YOU WANT TO CHANGE THE BASIC CONFIGURATION REGISTERS? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Config_OK, no_Calibrado);
    Serial.println(F("START CALIBRATION OF BASIC CONFIGURATION PARAMETERS"));
    Serial.println(F("Enter Metering constant (MC) in pulses/kWh"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
      ingreso = Serial.parseInt();
    }
    aux = (uint32_t)(450000000000 / ingreso);
    Set_PLconstH = (uint16_t)(aux >> 16);
    Set_PLconstL = (uint16_t) aux;
    
    aux1 = 0x0087;         //Default
    do{
      Serial.println(F("50/60 Hz? --> enter 0/1 respectively"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(ingreso > 1);
    if(ingreso == 1){aux1 = aux1 | 0x1000;}
  
    do{
      Serial.println(F("Measurement without/with DC? --> enter 0/1 respectively"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(ingreso > 1);
    if(ingreso == 1){aux1 = aux1 | 0x0800;}
  
    do{
      Serial.println(F("Enter CT/Rogowsky power adapter? --> enter 0/1 respectively"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(ingreso > 1);
    if(ingreso == 1){aux1 = aux1 | 0x0400;}
  
    do{
      Serial.println(F("Enter type of measurement 3P4W/3P3W? --> enter 0/1 respectively"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(ingreso > 1);
    if(ingreso == 1){aux1 = aux1 | 0x0100;}
    Set_MMode0 = aux1;
  
    aux1 = 0x0000;            //Default
    do{
      Serial.println(F("Enter digital gain (DPGA) for all 4 current channels --> enter Multiplier 1/2/4/8"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(!((ingreso == 1) || (ingreso == 2) || (ingreso == 4) || (ingreso == 8)));
    switch(ingreso){
      case 2:
        aux1 = aux1 |  0x4000;
        break;
      case 4:
        aux1 = aux1 |  0x8000;
        break;
      case 8:
        aux1 = aux1 |  0xC000;
        break;
      default:
        break;
    }
    do{
      Serial.println(F("Enter Analogue Gain (ADC) for all 7 channels --> enter Multiplier 1/2/4"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(!((ingreso == 1) || (ingreso == 2) || (ingreso == 4)));
    switch(ingreso){
      case 2:
        aux1 = aux1 |  0x1555;
        break;
      case 4:
        aux1 = aux1 |  0x2AAA;
        break;
      default:
        break;
    }
    Set_MMode1 = aux1;
  }
  
  Set_PStartTh = 0x0000;
  Set_QStartTh = 0x0000;
  Set_SStartTh = 0x0000;
  Set_PPhaseTh = 0x0000;
  Set_QPhaseTh = 0x0000;
  Set_SPhaseTh = 0x0000;

  Serial.println();
  Serial.println(F("DO YOU WANT TO CHANGE THE STARTING POWER THRESHOLD? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    Serial.println(F("Ingrese Irms nominal inferior del rango de trabajo en Amps"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
      ingreso = Serial.parseInt();
    }
    float apaSPT = ingreso * (0.1 / 100) * 3  * (50 / 100) ;          //3 * (50% of the start-up current), which in turn is 0.1% of the Imin
    float epaSPT = ingreso * (0.1 / 100) * (10 / 100);                //10% of the start-up current), which in turn is 0.1% of the Imin  
    
    Serial.println(F("Enter nominal working Vrms in Volts"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
      ingreso = Serial.parseInt();
    }
    apaSPT = apaSPT * ingreso / 0.00032;
    epaSPT = epaSPT * ingreso / 0.00032;
    
    Set_PStartTh = int(apaSPT);
    Set_QStartTh = int(apaSPT);
    Set_SStartTh = int(apaSPT);
    Set_PPhaseTh = int(epaSPT);
    Set_QPhaseTh = int(epaSPT);
    Set_SPhaseTh = int(epaSPT);
  }

  Serial.println(F("DO YOU WANT TO SAVE THESE NEW CALIBRATION VALUES IN THE M90 AND E2? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    IniReg_Config_Ram();
    SaveE2_Config_Ram();
    e2prom_write(e2_M90_Config_OK, Calibrado);
    Serial.println(F("M90E36A upgraded and E2 upgraded"));
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine to enter the configuration of the amplifier registers for DFT ANALYSIS of the M90E36A
void Ing_AmplifDft(void){
  uint32_t ingreso = 0;
  uint16_t aux1 = 0x0000;
  byte inCaracter = 0;

  Serial.println();
  Serial.println(F("DO YOU WANT TO CHANGE THE AMPLIFICATION REGISTERS FOR DFT ANALYSIS? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    Serial.println(F("START OF AMPLIFICATION INPUT FOR DFT ANALYSIS"));
    do{
      Serial.println(F("Enter voltage gain for DFT analysis --> enter Multiplier 1/2/4/8"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(!((ingreso == 1) || (ingreso == 2) || (ingreso == 4) || (ingreso == 8)));
    switch(ingreso){
      case 2:
        aux1 = aux1 | 0x2A00;
        break;
      case 4:
        aux1 = aux1 | 0x5400;      
        break;
      case 8:
        aux1 = aux1 | 0x7E00;
        break;      
      default:
        break;
    }
    do{
      Serial.println(F("Enter current gain for DFT analysis --> enter Multiplier 1/2/4/8/16/32/32/64/128"));
      while (Serial.available() == 0);
      while (Serial.available() > 0) {
        ingreso = Serial.parseInt();
      }
    }while(!((ingreso == 1) || (ingreso == 2) || (ingreso == 4) || (ingreso == 8) || (ingreso == 16) || (ingreso == 32) || (ingreso == 64) || (ingreso == 128)));
    switch(ingreso){
      case 2:
        aux1 = aux1 | 0x0049;
        break;
      case 4:
        aux1 = aux1 | 0x0092;      
        break;
      case 8:
        aux1 = aux1 | 0x00DB;
        break;      
      case 16:
        aux1 = aux1 | 0x0124;
        break;
      case 32:
        aux1 = aux1 | 0x016D;      
        break;
      case 64:
        aux1 = aux1 | 0x01B6;
        break;      
      case 128:
        aux1 = aux1 | 0x01FF;
        break;      
      default:
        break;
    }
  }
  
  Serial.println(F("DO YOU WANT TO SAVE THESE NEW AMPLIFICATION VALUES IN THE M90 AND E2? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2Write_16b(e2_M90_DftScale, aux1);                  //Grabo en E2 las amplificaciones
    write_M90_1RC(DFT_SCALE, aux1);                       //Grabo en el equipo las amplificaciones
    Serial.println(F("M90E36A upgraded and E2 upgraded"));
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine for entering the MEASUREMENT REGISTERS configuration of the M90E36A
void Ing_AdjRegs(void){
  uint32_t aux;
  byte inCaracter = 0;
  float variable, referencia = 0;
   
  Serial.println();

  Serial.println(F("DO YOU WANT TO CALIBRATE THE VOLTAGE AND CURRENT OFFSETS OF CHANNEL A? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("START CALIBRATING VOLTAGE and CURRENT OFFSETS CHANNEL A"));
    Serial.println(F("Set in the bank Ub=Uc=Unom , Ua=0, Ia=0, Ia=0 and then press a key"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    aux = read_Parametro(UrmsA, UrmsALSB);
    aux = aux >> 7;
    Set_UoffsetA = (uint16_t)((~aux) + 1);
    aux = read_Parametro(IrmsA, IrmsALSB);
    aux = aux >> 7;
    Set_IoffsetA = (uint16_t)((~aux) + 1);
  }

  Serial.println(F("DO YOU WANT TO CALIBRATE THE VOLTAGE AND CURRENT OFFSETS OF CHANNEL B? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Uc=Unom , Ub=0, Ib=0 and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    aux = read_Parametro(UrmsB, UrmsBLSB);
    aux = aux >> 7;
    Set_UoffsetB = (uint16_t)((~aux) + 1);
    aux = read_Parametro(IrmsB, IrmsBLSB);
    aux = aux >> 7;
    Set_IoffsetB = (uint16_t)((~aux) + 1);
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE VOLTAGE AND CURRENT OFFSETS OF CHANNEL C? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Unom , Uc=0, Ic=0 and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    aux = read_Parametro(UrmsC, UrmsCLSB);
    aux = aux >> 7;
    Set_UoffsetC = (uint16_t)((~aux) + 1);
    aux = read_Parametro(IrmsC, IrmsCLSB);
    aux = aux >> 7;
    Set_IoffsetC = (uint16_t)((~aux) + 1);
  }
  
  Serial.println(F("DO YOU WANT TO SAVE THESE NEW CALIBRATION VALUES IN THE M90 AND E2? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    IniReg_Adj_Ram();
    SaveE2_Adj_Ram();
    e2prom_write(e2_M90_Adj_OK, Calibrado);
    Serial.println(F("M90E36A upgraded and E2 upgraded"));
  }

  Serial.println(F("DO YOU WANT TO CALIBRATE THE VOLTAGE GAIN OF CHANNEL A? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Ib=Ic=Inom and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter voltage A reading of the reference meter in Volts."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    variable = 0;
    for(i=1;i<=VECES_READ;i++){
      variable = variable + 0.01 * (read_M90_1RC(UrmsA) + ((read_M90_1RC(UrmsALSB) >> 8) / 256 ));
      delay(1);
    }
    variable = variable / (i - 1);
    variable = referencia * (read_M90_1RC(UgainA)) / variable;
    Set_UgainA = (uint16_t)variable;
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE VOLTAGE GAIN OF CHANNEL B? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Ib=Ic=Inom and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter voltage B reading of the reference meter in Volts."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    variable = 0;
    for(i=1;i<=VECES_READ;i++){
      variable = variable + 0.01 * (read_M90_1RC(UrmsB) + ((read_M90_1RC(UrmsBLSB) >> 8) / 256 ));
      delay(1);
    }
    variable = variable / (i - 1);
    variable = referencia * (read_M90_1RC(UgainB)) / variable;
    Set_UgainB = (uint16_t)variable;
  }

  Serial.println(F("DO YOU WANT TO CALIBRATE THE VOLTAGE GAIN OF CHANNEL C? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Ib=Ic=Inom and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter reference meter C voltage reading in Volts"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    variable = 0;
    for(i=1;i<=VECES_READ;i++){
      variable = variable + 0.01 * (read_M90_1RC(UrmsC) + ((read_M90_1RC(UrmsCLSB) >> 8) / 256 ));
      delay(1);
    }
    variable = variable / (i - 1);
    variable = referencia * (read_M90_1RC(UgainC)) / variable;
    Set_UgainC = (uint16_t)variable;
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE CURRENT GAIN OF CHANNEL A? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Ib=Ic=Inom and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter reference meter A current reading in Amps"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    variable = 0;
    for(i=1;i<=VECES_READ;i++){
      variable = variable + 0.001 * (read_M90_1RC(IrmsA) + ((read_M90_1RC(IrmsALSB) >> 8) / 256 ));
      delay(1);
    }
    variable = variable / (i - 1);
    variable = referencia * (read_M90_1RC(IgainA)) / variable;
    Set_IgainA = (uint16_t)variable;
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE CURRENT GAIN OF CHANNEL B? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Ib=Ic=Inom and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter reference meter B current reading in Amps"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    variable = 0;
    for(i=1;i<=VECES_READ;i++){
      variable = variable + 0.001 * (read_M90_1RC(IrmsB) + ((read_M90_1RC(IrmsBLSB) >> 8) / 256 ));
      delay(1);
    }
    variable = variable / (i - 1);
    variable = referencia * (read_M90_1RC(IgainB)) / variable;
    Set_IgainB = (uint16_t)variable;
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE CURRENT GAIN OF CHANNEL C? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Ib=Ic=Inom and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter reference meter C current reading in Amps"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    variable = 0;
    for(i=1;i<=VECES_READ;i++){
      variable = variable + 0.001 * (read_M90_1RC(IrmsC) + ((read_M90_1RC(IrmsCLSB) >> 8) / 256 ));
      delay(1);
    }
    variable = variable / (i - 1);
    variable = referencia * (read_M90_1RC(IgainC)) / variable;
    Set_IgainC = (uint16_t)variable;
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE CURRENT GAIN OF CHANNEL N? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Adj_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Ib=Ic=Inom and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter reference meter current reading N in Amps"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    variable = 0;
    for(i=1;i<=VECES_READ;i++){
      variable = variable + 0.001 * (read_M90_1RC(IrmsN1));
      delay(1);
    }
    variable = variable / (i - 1);
    variable = referencia * (read_M90_1RC(IgainN)) / variable;
    Set_IgainN = (uint16_t)variable;
  }

  Serial.println(F("DO YOU WANT TO SAVE THESE NEW CALIBRATION VALUES IN THE M90 AND E2? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    IniReg_Adj_Ram();
    SaveE2_Adj_Ram();
    e2prom_write(e2_M90_Adj_OK, Calibrado);
    Serial.println(F("M90E36A upgraded and E2 upgraded"));
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine to enter the configuration of the ENERGY METERING registers of the M90E36A
void Ing_CalRegs(void){
  int32_t aux;
  byte inCaracter = 0;
  float referencia = 0;
  bool frecuencia;
   
  Serial.println();

  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER OFFSETS OF CHANNEL A? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("START CALIBRATION POWER OFFSETS CHANNEL A"));
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=0 and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    aux = read_Parametro(PmeanA, PmeanALSB);
    aux = (aux * 100000) / 65536;
    aux = aux >> 8;
    Set_PoffsetA = (int16_t)((~aux) + 1);
    aux = read_Parametro(PmeanAF, PmeanAFLSB);
    aux = (aux * 100000) / 65536;
    aux = aux >> 8;
    Set_PoffsetAF = (int16_t)((~aux) + 1);
    aux = read_Parametro(QmeanA, QmeanALSB);
    aux = (aux * 100000) / 65536;
    aux = aux >> 8;
    Set_QoffsetA = (int16_t)((~aux) + 1);
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER OFFSETS OF CHANNEL B? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ib=0 and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    aux = read_Parametro(PmeanB, PmeanBLSB);
    aux = (aux * 100000) /65536;
    aux = aux >> 8;
    Set_PoffsetB = (int16_t)((~aux) + 1);
    aux = read_Parametro(PmeanBF, PmeanBFLSB);
    aux = (aux * 100000) / 65536;
    aux = aux >> 8;
    Set_PoffsetBF = (int16_t)((~aux) + 1);
    aux = read_Parametro(QmeanB, QmeanBLSB);
    aux = (aux * 100000) /65536;
    aux = aux >> 8;
    Set_QoffsetB = (int16_t)((~aux) + 1);
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER OFFSETS OF CHANNEL C? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ic=0 and then press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    aux = read_Parametro(PmeanC, PmeanCLSB);
    aux = (aux * 100000) / 65536;
    aux = aux >> 8;
    Set_PoffsetC = (int16_t)((~aux) + 1);
    aux = read_Parametro(PmeanCF, PmeanCFLSB);
    aux = (aux * 100000) / 65536;
    aux = aux >> 8;
    Set_PoffsetCF = (int16_t)((~aux) + 1);
    aux = read_Parametro(QmeanC, QmeanCLSB);
    aux = (aux * 100000) / 65536;
    aux = aux >> 8;
    Set_QoffsetC = (int16_t)((~aux) + 1);
  }

  Serial.println(F("DO YOU WANT TO SAVE THESE NEW CALIBRATION VALUES IN THE M90 AND E2? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    IniReg_Cal_Ram();
    SaveE2_Cal_Ram();
    IniReg_Harm_Ram();
    SaveE2_Harm_Ram();
    e2prom_write(e2_M90_Cal_OK, Calibrado);
    Serial.println(F("M90E36A upgraded and E2 upgraded"));
  }

  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER GAIN OF CHANNEL A? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Inom, Ib=Ic=0, PF=1.0 (Pulses on CF1) and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter energy measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    Set_PQGainA = (int16_t) int((-referencia/(1+referencia)) * pow(2, 15));
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER GAIN OF CHANNEL B? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ib=Inom, Ia=Ic=0, PF=1.0 (Pulses on CF1) and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter energy measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    Set_PQGainB = (int16_t) int((-referencia/(1+referencia)) * pow(2, 15));
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER GAIN OF CHANNEL C? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ic=Inom, Ia=Ib=0, PF=1.0 (Pulses on CF1) and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter energy measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    Set_PQGainC = (int16_t) int((-referencia/(1+referencia)) * pow(2, 15));
  }

  frecuencia = bitRead(read_M90_1RC(MMode0), 12);
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE ENERGY PHASE ANGLE OF CHANNEL A? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Inom, Ib=Ic=0, PF=0.5L and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter phase measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    if(frecuencia == 0){
      Set_PhiA = (int16_t) referencia * 3763.739;
    }else{
      Set_PhiA = (int16_t) referencia * 3136.449;
    }
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE ENERGY PHASE ANGLE OF CHANNEL B? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ib=Inom, Ia=Ic=0, PF=0.5L and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter phase measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    if(frecuencia == 0){
      Set_PhiB = (int16_t) referencia * 3763.739;
    }else{
      Set_PhiB = (int16_t) referencia * 3136.449;
    }
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE ENERGY PHASE ANGLE OF CHANNEL C? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Cal_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ic=Inom, Ia=Ib=0, PF=0.5L and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter phase measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    if(frecuencia == 0){
      Set_PhiC = (int16_t) referencia * 3763.739;
    }else{
      Set_PhiC = (int16_t) referencia * 3136.449;
    }
  }
  
  Serial.println(F("DO YOU WANT TO SAVE THESE NEW CALIBRATION VALUES IN THE M90 AND E2? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    IniReg_Cal_Ram();
    SaveE2_Cal_Ram();
    e2prom_write(e2_M90_Cal_OK, Calibrado);
    Serial.println(F("M90E36A upgraded and E2 upgraded"));
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Routine to enter the configuration of the ENERGY METERING registers of the M90E36A
void Ing_HarmRegs(void){
  byte inCaracter = 0;
  float referencia = 0;
   
  Serial.println();

  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER GAIN OF THE FUNDAMENTAL OF CHANNEL A? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Harm_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ia=Inom, Ib=Ic=0, PF=1.0 (Pulses in CF3) and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter energy measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    Set_PGainAF = (int16_t) int((-referencia/(1+referencia)) * pow(2, 15));
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER GAIN OF THE FUNDAMENTAL OF CHANNEL B? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Harm_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ib=Inom, Ia=Ic=0, PF=1.0 (Pulses in CF3) and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter energy measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    Set_PGainBF = (int16_t) int((-referencia/(1+referencia)) * pow(2, 15));
  }
  
  Serial.println(F("DO YOU WANT TO CALIBRATE THE POWER GAIN OF THE C CHANNEL FUNDAMENTAL? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    e2prom_write(e2_M90_Harm_OK, no_Calibrado);
    Serial.println(F("Set in the bank Ua=Ub=Uc=Unom , Ic=Inom, Ia=Ib=0, PF=1.0 (Pulses on CF3) and press a key."));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {inCaracter = Serial.read();}
    Serial.println(F("Enter energy measurement error reading"));
    while (Serial.available() == 0);
    while (Serial.available() > 0) {
        referencia = Serial.parseFloat();
    }
    Set_PGainCF = (int16_t) int((-referencia/(1+referencia)) * pow(2, 15));
  }
  
  Serial.println(F("DO YOU WANT TO SAVE THESE NEW CALIBRATION VALUES IN THE M90 AND E2? --> y/n"));
  while (Serial.available() == 0);
  while (Serial.available() > 0) {inCaracter = Serial.read();}
  if(inCaracter == 's'){
    IniReg_Harm_Ram();
    SaveE2_Harm_Ram();
    e2prom_write(e2_M90_Harm_OK, Calibrado);
    Serial.println(F("M90E36A upgraded and E2 upgraded"));
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
void ini_Temperatura(void){
  write_M90_1RC(Temp_cfg1, 0xAA55);
  delay(1);
  write_M90_1RC(Temp_cfg2, 0x5122);
  delay(1);
  write_M90_1RC(Temp_cfg3, 0x012B);
  delay(1);
  write_M90_1RC(Temp_cfg1, 0x0000);
}

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
void ini_AmplifDft(void){
  uint16_t aux1;
  
  aux1 = e2Read_16b(e2_M90_DftScale);                   //Leo of the E2 amplifications
  write_M90_1RC(DFT_SCALE, aux1);                       // I record the amplifications in the equipment
}

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads a complete parameter, i.e. the high part (16 bits) + the low part (16 bits)
//Ex.: ((UrmsA << 16) + UrmsALSB) - Uses read_M90_1R
int32_t  read_Parametro(uint16_t regH_addr, uint16_t regL_addr)
{
  int32_t  val,val_h,val_l;
  int i;

  val_h = 0;
  val_l = 0;
  for(i=1;i<=VECES_READ;i++){
    val_h = val_h + (int32_t)read_M90_1RC(regH_addr);
    val_l = val_l + (int32_t)read_M90_1RC(regL_addr);
    delay(1);
  }
  val_h = (int32_t)(val_h / (i - 1));
  val_l = (int32_t)(val_l / (i - 1));

  val = val_h << 16;
  val = val | val_l;
  
  return(val);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the requested rms voltage value
float Calcula_Vrms(uint16_t Urms_M90, uint16_t UrmsLSB_M90){
  float variable;
  int i;
  
  variable = 0;
  for(i=1;i<=VECES_READ;i++){
    variable = variable + 0.01 * (read_M90_1RC(Urms_M90) + ((read_M90_1RC(UrmsLSB_M90) >> 8) / 256 ));
  }
  variable = variable / (i - 1);
  return variable;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the requested rms current value
float Calcula_Irms(uint16_t Irms_M90, uint16_t IrmsLSB_M90){
  float variable;
  int i;
  
  variable = 0;
  for(i=1;i<=VECES_READ;i++){
    variable = variable + 0.001 * (read_M90_1RC(Irms_M90) + ((read_M90_1RC(IrmsLSB_M90) >> 8) / 256 ));
  }
  variable = variable / (i - 1);
  return variable;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the requested power value
float Calcula_PQSmean(uint16_t PQSmean_M90, uint16_t PQSmeanLSB_M90){
  int32_t val;
  float variable;
  int8_t signo_Variable = 1;

  val = read_Parametro(PQSmean_M90, PQSmeanLSB_M90);      
  if((val & 0x80000000) == 0x80000000){
    val = (~val)+1;               //Complemento a 2
    signo_Variable = -1;
  }
  
  variable = signo_Variable * (((uint16_t)(val >> 16)) + (((uint16_t)((val >> 8) & 0x000000FF)) / 256)); 
  return variable;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the value of the total power requested
float Calcula_PQSmeanT(uint16_t PQSmeanT_M90, uint16_t PQSmeanTLSB_M90){
  int32_t val;
  float variable;
  int8_t signo_Variable = 1;

  val = read_Parametro(PQSmeanT_M90, PQSmeanTLSB_M90);      
  if((val & 0x80000000) == 0x80000000){
    val = (~val)+1;               //Complemento a 2
    signo_Variable = -1;
  }
  variable = signo_Variable * 4 * (((uint16_t)(val >> 16)) + (((uint16_t)((val >> 8) & 0x000000FF)) / 256)); 
  return variable;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates power factor value - Range: -1 to 1
float Calcula_PF(uint16_t PFmean_M90){
  int8_t signo_Variable = 1;
  uint16_t val;
  float pFactor;
  
  val = read_M90_1RC(PFmean_M90);
  if((val & 0x8000) == 0x8000){
    val = (~val)+1;              //Complement to 2
    signo_Variable = -1;
  }
  val = val & 0x7FFF;
  pFactor = signo_Variable * 0.001 * val;
  return pFactor;  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the harmonic distortion value in % of the voltage and current channels.
float Calcula_THDN_VI(uint16_t THDN){
  float distorsion;
  
  distorsion = 0.01 * read_M90_1RC(THDN);
  return distorsion;  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the fundamental value of the voltage and current channels.
float Calcula_FUND_VI(uint16_t canal_M90){
  float fundamental = 0;
  uint16_t escala, aux;
  
  aux = read_M90_1RC(canal_M90);

  switch(canal_M90){
    case AI_FUND:
      escala = read_M90_1RC(DFT_SCALE) & 0x0007;
      fundamental = (aux >> escala) * 3.2656 * 0.001;
    break;
    case BI_FUND:
      escala = (read_M90_1RC(DFT_SCALE) & 0x0038) >> 3;
      fundamental = (aux >> escala) * 3.2656 * 0.001;
    break;
    case CI_FUND:
      escala = (read_M90_1RC(DFT_SCALE) & 0x01C0) >> 6;
      fundamental = (aux >> escala) * 3.2656 * 0.001;
    break;
    case AV_FUND:
      escala = (read_M90_1RC(DFT_SCALE) & 0x0600) >> 9;
      fundamental = (aux >> escala) * 3.2656 * 0.01;
    break;
    case BV_FUND:
      escala = (read_M90_1RC(DFT_SCALE) & 0x1800) >> 11;
      fundamental = (aux >> escala) * 3.2656 * 0.01;
    break;
    case CV_FUND:
      escala = (read_M90_1RC(DFT_SCALE) & 0x6000) >> 13;
      fundamental = (aux >> escala) * 3.2656 * 0.01;
    break;
    default:
      ;
    break;
  }
  
  return fundamental;  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the value of a harmonic or the total in % of the voltage and current channels.
float Calcula_HR_VIT(uint16_t HRVIT){
  float distorsion;
  
  distorsion = read_M90_1RC(HRVIT) / 163.84;
  return distorsion;  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the value of the frequency in Hz
float Calcula_Frecuencia(void){
  float Frec = read_M90_1RC(M90_Freq) / 100;
  return Frec;  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Calculates the value of the temperature in ºC
float Calcula_Temperatura(void){
  int8_t signo_Variable = 1;
  uint16_t val;
  float Temp;
  
  val = read_M90_1RC(M90_Temp);
  if((val & 0x8000) == 0x8000){
    val = (~val)+1;               //Complement to 2
    signo_Variable = -1;
  }
  Temp = signo_Variable * val;
  return Temp;  
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Prepends a String with the date and time of RTC
//type=false --> for Monitor ||type=true --> for a file
//Comes in auxbuffer[] as: hhmmssddmmaa
String msg_time(bool tipo){

  String s="20";
  s = s + String(char(auxbuffer[11])) + String(char(auxbuffer[12])) + "/";    //year
  s = s + String(char(auxbuffer[9])) + String(char(auxbuffer[10])) + "/";     //month
  s = s + String(char(auxbuffer[7])) + String(char(auxbuffer[8]));            //day
  if(tipo == false){
    s = s + "  ";      
  }else{
    s = s + separador; 
  }
  s = s + String(char(auxbuffer[1])) + String(char(auxbuffer[2])) + ":";      //year
  s = s + String(char(auxbuffer[3])) + String(char(auxbuffer[4])) + ":";      //month 
  s = s + String(char(auxbuffer[5])) + String(char(auxbuffer[6]));            //day  
  if(tipo == true){
    s = s + separador;
  }
  return s;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads and transmits by serial the values of voltages, currents, active and reactive power, apparent power, power factor and harmonic distortion per channel,
// apparent power, power factor and harmonic distortion per channel. Frequency and temperature.
void Transmite_VI_PQS(void){
  serialOutput("", 0, conCR);
  serialOutput(F("------------------------------"), 0, conCR);

  get_time();
  serialOutput(msg_time(false), 0, conCR);
  
  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("UrmsA(V): "), 0, sinCR);
  serialOutput(String(Calcula_Vrms(UrmsA, UrmsALSB),2), 0, conCR);
  serialOutput(F("UrmsB(V): "), 0, sinCR);
  serialOutput(String(Calcula_Vrms(UrmsB, UrmsBLSB),2), 0, conCR);
  serialOutput(F("UrmsC(V): "), 0, sinCR);
  serialOutput(String(Calcula_Vrms(UrmsC, UrmsCLSB),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("IrmsA(A): "), 0, sinCR);
  serialOutput(String(Calcula_Irms(IrmsA, IrmsALSB),2), 0, conCR);
  serialOutput(F("IrmsB(A): "), 0, sinCR);
  serialOutput(String(Calcula_Irms(IrmsB, IrmsBLSB),2), 0, conCR);
  serialOutput(F("IrmsC(A): "), 0, sinCR);
  serialOutput(String(Calcula_Irms(IrmsC, IrmsCLSB),2), 0, conCR);
  serialOutput(F("IrmsN(A): "), 0, sinCR);
  serialOutput(String(Calcula_Irms(IrmsN1, 0),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("PmeanA(W): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(PmeanA, PmeanALSB),2), 0, conCR);
  serialOutput(F("PmeanB(W): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(PmeanB, PmeanBLSB),2), 0, conCR);
  serialOutput(F("PmeanC(W): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(PmeanC, PmeanCLSB),2), 0, conCR);
  serialOutput(F("PmeanT(W): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmeanT(PmeanT, PmeanTLSB),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("QmeanA(VAR): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(QmeanA, QmeanALSB),2), 0, conCR);
  serialOutput(F("QmeanB(VAR): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(QmeanB, QmeanBLSB),2), 0, conCR);
  serialOutput(F("QmeanC(VAR): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(QmeanC, QmeanCLSB),2), 0, conCR);
  serialOutput(F("QmeanT(VAR): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmeanT(QmeanT, QmeanTLSB),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("SmeanA(VA): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(SmeanA, SmeanALSB),2), 0, conCR);
  serialOutput(F("SmeanB(VA): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(SmeanB, SmeanBLSB),2), 0, conCR);
  serialOutput(F("SmeanC(VA): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmean(SmeanC, SmeanCLSB),2), 0, conCR);
  serialOutput(F("SmeanT(VA): "), 0, sinCR);
  serialOutput(String(Calcula_PQSmeanT(SAmeanT, SAmeanTLSB),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("PFmeanA: "), 0, sinCR);
  serialOutput(String(Calcula_PF(PFmeanA),2), 0, conCR);
  serialOutput(F("PFmeanB: "), 0, sinCR);
  serialOutput(String(Calcula_PF(PFmeanB),2), 0, conCR);
  serialOutput(F("PFmeanC: "), 0, sinCR);
  serialOutput(String(Calcula_PF(PFmeanC),2), 0, conCR);
  serialOutput(F("PFmeanT: "), 0, sinCR);
  serialOutput(String(Calcula_PF(PFmeanT),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);
  
  serialOutput(F("THDNUA(%): "), 0, sinCR);
  serialOutput(String(Calcula_THDN_VI(THDNUA),2), 0, conCR);
  serialOutput(F("THDNUB(%): "), 0, sinCR);
  serialOutput(String(Calcula_THDN_VI(THDNUB),2), 0, conCR);
  serialOutput(F("THDNUC(%): "), 0, sinCR);
  serialOutput(String(Calcula_THDN_VI(THDNUC),2), 0, conCR);
  serialOutput(F("THDNIA(%): "), 0, sinCR);
  serialOutput(String(Calcula_THDN_VI(THDNIA),2), 0, conCR);
  serialOutput(F("THDNIB(%): "), 0, sinCR);
  serialOutput(String(Calcula_THDN_VI(THDNIB),2), 0, conCR);
  serialOutput(F("THDNIC(%): "), 0, sinCR);
  serialOutput(String(Calcula_THDN_VI(THDNIC),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);
  
  serialOutput(F("Frec.(Hz): "), 0, sinCR);
  serialOutput(String(Calcula_Frecuencia(),2), 0, conCR);
  serialOutput(F("Temp.("), 0, sinCR);
  serialOutput(String(char(grados)), 0, sinCR);
  serialOutput(F("C): "), 0, sinCR);
  serialOutput(String(Calcula_Temperatura(),2), 0, conCR);

  serialOutput(F("------------------------------"), 0, conCR);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads and records in SD memory the values of voltages, currents, active power, reactive power,
// apparent power, power factor and harmonic distortion per channel. Frequency and temperature.
void registra_M90(void){
  get_time();
  
//Assembled start of the measurement message
  msg[numeroMedWrite] = msg_time(true);     //Day and time
  
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Vrms(UrmsA, UrmsALSB),2) + separador;          //RMS voltages
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Vrms(UrmsB, UrmsBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Vrms(UrmsC, UrmsCLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsA, IrmsALSB),2) + separador;          //RMS currents
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsB, IrmsBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsC, IrmsCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsN1, 0),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(PmeanA, PmeanALSB),2) + separador;      //Active power
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(PmeanB, PmeanBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(PmeanC, PmeanCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmeanT(PmeanT, PmeanTLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(QmeanA, QmeanALSB),2) + separador;     //Active power
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(QmeanB, QmeanBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(QmeanC, QmeanCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmeanT(QmeanT, QmeanTLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(SmeanA, SmeanALSB),2) + separador;     //Apparent power
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(SmeanB, SmeanBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(SmeanC, SmeanCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmeanT(SAmeanT, SAmeanTLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanA),2) + separador;                    //Power factor
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanC),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanT),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNUA),2) + separador;                //Harmonic distortion
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNUB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNUC),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNIA),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNIB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNIC),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Frecuencia(),2) + separador;                   //Network frequency
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Temperatura(),2) + separador;                  //Housing temperature

  msg[numeroMedWrite] = msg[numeroMedWrite] + LF;


//End message

//Uncomment the following line to display the message with the measurement on the monitor
// serialOutputTmp(msg[numeroMedWrite], 0, sinCR);
  
  numeroMedWrite++;
  if(numeroMedWrite >= BuffMedicionesMax){
    buff_Lleno = true;
    numeroMedWrite = 0;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads and transmits by serial the values of the harmonic and total voltage components
//for each channel (A, B, C)


void Transmite_HRV(void){
  uint16_t j_HR, i_HR;
  
  serialOutput("", 0, conCR);
  serialOutput(F("------------------------------"), 0, conCR);

  get_time();
  serialOutput(msg_time(false), 0, conCR);
  

  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("AV_FUND(V): "), 0, sinCR);
  serialOutput(String(Calcula_FUND_VI(AV_FUND),2), 0,conCR);                 //Fundamental of channel A

  i_HR = 2;
  for(j_HR=AV_HR2;j_HR<=(AV_HR2+MaxCompArm-2);j_HR++){
    serialOutput(F("AV_HR"), 0, sinCR);
    if(i_HR < 10) serialOutput(F("0"), 0, sinCR);
    serialOutput(String(i_HR), 0, sinCR);
    serialOutput(F("(%): "), 0, sinCR);
    serialOutput(String(Calcula_HR_VIT(j_HR),2), 0,conCR);                   //Channel A harmonics
    i_HR++;
  }
  serialOutput(F("AV_THD(%):  "), 0, sinCR);
  serialOutput(String(Calcula_HR_VIT(AV_THD),2), 0,conCR);                   //Total harmonics of channel A
  
  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("BV_FUND(V): "), 0, sinCR);
  serialOutput(String(Calcula_FUND_VI(BV_FUND),2), 0,conCR);                 //Fundamental of channel A

  i_HR = 2;
  for(j_HR=BV_HR2;j_HR<=(BV_HR2+MaxCompArm-2);j_HR++){
    serialOutput(F("BV_HR"), 0, sinCR);
    if(i_HR < 10) serialOutput(F("0"), 0, sinCR);
    serialOutput(String(i_HR), 0, sinCR);
    serialOutput(F("(%): "), 0, sinCR);
    serialOutput(String(Calcula_HR_VIT(j_HR),2), 0,conCR);                   //Channel A harmonics
    i_HR++;
  }
  serialOutput(F("BV_THD(%):  "), 0, sinCR);
  serialOutput(String(Calcula_HR_VIT(BV_THD),2), 0,conCR);                   //Total harmonics of channel A
  
  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("CV_FUND(V): "), 0, sinCR);
  serialOutput(String(Calcula_FUND_VI(CV_FUND),2), 0,conCR);                  //Fundamental of channel A

  i_HR = 2;
  for(j_HR=CV_HR2;j_HR<=(CV_HR2+MaxCompArm-2);j_HR++){
    serialOutput(F("CV_HR"), 0, sinCR);
    if(i_HR < 10) serialOutput(F("0"), 0, sinCR);
    serialOutput(String(i_HR), 0, sinCR);
    serialOutput(F("(%): "), 0, sinCR);
    serialOutput(String(Calcula_HR_VIT(j_HR),2), 0,conCR);                   //Channel A harmonics
    i_HR++;
  }
  serialOutput(F("CV_THD(%):  "), 0, sinCR);
  serialOutput(String(Calcula_HR_VIT(CV_THD),2), 0,conCR);                   //Total harmonics of channel A

  serialOutput(F("------------------------------"), 0, conCR);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads and transmits by serial the values of the harmonic and total current components
//of each channel (A, B, C)


void Transmite_HRI(void){
  uint16_t j_HR, i_HR;
  
  serialOutput("", 0, conCR);
  serialOutput(F("------------------------------"), 0, conCR);

  get_time();
  serialOutput(msg_time(false), 0, conCR);
  

  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("AI_FUND(A): "), 0, sinCR);
  serialOutput(String(Calcula_FUND_VI(AI_FUND),2), 0,conCR);                 //Fundamental of channel A

  i_HR = 2;
  for(j_HR=AI_HR2;j_HR<=(AI_HR2+MaxCompArm-2);j_HR++){
    serialOutput(F("AI_HR"), 0, sinCR);
    if(i_HR < 10) serialOutput(F("0"), 0, sinCR);
    serialOutput(String(i_HR), 0, sinCR);
    serialOutput(F("(%): "), 0, sinCR);
    serialOutput(String(Calcula_HR_VIT(j_HR),2), 0,conCR);                   //Channel A harmonics
    i_HR++;
  }
  serialOutput(F("AI_THD(%):  "), 0, sinCR);
  serialOutput(String(Calcula_HR_VIT(AI_THD),2), 0,conCR);                   //Total harmonics of channel A
  
  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("BI_FUND(A): "), 0, sinCR);
  serialOutput(String(Calcula_FUND_VI(BI_FUND),2), 0,conCR);                 //Fundamental of channel B

  i_HR = 2;
  for(j_HR=BI_HR2;j_HR<=(BI_HR2+MaxCompArm-2);j_HR++){
    serialOutput(F("BI_HR"), 0, sinCR);
    if(i_HR < 10) serialOutput(F("0"), 0, sinCR);
    serialOutput(String(i_HR), 0, sinCR);
    serialOutput(F("(%): "), 0, sinCR);
    serialOutput(String(Calcula_HR_VIT(j_HR),2), 0,conCR);                   //B-channel harmonics
    i_HR++;
  }
  serialOutput(F("BI_THD(%):  "), 0, sinCR);
  serialOutput(String(Calcula_HR_VIT(BI_THD),2), 0,conCR);                   //Total harmonics of channel B
  
  serialOutput(F("------------------------------"), 0, conCR);

  serialOutput(F("CI_FUND(A): "), 0, sinCR);
  serialOutput(String(Calcula_FUND_VI(CI_FUND),2), 0,conCR);                 //Fundamental of channel C

  i_HR = 2;
  for(j_HR=CI_HR2;j_HR<=(CI_HR2+MaxCompArm-2);j_HR++){
    serialOutput(F("CI_HR"), 0, sinCR);
    if(i_HR < 10) serialOutput(F("0"), 0, sinCR);
    serialOutput(String(i_HR), 0, sinCR);
    serialOutput(F("(%): "), 0, sinCR);
    serialOutput(String(Calcula_HR_VIT(j_HR),2), 0,conCR);                   //C-channel harmonics
    i_HR++;
  }
  serialOutput(F("CI_THD(%):  "), 0, sinCR);
  serialOutput(String(Calcula_HR_VIT(CI_THD),2), 0,conCR);                   //Total harmonics of channel C

  serialOutput(F("------------------------------"), 0, conCR);
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads and records the values of current harmonics and total distortion in the SD memory.
//per channel (A, B, C)
void registra_M90_HRI(void){
  uint16_t j_HR;
  
  get_time();
  
//Assembled start of the measurement message
  msg[numeroMedWrite] = msg_time(true);     //Day and time

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_FUND_VI(AI_FUND),2) + separador;                //Channel A current fundamental
  for(j_HR=AI_HR2;j_HR<=(AI_HR2+MaxCompArm-2);j_HR++){
    msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(j_HR),2) + separador;                  //Channel A current harmonics
  }
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(AI_THD),2) + separador;                  //Total current harmonics of channel A

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_FUND_VI(BI_FUND),2) + separador;                 //Fundamental current of channel B
  for(j_HR=BI_HR2;j_HR<=(BI_HR2+MaxCompArm-2);j_HR++){
    msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(j_HR),2) + separador;                   //Channel B current harmonics
  }
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(BI_THD),2) + separador;                   //Total current harmonics of channel B

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_FUND_VI(CI_FUND),2) + separador;                 //C-channel C current fundamental
  for(j_HR=CI_HR2;j_HR<=(CI_HR2+MaxCompArm-2);j_HR++){
    msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(j_HR),2) + separador;                    //C-channel current harmonics
  }
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(CI_THD),2) + separador;                   //Total current harmonics of channel C

  msg[numeroMedWrite] = msg[numeroMedWrite] + LF;

//End message

//Uncomment the following line to display the message with the measurement on the monitor
// serialOutputTmp(msg[numeroMedWrite], 0, sinCR); 
  
  numeroMedWrite++;
  if(numeroMedWrite >= BuffMedicionesMax){
    buff_Lleno = true;
    numeroMedWrite = 0;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Reads and records in SD memory the values of current variables and harmonics and distortion.
//total per channel (A, B, C)
void registra_M90_Todo(void){
  uint16_t j_HR;
  float auxiliar;
  
  get_time();

//Assembled start of the measurement message
  msg[numeroMedWrite] = msg_time(true);     //Day and time
  
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Vrms(UrmsA, UrmsALSB),2) + separador;          //RMS voltages
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Vrms(UrmsB, UrmsBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Vrms(UrmsC, UrmsCLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsA, IrmsALSB),2) + separador;           //Rms currents
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsB, IrmsBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsC, IrmsCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Irms(IrmsN1, 0),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(PmeanA, PmeanALSB),2) + separador;     //Active power
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(PmeanB, PmeanBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(PmeanC, PmeanCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmeanT(PmeanT, PmeanTLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(QmeanA, QmeanALSB),2) + separador;     //Reactive power
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(QmeanB, QmeanBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(QmeanC, QmeanCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmeanT(QmeanT, QmeanTLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(SmeanA, SmeanALSB),2) + separador;     //Apparent power
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(SmeanB, SmeanBLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmean(SmeanC, SmeanCLSB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PQSmeanT(SAmeanT, SAmeanTLSB),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanA),2) + separador;                    //Power factor
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanC),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_PF(PFmeanT),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNUA),2) + separador;                 //Harmonic distortion
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNUB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNUC),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNIA),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNIB),2) + separador;
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_THDN_VI(THDNIC),2) + separador;

  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Frecuencia(),2) + separador;                   //Network frequency
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_Temperatura(),2) + separador;                  //Housing temperature

  auxiliar = Calcula_FUND_VI(AI_FUND);
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(auxiliar,2) + separador;                                 //Channel A current fundamental
  auxiliar = auxiliar / 100;
  for(j_HR=AI_HR2;j_HR<=(AI_HR2+MaxCompArm-2);j_HR++){
    msg[numeroMedWrite] = msg[numeroMedWrite] + String((Calcula_HR_VIT(j_HR) * auxiliar),2) + separador;      //Channel A current harmonics
  }
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(AI_THD),2) + separador;                   //Total current harmonics of channel A

  auxiliar = Calcula_FUND_VI(BI_FUND);
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(auxiliar,2) + separador;                                 //Fundamental current of channel B
  auxiliar = auxiliar / 100;
  for(j_HR=BI_HR2;j_HR<=(BI_HR2+MaxCompArm-2);j_HR++){
    msg[numeroMedWrite] = msg[numeroMedWrite] + String((Calcula_HR_VIT(j_HR) * auxiliar),2) + separador;      //Channel B current harmonics
  }
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(BI_THD),2) + separador;                   //Total current harmonics of channel B

  auxiliar = Calcula_FUND_VI(CI_FUND);
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(auxiliar,2) + separador;                                 //C-channel C current fundamental
  auxiliar = auxiliar / 100;
  for(j_HR=CI_HR2;j_HR<=(CI_HR2+MaxCompArm-2);j_HR++){
    msg[numeroMedWrite] = msg[numeroMedWrite] + String((Calcula_HR_VIT(j_HR) * auxiliar),2) + separador;      //C-channel current harmonics
  }
  msg[numeroMedWrite] = msg[numeroMedWrite] + String(Calcula_HR_VIT(CI_THD),2) + separador;                   //Total current harmonics of channel C

  msg[numeroMedWrite] = msg[numeroMedWrite] + LF;


//End message

//Uncomment the following line to display the message with the measurement on the monitor
// serialOutputTmp(msg[numeroMedWrite], 0, sinCR);     
  
  numeroMedWrite++;
  if(numeroMedWrite >= BuffMedicionesMax){
    buff_Lleno = true;
    numeroMedWrite = 0;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Writes the buffer from RAM to the SD memory up to the "last_Record" value.  
bool graboRam_SD(char *archivo, unsigned int ultimo_Registro){
  File dataFile = SD.open(archivo, FILE_WRITE);  
  
  if (dataFile) {
    for(numeroMedSD=0;numeroMedSD<ultimo_Registro;numeroMedSD++){
      dataFile.print(msg[numeroMedSD]);
    }
    dataFile.close();
    return true;
  } else {
//    serialOutput("Error opening ", 0, sinCR);
//    serialOutput(archivo, 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Prepare all variables and file for datalogging
bool preparo_Registro(void){
  parada();
  numeroMedWrite = 0;
  numeroMedSD = 0;
  registrandoM90_flag = 1;
  e2prom_write(e2_M90_registrando, registrandoM90_flag);      //Save to E2 register flag
  regM90_flag = false;
  buff_Lleno = false;
  rtc_write(rtc_m_st + 7, 0x01);                              //Stores in startup code clock
  rtc_write(rtc_m_st + 9, byte(tiempo_reg));                  //save on clock time recording
  rtc_write(rtc_m_st + 10, byte(tiempo_reg>>8));              //save on clock time recording
  get_time();
  sprintf(ensayo,"%s%s%d%d",archivoBack,".0", (auxbuffer[5] - 48), (auxbuffer[6] - 48));
  p_SerialTmp = p_Serial;

  File dataFile = SD.open(ensayo, FILE_WRITE);  
  if (dataFile) {
    dataFile.print("Day;Time;UrmsA(V);UrmsB(V);UrmsC(V);IrmsA(A);IrmsB(A);IrmsC(A);IrmsN(A);PmeanA(W);PmeanB(W);PmeanC(W);PmeanT(W);QmeanA(VAR);QmeanB(VAR);QmeanC(VAR);QmeanT(VAR);");
    dataFile.println("SmeanA(VA);SmeanB(VA);SmeanC(VA);SmeanT(VA);PFmeanA;PFmeanB;PFmeanC;PFmeanT;THDNUA(%);THDNUB(%);THDNUC(%);THDNIA(%);THDNIB(%);THDNIC(%);Frec.(Hz);Temp.(ºC)");
    dataFile.close();
    return true;
  } else {
    serialOutput("Error opening ", 0, sinCR);
    serialOutput(ensayo, 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------
  
//--------------------------------------------------------------------------------------------------
//Prepare all variables and file for harmonics data logging
bool preparo_RegistroHR(void){
  uint8_t k;
  parada();
  numeroMedWrite = 0;
  numeroMedSD = 0;
  regHR_flag = 1;
  e2prom_write(e2_M90_regHR, regHR_flag);                     //Save to E2 register flag
  e2prom_write(e2_M90_MaxCompArm, MaxCompArm);                //Stores in E2 the maximum required Harmonic
  buff_Lleno = false;
  rtc_write(rtc_m_st + 7, 0x01);                              //Stores in startup code clock

  get_time();
  sprintf(ensayo,"%s%s%d%d",archivoBack,".0", (auxbuffer[5] - 48), (auxbuffer[6] - 48));
  p_SerialTmp = p_Serial;

  File dataFile = SD.open(ensayo, FILE_WRITE);  
  if (dataFile) {
    dataFile.print("Day;Time;");

    dataFile.print("AI_FUND;");
    for(k=2;k<=MaxCompArm;k++){
      dataFile.print("AI_HD"); dataFile.print(k); dataFile.print("(%);");
    }
    dataFile.print("AI_THD(%);");
    
    dataFile.print("BI_FUND;");
    for(k=2;k<=MaxCompArm;k++){
      dataFile.print("BI_HD"); dataFile.print(k); dataFile.print("(%);");
    }
    dataFile.print("BI_THD(%);");
    
    dataFile.print("CI_FUND;");
    for(k=2;k<=MaxCompArm;k++){
      dataFile.print("CI_HD"); dataFile.print(k); dataFile.print("(%);");
    }
    dataFile.println("CI_THD(%)");
    dataFile.close();
    return true;
  } else {
    serialOutput("Error opening  ", 0, sinCR);
    serialOutput(ensayo, 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------
  
//--------------------------------------------------------------------------------------------------
//Prepare all variables and file for harmonics data logging
bool preparo_RegistroTodo(void){
  uint8_t k;
  parada();
  numeroMedWrite = 0;
  numeroMedSD = 0;
  regTodo_flag = 1;
  e2prom_write(e2_M90_regTodo, regTodo_flag);                 //Save to E2 log all flag
  e2prom_write(e2_M90_MaxCompArm, MaxCompArm);                //Stores in E2 the maximum required Harmonic
  buff_Lleno = false;
  rtc_write(rtc_m_st + 7, 0x01);                              //Stores in startup code clock

  get_time();
  sprintf(ensayo,"%s%s%d%d",archivoBack,".0", (auxbuffer[5] - 48), (auxbuffer[6] - 48));
  p_SerialTmp = p_Serial;

  File dataFile = SD.open(ensayo, FILE_WRITE);  
  if (dataFile) {
    dataFile.print("Day;Time;UrmsA(V);UrmsB(V);UrmsC(V);IrmsA(A);IrmsB(A);IrmsC(A);IrmsN(A);PmeanA(W);PmeanB(W);PmeanC(W);PmeanT(W);QmeanA(VAR);QmeanB(VAR);QmeanC(VAR);QmeanT(VAR);");
    dataFile.print("SmeanA(VA);SmeanB(VA);SmeanC(VA);SmeanT(VA);PFmeanA;PFmeanB;PFmeanC;PFmeanT;THDNUA(%);THDNUB(%);THDNUC(%);");
    dataFile.print("THDNIA(%);THDNIB(%);THDNIC(%);");
    dataFile.print("Frec.(Hz);Temp.(ºC);");

    dataFile.print("AI_FUND(A);");
    for(k=2;k<=MaxCompArm;k++){
      dataFile.print("AI_HD"); dataFile.print(k); dataFile.print("(A);");
    }
    dataFile.print("AI_THD(%);");
    
    dataFile.print("BI_FUND(A);");
    for(k=2;k<=MaxCompArm;k++){
      dataFile.print("BI_HD"); dataFile.print(k); dataFile.print("(A);");
    }
    dataFile.print("BI_THD(%);");
    
    dataFile.print("CI_FUND(A);");
    for(k=2;k<=MaxCompArm;k++){
      dataFile.print("CI_HD"); dataFile.print(k); dataFile.print("(A);");
    }
    dataFile.println("CI_THD(%)");
    dataFile.close();
    return true;
  } else {
    serialOutput("Error al abrir ", 0, sinCR);
    serialOutput(ensayo, 0, conCR);
    return false;
  }
}
//--------------------------------------------------------------------------------------------------
  
//--------------------------------------------------------------------------------------------------
void Set_DMA_M90(uint8_t set_dmaCanales){

//Seteo DMACtrl para transmitir en 24 bits
// SPI en DMA: Modo_0, 8bits, 512Khz

//I4=0, I1=0, V1=1, I2=0 |||| V2=0, I3=0, V3=0, PIN_DIR_SEL=1 |||| CH_BIT_WIDTH=01, CLK_IDLE=0, CLK_DRV=0 |||| CLK_DIV=1111
  if(set_dmaCanales == 1) write_M90_1RC(DMACtrl, (uint16_t)0x214F);       //Channel V1                      

//I4=0, I1=1, V1=1, I2=0 |||| V2=0, I3=0, V3=0, PIN_DIR_SEL=1 |||| CH_BIT_WIDTH=01, CLK_IDLE=0, CLK_DRV=0 |||| CLK_DIV=1111
  if(set_dmaCanales == 2) write_M90_1RC(DMACtrl, (uint16_t)0x614F);       //Channels I1 and V1                 

//I4=0, I1=1, V1=1, I2=1 |||| V2=0, I3=0, V3=0, PIN_DIR_SEL=1 |||| CH_BIT_WIDTH=01, CLK_IDLE=0, CLK_DRV=0 |||| CLK_DIV=1011
  if(set_dmaCanales == 3) write_M90_1RC(DMACtrl, (uint16_t)0x714B);       //Channels I1, I2 and V1                  

//I4=0, I1=1, V1=1, I2=1 |||| V2=0, I3=1, V3=0, PIN_DIR_SEL=1 |||| CH_BIT_WIDTH=01, CLK_IDLE=0, CLK_DRV=0 |||| CLK_DIV=1001
  if(set_dmaCanales == 4) write_M90_1RC(DMACtrl, (uint16_t)0x7549);       //Channels I1, I2, I3 and V1                
                                                                  
//I4=1, I1=1, V1=1, I2=1 |||| V2=0, I3=1, V3=0, PIN_DIR_SEL=1 |||| CH_BIT_WIDTH=01, CLK_IDLE=0, CLK_DRV=0 |||| CLK_DIV=0111
  if(set_dmaCanales == 5) write_M90_1RC(DMACtrl, (uint16_t)0xF547);       //Channels I1, I2, I3 and V1                 
                                                                  
//SPI0 pin configuration --> MISO: PA25, (MISO), MOSI: PA26, (MOSI), SCLK: PA27, (SCLK), NSS: PA28, (NPCS0)

  pinMode(MISO,OUTPUT);
  pinMode(MOSI,INPUT);
  pinMode(SCK,INPUT);
  pinMode(CS_M90,INPUT);

//Activate DMA mode on M90E36A
// digitalWrite(M90_dmaCTRL,HIGH);                               

//Transfers pins 25, 26, 27, 28 to function A
  REG_PIOA_ABSR &= 0xE1FFFFFF;

//Power up uC as SPI Slave
//Power up SPI clock
  REG_PMC_PCER0 |= PMC_PCER0_PID24;                               

//Unlock user interface for SPI
// REG_SPI0_WPMR = 0x53504900;                                 
// REG_PIOA_WPMR = 0x50494F00;
  
//Reset SPI0
  REG_SPI0_CR = SPI_CR_SWRST;                                
  delay(1);


//Enables SPI0
  REG_SPI0_CR = SPI_CR_SPIEN;                                    
//Modo Slave
  REG_SPI0_MR = 0x00000010;   

//SPI in MODE_0 and 8 bits transfer.
  SPI0->SPI_CSR[0] = SPI0->SPI_CSR[0] & 0x00FFFF0E;
  SPI0->SPI_CSR[0] = SPI0->SPI_CSR[0] | 0x00000002;
//SPI en MODO_0 y transferencia de 16 bits.
//  SPI0->SPI_CSR[0] = SPI0->SPI_CSR[0] & 0x00FFFF0E;
//  SPI0->SPI_CSR[0] = SPI0->SPI_CSR[0] | 0x00000082;
  
//preparing interruption for SPI
  SPI0->SPI_IER = SPI_IER_RDRF;
  NVIC_EnableIRQ(SPI0_IRQn);

//Lock user interface for SPI
// REG_SPI0_WPMR = 0x53504901;
// REG_PIOA_WPMR = 0x50494F01;

//Activate DMA mode on M90E36A

  digitalWrite(M90_dmaCTRL,HIGH);                                
}

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//-----------------------------Fin Driver M90E36A --------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                                 INTERRUPTIONS
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
void Check_M90E36A(void){
  check_M90 = true;
}

//--------------------------------------------------------------------------------------------------
//Register routine timed EEProm (I2C) data from M90E36A
//Pumem points to the first free memory location
void Reg_M90_Ram(void){
  regM90_flag = true;
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Service routine SPI interrupt in Slave mode
//Interrupt when there is valid data in the receive register
void SPI0_Handler(){
//ISR(SPI_STC_vect){  
  uint32_t Status_RD = SPI0->SPI_SR;

  if(semaforo_DMA == s_Buff1){
    if(Status_RD & SPI_SR_RDRF) {
//    SPI_RD_Data = SPI0->SPI_RDR & SPI_RDR_RD_Msk;
      buff_DMA1[punt_buff_DMA1] = (byte)SPI0->SPI_RDR;
    }
    if(punt_buff_DMA1 < tope_buff_DMA){
       punt_buff_DMA1++;
    } else {
       punt_buff_DMA1 = 0;
       semaforo_DMA = s_Buff2;
       transm_USB = Habilitada;
    }
  } else {
    if(Status_RD & SPI_SR_RDRF) {
  //    SPI_RD_Data = SPI0->SPI_RDR & SPI_RDR_RD_Msk;
      buff_DMA2[punt_buff_DMA2] = (byte)SPI0->SPI_RDR;
    }
    if(punt_buff_DMA2 < tope_buff_DMA){
       punt_buff_DMA2++;
    } else {
       punt_buff_DMA2 = 0;
       semaforo_DMA = s_Buff1;
       transm_USB = Habilitada;
    }
  }
  
// Enviar algo para recibir algo
//  if (Status_RD & SPI_SR_TDRE) {
//    SPI0->SPI_TDR = (uint16_t)SPI_RD_Data;
//  }
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//void ISR_SPI(void){
//  SPI_RD_flag = true; 
//}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Rutina de procesamiento de la interrupción del Timer4 en modo DMA del M90E36A
//Cuando interrumpe significa que para el proceso de DMA
//void DMA_Off(void){
//  Timer4.stop();
//  DMA_Off_Flag = true;
//}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                              Utility routines
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Rotate "value" to the left "shift" positions 
byte rotl_b(byte value, byte shift) {
    if ((shift < 1) || (shift > 7))
      return value;
    return (value << shift) | (value >> (8 - shift));
}
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//Rotate "value" to the right "shift" positions
byte rotr_b(byte value, byte shift) {
    if ((shift < 1) || (shift > 7))
      return value;
    return (value >> shift) | (value << (8 - shift));
}

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//                               END OF THE PROGRAMME
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
