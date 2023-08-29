//-------------------------------------------------------------
//-------------------------------------------------------------

//Definiciones de pines y registros para Microchip/Atmel M90E36A
#define CS_M90 10                  //Pin correspondiente al CS del chip (pueder ser 42[CS3 en J26] o 10[PWM10 en J35])
#define M90_dmaCTRL 42             //Pin correspondiente al dma-Control (pueder ser 42[CS3 en J26] o 11[PWM11 en J35])
#define M90_PM0 12                 //Pin correspondiente a PM0 (Solo para pruebas con placa de Microchip)
#define M90_PM1 13                 //Pin correspondiente a PM1 (Solo para pruebas con placa de microchip)

#define REG_M90_CONF_BEGIN 0x0000  //Primer registro de configuración, calibración, etc.
#define REG_M90_CONF_END 0x006F    //Ultimo registro de configuración, calibración, etc.
#define REG_M90_CONF_TOTAL 112     //Cantidad total de registros de configuración, calibración, etc.

#define REG_M90_MED_BEGIN 0x0080   //Primer registro de mediciones.
#define REG_M90_MED_END 0x01CF     //Ultimo registro de mediciones.
#define REG_M90_MED_TOTAL 336      //Cantidad total de registros de mediciones.

#define REG_M90_HRIA_BEGIN 0x0100  //Primer registro de armónicos de corriente del canal A.
#define REG_M90_HRIA_END 0x011E    //Ultimo registro de armónicos de corriente del canal A.
#define REG_M90_HRIA_TOTAL 31      //Cantidad total de registros de armónicos de corriente del canal A.
#define REG_M90_HRIB_BEGIN 0x0120  //Primer registro de armónicos de corriente del canal B.
#define REG_M90_HRIB_END 0x013E    //Ultimo registro de armónicos de corriente del canal B.
#define REG_M90_HRIB_TOTAL 31      //Cantidad total de registros de armónicos de corriente del canal B.
#define REG_M90_HRIC_BEGIN 0x0140  //Primer registro de armónicos de corriente del canal C.
#define REG_M90_HRIC_END 0x015E    //Ultimo registro de armónicos de corriente del canal C.
#define REG_M90_HRIC_TOTAL 31      //Cantidad total de registros de armónicos de corriente del canal C.

#define REG_M90_HRVA_BEGIN 0x0160  //Primer registro de armónicos de tensión del canal A.
#define REG_M90_HRVA_END 0x017E    //Ultimo registro de armónicos de tensión del canal A.
#define REG_M90_HRVA_TOTAL 31      //Cantidad total de registros de armónicos de tensión del canal A.
#define REG_M90_HRVB_BEGIN 0x0180  //Primer registro de armónicos de tensión del canal B.
#define REG_M90_HRVB_END 0x019E    //Ultimo registro de armónicos de tensión del canal B.
#define REG_M90_HRVB_TOTAL 31      //Cantidad total de registros de armónicos de tensión del canal B.
#define REG_M90_HRVC_BEGIN 0x01A0  //Primer registro de armónicos de tensión del canal C.
#define REG_M90_HRVC_END 0x01BE    //Ultimo registro de armónicos de tensión del canal C.
#define REG_M90_HRVC_TOTAL 31      //Cantidad total de registros de armónicos de tensión del canal C.

#define CONTROL_TIME 20000000      //Tiempo de chequeo de la integridad de los registros
#define VECES_READ 8       //Veces que se lee un registro para el cálculo de variables (V, I, P, etc.)
#define CALIBRACION 0x5678         //Valor para habilitar la escritura de los registros de configuración que correspondan
#define OPERACION 0X8765           //Valor para habilitar la operacion con lo chequeos de error de los registros de configuración que correspondan
#define RESET_M90 0x789A     //Valor para resetear el chip M90E36A
#define no_Calibrado 0xFF    //Indica que un item no esta calibrado
#define Calibrado 0x01       //Indica que un item esta calibrado

//Status and Special Register
#define SoftReset 0x0000           //W Software Reset P 40
#define SysStatus0 SoftReset+1     //R/C System Status 0 P 41
#define SysStatus1 SoftReset+2     //R/C System Status 1 P 41
#define FuncEn0 SoftReset+3        //R/W Function Enable 0 P 43
#define FuncEn1 SoftReset+4        //R/W Function Enable 1 P 43
#define ZXConfig SoftReset+7       //R/W Zero-Crossing Configuration Configuration of ZX0/1/2 pins’ source P 45
#define SagTh SoftReset+8          //R/W Voltage Sag Threshold P 45
#define PhaseLossTh SoftReset+9    //R/W Voltage Phase Losing Threshold Similar to Voltage Sag Threshold register P 45
#define INWarnTh0 SoftReset+10     //R/W Threshold for calculated (Ia + Ib +Ic) Nline rms current Check SysStatus0/1 register. P 46
#define INWarnTh1 SoftReset+11     //R/W Threshold for sampled (from ADC) Nline rms current Check SysStatus0/1 register. P 46
#define THDNUTh SoftReset+12       //R/W Voltage THD Warning Threshold Check SysStatus0/1 register. P 46
#define THDNITh SoftReset+13       //R/W Current THD Warning Threshold Check SysStatus0/1 register. P 46
#define DMACtrl SoftReset+14       //R/W DMA Mode Interface Control DMA mode interface control P 47
#define LastSPIData SoftReset+15   //R Last Read/ Write SPI Value Refer to 4.2.2 Reliability Enhancement Feature P 48 Low Power Mode Register

//Low Power Mode Register
#define DetectCtrl 0x0010          //R/W Current Detect Control P 49
#define DetectTh1 DetectCtrl+1     //R/W Channel 1 current threshold in Detection mode P 50
#define DetectTh2 DetectCtrl+2     //R/W Channel 2 current threshold in Detection mode P 50
#define DetectTh3 DetectCtrl+3     //R/W Channel 3 current threshold in Detection mode P 51
#define PMOffsetA DetectCtrl+4     //R/W Ioffset for phase A in Partial Measurement mode P 52
#define PMOffsetB DetectCtrl+5     //R/W Ioffset for phase B in Partial Measurement mode P 52
#define PMOffsetC DetectCtrl+6     //R/W Ioffset for phase C in Partial Measurement mode P 52
#define PMPGA DetectCtrl+7         //R/W PGAgain Configuration in Partial Measurement mode P 52
#define PMIrmsA DetectCtrl+8       //R Irms for phase A in Partial Measurement mode P 52
#define PMIrmsB DetectCtrl+9       //R Irms for phase B in Partial Measurement mode P 53
#define PMIrmsC DetectCtrl+10      //R Irms for phase C in Partial Measurement mod
#define PMConfig DetectCtrl+11     //R/W Measure configuration in Partial Measurement mode P 53
#define PMAvgSamples DetectCtrl+12 //R/W Number of 8K samples to be averaged in RMS/mean computation P 53
#define PMIrmsLSB DetectCtrl+13    //R LSB bits of PMRrms[A/B/C] It returns MSB of the mean measurement data in Mean value test

//Configuration Registers
#define ConfigStart 0x0030         //R/W Calibration Start Command P 56
#define PLconstH ConfigStart+1     //R/W High Word of PL_Constant P 57
#define PLconstL ConfigStart+2     //R/W Low Word of PL_Constant P 57
#define MMode0 ConfigStart+3       //R/W Metering method configuration P 58
#define MMode1 ConfigStart+4       //R/W PGA gain configuration P 59
#define PStartTh ConfigStart+5     //R/W Active Startup Power Threshold. Refer to Table-5.
#define QStartTh ConfigStart+6     //R/W Reactive Startup Power Threshold.
#define SStartTh ConfigStart+7     //R/W Apparent Startup Power Threshold.
#define PPhaseTh ConfigStart+8     //R/W Startup Power Threshold (Active Energy Accumulation)
#define QPhaseTh ConfigStart+9     //R/W Startup Power Threshold (ReActive Energy Accumulation)
#define SPhaseTh ConfigStart+10    //R/W Startup Power Threshold (Apparent Energy Accumulation)
#define CS_0 ConfigStart+11        //R/W Checksum 0

//Calibration Registers
#define CalStart 0x0040            //R/W Calibration Start Command Refer to Table-6.
#define PoffsetA CalStart+1        //R/W Phase A Active Power Offset P 61
#define QoffsetA CalStart+2        //R/W Phase A Reactive Power Offset P 61
#define PoffsetB CalStart+3        //R/W Phase B Active Power Offset
#define QoffsetB CalStart+4        //R/W Phase B Reactive Power Offset
#define PoffsetC CalStart+5        //R/W Phase C Active Power Offset
#define QoffsetC CalStart+6        //R/W Phase C Reactive Power Offset
#define PQGainA CalStart+7         //R/W Phase A calibration gain P 62
#define PhiA CalStart+8            //R/W Phase A calibration phase angle P 62
#define PQGainB CalStart+9         //R/W Phase B calibration gain
#define PhiB CalStart+10           //R/W Phase B calibration phase angle
#define PQGainC CalStart+11        //R/W Phase C calibration gain
#define PhiC CalStart+12           //R/W Phase C calibration phase angle
#define CS_1 CalStart+13           //R/W Checksum 1

//Fundamental/ Harmonic Energy Calibration registers
#define HarmStart 0x0050           //R/W Harmonic Calibration Startup Command Refer to Table-7.
#define PoffsetAF HarmStart+1      //R/W Phase A Fundamental Active PowerOffset
#define PoffsetBF HarmStart+2      //R/W Phase B Fundamental Active PowerOffset
#define PoffsetCF HarmStart+3      //R/W Phase C Fundamental Active PowerOffset
#define PGainAF HarmStart+4        //R/W Phase A Fundamental Active PowerGain
#define PGainBF HarmStart+5        //R/W Phase B Fundamental Active PowerGain
#define PGainCF HarmStart+6        //R/W Phase C Fundamental Active PowerGain
#define CS_2 HarmStart+7           //R/W Checksum 2

//Measurement Calibration
#define AdjStart 0x0060            //R/W Measurement Calibration Startup Command Refer to Table-8.
#define UgainA AdjStart+1          //R/W Phase A Voltage RMS Gain
#define IgainA AdjStart+2          //R/W Phase A Current RMS Gain
#define UoffsetA AdjStart+3        //R/W Phase A Voltage RMS Offset
#define IoffsetA AdjStart+4        //R/W Phase A Current RMS Offset
#define UgainB AdjStart+5          //R/W Phase B Voltage RMS Gain
#define IgainB AdjStart+6          //R/W Phase B Current RMS Gain
#define UoffsetB AdjStart+7        //R/W Phase B Voltage RMS Offset
#define IoffsetB AdjStart+8        //R/W Phase B Current RMS Offset
#define UgainC AdjStart+9          //R/W Phase C Voltage RMS Gain
#define IgainC AdjStart+10         //R/W Phase C Current RMS Gain
#define UoffsetC AdjStart+11       //R/W Phase C Voltage RMS Offset
#define IoffsetC AdjStart+12       //R/W Phase C Current RMS Offset
#define IgainN AdjStart+13         //R/W Sampled N line Current RMS Gain
#define IoffsetN AdjStart+14       //R/W Sampled N line Current RMS Offset
#define CS_3 AdjStart+15           //R/W Checksum 3

//Energy Register
#define APenergyT 0x0080           //R/C Total Forward Active Energy
#define APenergyA APenergyT+1      //R/C Phase A Forward Active Energy
#define APenergyB APenergyT+2      //R/C Phase B Forward Active Energy
#define APenergyC APenergyT+3      //R/C Phase C Forward Active Energy
#define ANenergyT APenergyT+4      //R/C Total Reverse Active Energy
#define ANenergyA APenergyT+5      //R/C Phase A Reverse Active Energy
#define ANenergyB APenergyT+6      //R/C Phase B Reverse Active Energy
#define ANenergyC APenergyT+7      //R/C Phase C Reverse Active Energy
#define RPenergyT APenergyT+8      //R/C Total Forward Reactive Energy
#define RPenergyA APenergyT+9      //R/C Phase A Forward Reactive Energy
#define RPenergyB APenergyT+10     //R/C Phase B Forward Reactive Energy
#define RPenergyC APenergyT+11     //R/C Phase C Forward Reactive Energy
#define RNenergyT APenergyT+12     //R/C Total Reverse Reactive Energy
#define RNenergyA APenergyT+13     //R/C Phase A Reverse Reactive Energy
#define RNenergyB APenergyT+14     //R/C Phase B Reverse Reactive Energy
#define RNenergyC APenergyT+15     //R/C Phase C Reverse Reactive Energy
#define SAenergyT APenergyT+16     //R/C Total (Arithmetic Sum) Apparent Energy
#define SenergyA APenergyT+17      //R/C Phase A Apparent Energy
#define SenergyB APenergyT+18      //R/C Phase B Apparent Energy
#define SenergyC APenergyT+19      //R/C Phase C Apparent Energy
#define SVenergyT APenergyT+20     //R/C (Vector Sum) Total Apparent Energy
#define EnStatus0 APenergyT+21     //R Metering Status 0 P 66
#define EnStatus1 APenergyT+22     //R Metering Status 1 P 66
#define SVmeanT APenergyT+24       //R (Vector Sum) Total Apparent Power
#define SVmeanTLSB APenergyT+25    //R LSB of (Vector Sum) Total Apparent Power

//Fundamental / Harmonic Energy Register
#define APenergyTF 0x00A0          //R/C Total Forward Active Fundamental EnergyRefer to Table-10.P 67
#define APenergyAF APenergyTF+1    //R/C Phase A Forward Active FundamentalEnergy
#define APenergyBF APenergyTF+2    //R/C Phase B Forward Active FundamentalEnergy
#define APenergyCF APenergyTF+3    //R/C Phase C Forward Active FundamentalEnergy
#define ANenergyTF APenergyTF+4    //R/C Total Reverse Active Fundamental Energy
#define ANenergyAF APenergyTF+5    //R/C Phase A Reverse Active FundamentalEnergy
#define ANenergyBF APenergyTF+6    //R/C Phase B Reverse Active FundamentalEnergy
#define ANenergyCF APenergyTF+7    //R/C Phase C Reverse Active FundamentalEnergy
#define APenergyTH APenergyTF+8    //R/C Total Forward Active Harmonic Energy
#define APenergyAH APenergyTF+9    //R/C Phase A Forward Active Harmonic Energy
#define APenergyBH APenergyTF+10   //R/C Phase B Forward Active Harmonic Energy
#define APenergyCH APenergyTF+11   //R/C Phase C Forward Active Harmonic Energy
#define ANenergyTH APenergyTF+12   //R/C Total Reverse Active Harmonic Energy
#define ANenergyAH APenergyTF+13   //R/C Phase A Reverse Active Harmonic Energy
#define ANenergyBH APenergyTF+14   //R/C Phase B Reverse Active Harmonic Energy
#define ANenergyCH APenergyTF+15   //R/C Phase C Reverse Active Harmonic Energy

//Power and Power Factor Registers
#define PmeanT 0x00B0              //R Total (all-phase-sum) Active Power Refer to Table-11. P 68
#define PmeanA PmeanT+1            //R Phase A Active Power
#define PmeanB PmeanT+2            //R Phase B Active Power
#define PmeanC PmeanT+3            //R Phase C Active Power
#define QmeanT PmeanT+4            //R Total (all-phase-sum) Reactive Power
#define QmeanA PmeanT+5            //R Phase A Reactive Power
#define QmeanB PmeanT+6            //R Phase B Reactive Power
#define QmeanC PmeanT+7            //R Phase C Reactive Power
#define SAmeanT PmeanT+8           //R Total (Arithmetic Sum) apparent power
#define SmeanA PmeanT+9            //R phase A apparent power
#define SmeanB PmeanT+10           //R phase B apparent power
#define SmeanC PmeanT+11           //R phase C apparent power
#define PFmeanT PmeanT+12          //R Total power factor
#define PFmeanA PmeanT+13          //R phase A power factor
#define PFmeanB PmeanT+14          //R phase B power factor
#define PFmeanC PmeanT+15          //R phase C power factor
#define PmeanTLSB PmeanT+16        //R Lower word of Total (all-phase-sum) Active Power
#define PmeanALSB PmeanT+17        //R Lower word of Phase A Active Power
#define PmeanBLSB PmeanT+18        //R Lower word of Phase B Active Power
#define PmeanCLSB PmeanT+19        //R Lower word of Phase C Active Power
#define QmeanTLSB PmeanT+20        //R Lower word of Total (all-phase-sum) Reactive Power
#define QmeanALSB PmeanT+21        //R Lower word of Phase A Reactive Power
#define QmeanBLSB PmeanT+22        //R Lower word of Phase B Reactive Power
#define QmeanCLSB PmeanT+23        //R Lower word of Phase C Reactive Power
#define SAmeanTLSB PmeanT+24       //R Lower word of Total (Arithmetic Sum) apparent power
#define SmeanALSB PmeanT+25        //R Lower word of phase A apparent power
#define SmeanBLSB PmeanT+26        //R Lower word of phase B apparent power
#define SmeanCLSB PmeanT+27        //R Lower word of phase C apparent power

//Fundamental / Harmonic Power and Voltage / Current RMS Registers
#define PmeanTF 0x00D0             //R Total active fundamental powerRefer to Table-12. P 69
#define PmeanAF PmeanTF+1          //R phase A active fundamental power
#define PmeanBF PmeanTF+2          //R phase B active fundamental power
#define PmeanCF PmeanTF+3          //R phase C active fundamental power
#define PmeanTH PmeanTF+4          //R Total active harmonic power
#define PmeanAH PmeanTF+5          //R phase A active harmonic power
#define PmeanBH PmeanTF+6          //R phase B active harmonic power
#define PmeanCH PmeanTF+7          //R phase C active harmonic power
#define IrmsN1 PmeanTF+8           //R N Line Sampled current RMS
#define UrmsA PmeanTF+9            //R phase A voltage RMS
#define UrmsB PmeanTF+10           //R phase B voltage RMS
#define UrmsC PmeanTF+11           //R phase C voltage RMS
#define IrmsN0 PmeanTF+12          //R N Line calculated current RMS
#define IrmsA PmeanTF+13           //R phase A current RMS
#define IrmsB PmeanTF+14           //R phase B current RMS
#define IrmsC PmeanTF+15           //R phase C current RMS
#define PmeanTFLSB PmeanTF+16      //R Lower word of Total active fundamental Power
#define PmeanAFLSB PmeanTF+17      //R Lower word of phase A active fundamental Power
#define PmeanBFLSB PmeanTF+18      //R Lower word of phase B active fundamental Power
#define PmeanCFLSB PmeanTF+19      //R Lower word of phase C active fundamental Power
#define PmeanTHLSB PmeanTF+20      //R Lower word of Total active harmonic Power
#define PmeanAHLSB PmeanTF+21      //R Lower word of phase A active harmonic Power
#define PmeanBHLSB PmeanTF+22      //R Lower word of phase B active harmonic Power
#define PmeanCHLSB PmeanTF+23      //R Lower word of phase C active harmonic Power
#define UrmsALSB PmeanTF+25        //R Lower word of phase A voltage RMS
#define UrmsBLSB PmeanTF+26        //R Lower word of phase B voltage RMS
#define UrmsCLSB PmeanTF+27        //R Lower word of phase C voltage RMS
#define IrmsALSB PmeanTF+29        //R Lower word of phase A current RMS
#define IrmsBLSB PmeanTF+30        //R Lower word of phase B current RMS
#define IrmsCLSB PmeanTF+31        //R Lower word of phase C current RMS

//THD+N, Frequency, Angle and Temperature Registers
#define THDNUA 0x00F1              //R phase A voltage THD+N Refer to Table-13. P 70
#define THDNUB THDNUA+1            //R phase B voltage THD+N
#define THDNUC THDNUA+2            //R phase C voltage THD+N
#define THDNIA THDNUA+4            //R phase A current THD+N
#define THDNIB THDNUA+5            //R phase B current THD+N
#define THDNIC THDNUA+6            //R phase C current THD+N
#define M90_Freq THDNUA+7    //R Frequency
#define PAngleA THDNUA+8           //R phase A mean phase angle
#define PAngleB THDNUA+9           //R phase B mean phase angle
#define PAngleC THDNUA+10          //R phase C mean phase angle
#define M90_Temp THDNUA+11         //R Measured temperature
#define UangleA THDNUA+12          //R phase A voltage phase angle
#define UangleB THDNUA+13          //R phase B voltage phase angle
#define UangleC THDNUA+14          //R phase C voltage phase angle

//Harmonic Fourier Analysis Registers
#define AI_HR2 0x0100              //R phase A, Current, Harmonic Ratio for 2-th order component - (Harmonic Ratio (%) = Register Value / 163.84 <-- Válido para cada armónico y armónico total)
#define AI_HR3 AI_HR2+1            //R phase A, Current, Harmonic Ratio for 3-th order component
#define AI_HR4 AI_HR2+2            //R phase A, Current, Harmonic Ratio for 4-th order component
#define AI_HR5 AI_HR2+3            //R phase A, Current, Harmonic Ratio for 5-th order component
#define AI_HR6 AI_HR2+4            //R phase A, Current, Harmonic Ratio for 6-th order component
#define AI_HR7 AI_HR2+5            //R phase A, Current, Harmonic Ratio for 7-th order component
#define AI_HR8 AI_HR2+6            //R phase A, Current, Harmonic Ratio for 8-th order component
#define AI_HR9 AI_HR2+7            //R phase A, Current, Harmonic Ratio for 9-th order component
#define AI_HR10 AI_HR2+8           //R phase A, Current, Harmonic Ratio for 10-th order component
#define AI_HR11 AI_HR2+9           //R phase A, Current, Harmonic Ratio for 11-th order component
#define AI_HR12 AI_HR2+10          //R phase A, Current, Harmonic Ratio for 12-th order component
#define AI_HR13 AI_HR2+11          //R phase A, Current, Harmonic Ratio for 13-th order component
#define AI_HR14 AI_HR2+12          //R phase A, Current, Harmonic Ratio for 14-th order component
#define AI_HR15 AI_HR2+13          //R phase A, Current, Harmonic Ratio for 15-th order component
#define AI_HR16 AI_HR2+14          //R phase A, Current, Harmonic Ratio for 16-th order component
#define AI_HR17 AI_HR2+15          //R phase A, Current, Harmonic Ratio for 17-th order component
#define AI_HR18 AI_HR2+16          //R phase A, Current, Harmonic Ratio for 18-th order component
#define AI_HR19 AI_HR2+17          //R phase A, Current, Harmonic Ratio for 19-th order component
#define AI_HR20 AI_HR2+18          //R phase A, Current, Harmonic Ratio for 20-th order component
#define AI_HR21 AI_HR2+19          //R phase A, Current, Harmonic Ratio for 21-th order component
#define AI_THD AI_HR2+31           //R phase A, Current, Total Harmonic Distortion Ratio

#define BI_HR2 0x0120              //R phase B, Current, Harmonic Ratio for 2-th order component
#define BI_HR3 BI_HR2+1            //R phase B, Current, Harmonic Ratio for 3-th order component
#define BI_HR4 BI_HR2+2            //R phase B, Current, Harmonic Ratio for 4-th order component
#define BI_HR5 BI_HR2+3            //R phase B, Current, Harmonic Ratio for 5-th order component
#define BI_HR6 BI_HR2+4            //R phase B, Current, Harmonic Ratio for 6-th order component
#define BI_HR7 BI_HR2+5            //R phase B, Current, Harmonic Ratio for 7-th order component
#define BI_HR8 BI_HR2+6            //R phase B, Current, Harmonic Ratio for 8-th order component
#define BI_HR9 BI_HR2+7            //R phase B, Current, Harmonic Ratio for 9-th order component
#define BI_HR10 BI_HR2+8           //R phase B, Current, Harmonic Ratio for 10-th order component
#define BI_HR11 BI_HR2+9           //R phase B, Current, Harmonic Ratio for 11-th order component
#define BI_HR12 BI_HR2+10          //R phase B, Current, Harmonic Ratio for 12-th order component
#define BI_HR13 BI_HR2+11          //R phase B, Current, Harmonic Ratio for 13-th order component
#define BI_HR14 BI_HR2+12          //R phase B, Current, Harmonic Ratio for 14-th order component
#define BI_HR15 BI_HR2+13          //R phase B, Current, Harmonic Ratio for 15-th order component
#define BI_HR16 BI_HR2+14          //R phase B, Current, Harmonic Ratio for 16-th order component
#define BI_HR17 BI_HR2+15          //R phase B, Current, Harmonic Ratio for 17-th order component
#define BI_HR18 BI_HR2+16          //R phase B, Current, Harmonic Ratio for 18-th order component
#define BI_HR19 BI_HR2+17          //R phase B, Current, Harmonic Ratio for 19-th order component
#define BI_HR20 BI_HR2+18          //R phase B, Current, Harmonic Ratio for 20-th order component
#define BI_HR21 BI_HR2+19          //R phase B, Current, Harmonic Ratio for 21-th order component
#define BI_THD BI_HR2+31           //R phase B, Current, Total Harmonic Distortion Ratio

#define CI_HR2 0x0140              //R phase C, Current, Harmonic Ratio for 2-th order component
#define CI_HR3 CI_HR2+1            //R phase C, Current, Harmonic Ratio for 3-th order component
#define CI_HR4 CI_HR2+2            //R phase C, Current, Harmonic Ratio for 4-th order component
#define CI_HR5 CI_HR2+3            //R phase C, Current, Harmonic Ratio for 5-th order component
#define CI_HR6 CI_HR2+4            //R phase C, Current, Harmonic Ratio for 6-th order component
#define CI_HR7 CI_HR2+5            //R phase C, Current, Harmonic Ratio for 7-th order component
#define CI_HR8 CI_HR2+6            //R phase C, Current, Harmonic Ratio for 8-th order component
#define CI_HR9 CI_HR2+7            //R phase C, Current, Harmonic Ratio for 9-th order component
#define CI_HR10 CI_HR2+8           //R phase C, Current, Harmonic Ratio for 10-th order component
#define CI_HR11 CI_HR2+9           //R phase C, Current, Harmonic Ratio for 11-th order component
#define CI_HR12 CI_HR2+10          //R phase C, Current, Harmonic Ratio for 12-th order component
#define CI_HR13 CI_HR2+11          //R phase C, Current, Harmonic Ratio for 13-th order component
#define CI_HR14 CI_HR2+12          //R phase C, Current, Harmonic Ratio for 14-th order component
#define CI_HR15 CI_HR2+13          //R phase C, Current, Harmonic Ratio for 15-th order component
#define CI_HR16 CI_HR2+14          //R phase C, Current, Harmonic Ratio for 16-th order component
#define CI_HR17 CI_HR2+15          //R phase C, Current, Harmonic Ratio for 17-th order component
#define CI_HR18 CI_HR2+16          //R phase C, Current, Harmonic Ratio for 18-th order component
#define CI_HR19 CI_HR2+17          //R phase C, Current, Harmonic Ratio for 19-th order component
#define CI_HR20 CI_HR2+18          //R phase C, Current, Harmonic Ratio for 20-th order component
#define CI_HR21 CI_HR2+19          //R phase C, Current, Harmonic Ratio for 21-th order component
#define CI_THD CI_HR2+31           //R phase C, Current, Total Harmonic Distortion Ratio

#define AV_HR2 0x0160              //R phase A, Voltage, Harmonic Ratio for 2-th order component
#define AV_HR3 AV_HR2+1            //R phase A, Voltage, Harmonic Ratio for 3-th order component
#define AV_HR4 AV_HR2+2            //R phase A, Voltage, Harmonic Ratio for 4-th order component
#define AV_HR5 AV_HR2+3            //R phase A, Voltage, Harmonic Ratio for 5-th order component
#define AV_HR6 AV_HR2+4            //R phase A, Voltage, Harmonic Ratio for 6-th order component
#define AV_HR7 AV_HR2+5            //R phase A, Voltage, Harmonic Ratio for 7-th order component
#define AV_HR8 AV_HR2+6            //R phase A, Voltage, Harmonic Ratio for 8-th order component
#define AV_HR9 AV_HR2+7            //R phase A, Voltage, Harmonic Ratio for 9-th order component
#define AV_HR10 AV_HR2+8           //R phase A, Voltage, Harmonic Ratio for 10-th order component
#define AV_HR11 AV_HR2+9           //R phase A, Voltage, Harmonic Ratio for 11-th order component
#define AV_HR12 AV_HR2+10          //R phase A, Voltage, Harmonic Ratio for 12-th order component
#define AV_HR13 AV_HR2+11          //R phase A, Voltage, Harmonic Ratio for 13-th order component
#define AV_HR14 AV_HR2+12          //R phase A, Voltage, Harmonic Ratio for 14-th order component
#define AV_HR15 AV_HR2+13          //R phase A, Voltage, Harmonic Ratio for 15-th order component
#define AV_HR16 AV_HR2+14          //R phase A, Voltage, Harmonic Ratio for 16-th order component
#define AV_HR17 AV_HR2+15          //R phase A, Voltage, Harmonic Ratio for 17-th order component
#define AV_HR18 AV_HR2+16          //R phase A, Voltage, Harmonic Ratio for 18-th order component
#define AV_HR19 AV_HR2+17          //R phase A, Voltage, Harmonic Ratio for 19-th order component
#define AV_HR20 AV_HR2+18          //R phase A, Voltage, Harmonic Ratio for 20-th order component
#define AV_HR21 AV_HR2+19          //R phase A, Voltage, Harmonic Ratio for 21-th order component
#define AV_THD AV_HR2+31           //R phase A, Voltage, Total Harmonic Distortion Ratio

#define BV_HR2 0x0180              //R phase B, Voltage, Harmonic Ratio for 2-th order component
#define BV_HR3 BV_HR2+1            //R phase B, Voltage, Harmonic Ratio for 3-th order component
#define BV_HR4 BV_HR2+2            //R phase B, Voltage, Harmonic Ratio for 4-th order component
#define BV_HR5 BV_HR2+3            //R phase B, Voltage, Harmonic Ratio for 5-th order component
#define BV_HR6 BV_HR2+4            //R phase B, Voltage, Harmonic Ratio for 6-th order component
#define BV_HR7 BV_HR2+5            //R phase B, Voltage, Harmonic Ratio for 7-th order component
#define BV_HR8 BV_HR2+6            //R phase B, Voltage, Harmonic Ratio for 8-th order component
#define BV_HR9 BV_HR2+7            //R phase B, Voltage, Harmonic Ratio for 9-th order component
#define BV_HR10 BV_HR2+8           //R phase B, Voltage, Harmonic Ratio for 10-th order component
#define BV_HR11 BV_HR2+9           //R phase B, Voltage, Harmonic Ratio for 11-th order component
#define BV_HR12 BV_HR2+10          //R phase B, Voltage, Harmonic Ratio for 12-th order component
#define BV_HR13 BV_HR2+11          //R phase B, Voltage, Harmonic Ratio for 13-th order component
#define BV_HR14 BV_HR2+12          //R phase B, Voltage, Harmonic Ratio for 14-th order component
#define BV_HR15 BV_HR2+13          //R phase B, Voltage, Harmonic Ratio for 15-th order component
#define BV_HR16 BV_HR2+14          //R phase B, Voltage, Harmonic Ratio for 16-th order component
#define BV_HR17 BV_HR2+15          //R phase B, Voltage, Harmonic Ratio for 17-th order component
#define BV_HR18 BV_HR2+16          //R phase B, Voltage, Harmonic Ratio for 18-th order component
#define BV_HR19 BV_HR2+17          //R phase B, Voltage, Harmonic Ratio for 19-th order component
#define BV_HR20 BV_HR2+18          //R phase B, Voltage, Harmonic Ratio for 20-th order component
#define BV_HR21 BV_HR2+19          //R phase B, Voltage, Harmonic Ratio for 21-th order component
#define BV_THD BV_HR2+31           //R phase B, Voltage, Total Harmonic Distortion Ratio

#define CV_HR2 0x01A0              //R phase C, Voltage, Harmonic Ratio for 2-th order component
#define CV_HR3 CV_HR2+1            //R phase C, Voltage, Harmonic Ratio for 3-th order component
#define CV_HR4 CV_HR2+2            //R phase C, Voltage, Harmonic Ratio for 4-th order component
#define CV_HR5 CV_HR2+3            //R phase C, Voltage, Harmonic Ratio for 5-th order component
#define CV_HR6 CV_HR2+4            //R phase C, Voltage, Harmonic Ratio for 6-th order component
#define CV_HR7 CV_HR2+5            //R phase C, Voltage, Harmonic Ratio for 7-th order component
#define CV_HR8 CV_HR2+6            //R phase C, Voltage, Harmonic Ratio for 8-th order component
#define CV_HR9 CV_HR2+7            //R phase C, Voltage, Harmonic Ratio for 9-th order component
#define CV_HR10 CV_HR2+8           //R phase C, Voltage, Harmonic Ratio for 10-th order component
#define CV_HR11 CV_HR2+9           //R phase C, Voltage, Harmonic Ratio for 11-th order component
#define CV_HR12 CV_HR2+10          //R phase C, Voltage, Harmonic Ratio for 12-th order component
#define CV_HR13 CV_HR2+11          //R phase C, Voltage, Harmonic Ratio for 13-th order component
#define CV_HR14 CV_HR2+12          //R phase C, Voltage, Harmonic Ratio for 14-th order component
#define CV_HR15 CV_HR2+13          //R phase C, Voltage, Harmonic Ratio for 15-th order component
#define CV_HR16 CV_HR2+14          //R phase C, Voltage, Harmonic Ratio for 16-th order component
#define CV_HR17 CV_HR2+15          //R phase C, Voltage, Harmonic Ratio for 17-th order component
#define CV_HR18 CV_HR2+16          //R phase C, Voltage, Harmonic Ratio for 18-th order component
#define CV_HR19 CV_HR2+17          //R phase C, Voltage, Harmonic Ratio for 19-th order component
#define CV_HR20 CV_HR2+18          //R phase C, Voltage, Harmonic Ratio for 20-th order component
#define CV_HR21 CV_HR2+19          //R phase C, Voltage, Harmonic Ratio for 21-th order component
#define CV_THD CV_HR2+31           //R phase C, Voltage, Total Harmonic Distortion Ratio

#define AI_FUND 0x01C0             //R phase A, Current, Fundamental component value 
                                   //Current, Fundamental component value = Register Value * 3.2656*10-3 / 2^scale, Register (1C0H, 1C2H, 1C4H)
                                   //Voltage, Fundamental component value = Register Value * 3.2656*10-2 / 2^scale, Register (1C1H, 1C3H, 1C5H).
                                   //The scale is defined by the DFT_SCALE(1D0H) register.
#define AV_FUND AI_FUND+1          //R phase A, Voltage, Fundamental component value
#define BI_FUND AI_FUND+2          //R phase B, Current, Fundamental component value
#define BV_FUND AI_FUND+3          //R phase B, Voltage, Fundamental component value
#define CI_FUND AI_FUND+4          //R phase C, Current, Fundamental component value
#define CV_FUND AI_FUND+5          //R phase C, Voltage, Fundamental component value

#define DFT_SCALE 0x01D0           //RWInput Gain = 2^Scale, i.e. Scale = # of bit shifts
                                   //[2:0]: Scale for Channel A-I.
                                   //[5:3]: Scale for Channel B-I.
                                   //[8:6]: Scale for Channel C-I.
                                   //[10:9]: Scale for Channel A-V.
                                   //[12:11]: Scale for Channel B-V.
                                   //[14:13]: Scale for Channel C-V.
                                   //[15]: Window disable. ‘1’ disable the Hanning window.
#define DFT_CTRL 0x01D1            //RW Bit[0]: DFT_START. This bit is automatically cleared after DFT finishes.
                                   //0: Reset and abort the DFT computation.
                                   //1: Start the DFT. 

//Registros relacionados con la temperatura
#define Temp_cfg1 0x02FD     //Registro 1 de configuración del sensor de temperatura
#define Temp_cfg2 0x0216     //Registro 2 de configuración del sensor de temperatura
#define Temp_cfg3 0x0219     //Registro 3 de configuración del sensor de temperatura
#define Temp_comp1 0x02FC    //Registro 1 de compensación de temperatura 
#define Temp_comp2 0x0270    //Registro 2 de compensación de temperatura 
#define Temp_comp3 0x027B    //Registro 3 de compensación de temperatura 

//--------------------------------------------------------------------------------------------------
//-------------------------------------------------------------
//Seteo del bus SPI para manejo del chip M90E36A
SPISettings Spi_M90_param(500000, MSBFIRST, SPI_MODE0); 

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
uint16_t Set_FuncEn0 = 0x0000;
uint16_t Set_FuncEn1 = 0x0000;
uint16_t Set_ZXConfig = 0x0001;
uint16_t Set_SagTh = 0x0000;
uint16_t Set_PhaseLossTh = 0x0000;
uint16_t Set_INWarnTh0 = 0xFFFF;
uint16_t Set_INWarnTh1 = 0xFFFF;
uint16_t Set_THDNUTh = 0xFFFF;
uint16_t Set_THDNITh = 0xFFFF;
uint16_t Set_DMACtrl = 0xFE44;
//------
//Variables de seteo de los registros del ATM90E36A
uint16_t Set_PLconstH = 0x0861; //PLconst = 0.01CF o PLconst = 45000000000/MC
uint16_t Set_PLconstL = 0x4C68; //MC = Meter constant Ej: 3200 imp/kWh
uint16_t Set_MMode0 = 0x0487;   //3P4W, 50Hz, Rogowsky
uint16_t Set_MMode1 = 0x0000;   //Ganacias digitales y de ADC en (x1)
uint16_t Set_PStartTh = 0x0000;
uint16_t Set_QStartTh = 0x0000;
uint16_t Set_SStartTh = 0x0000;
uint16_t Set_PPhaseTh = 0x0000;
uint16_t Set_QPhaseTh = 0x0000;
uint16_t Set_SPhaseTh = 0x0000;
//------
//Variables de seteo de los registros de offset y ganancia de energia y potencia por canal
int16_t Set_PoffsetA = 0x0000;
int16_t Set_QoffsetA = 0x0000;
int16_t Set_PoffsetB = 0x0000;
int16_t Set_QoffsetB = 0x0000;
int16_t Set_PoffsetC = 0x0000;
int16_t Set_QoffsetC = 0x0000;
int16_t Set_PQGainA = 0xFFDD;
int16_t Set_PhiA = 0x0000;
int16_t Set_PQGainB = 0xFFDD;
int16_t Set_PhiB = 0x0000;
int16_t Set_PQGainC = 0xFFD1;
int16_t Set_PhiC = 0x0000;
//------
//Variables de seteo de los registros de offset y ganancia de potencia fundamental por canal
int16_t Set_PoffsetAF = 0x0000;
int16_t Set_PoffsetBF = 0x0000;
int16_t Set_PoffsetCF = 0x0000;
int16_t Set_PGainAF = 0x0000;
int16_t Set_PGainBF = 0x0000;
int16_t Set_PGainCF = 0x0000;
//------
//Variables de seteo de los registros de offset y ganancia de tensión y corriente por canal
uint16_t Set_UgainA = 0xCE40;
uint16_t Set_IgainA = 0xCE40; 
uint16_t Set_UoffsetA = 0x0000;
uint16_t Set_IoffsetA = 0x0000;
uint16_t Set_UgainB = 0xCE40;
uint16_t Set_IgainB = 0xCE40;
uint16_t Set_UoffsetB = 0x0000;
uint16_t Set_IoffsetB = 0x0000;
uint16_t Set_UgainC = 0xCE40;
uint16_t Set_IgainC = 0xCE40;
uint16_t Set_UoffsetC = 0x0000;
uint16_t Set_IoffsetC = 0x0000;
uint16_t Set_IgainN = 0xCE40;
uint16_t Set_IoffsetN = 0x0000;
//--------------------------------------------------------------------------------------------------
//Definición de los lugares en E2 para guardar los registros del M90E36A o sus equivalentes en RAM
const unsigned long e2_SetM90 = 130944;  //Dirección de comienzo de los últimos 128 bytes de la E2 
           //para registros M90E36A (130944<->131071)
const unsigned long e2_FuncEn0 = e2_SetM90;
const unsigned long e2_FuncEn1 = e2_FuncEn0 + 2;
const unsigned long e2_ZXConfig = e2_FuncEn0 + 4;
const unsigned long e2_SagTh = e2_FuncEn0 + 6;
const unsigned long e2_PhaseLossTh = e2_FuncEn0 + 8;
const unsigned long e2_INWarnTh0 = e2_FuncEn0 + 10;
const unsigned long e2_INWarnTh1 = e2_FuncEn0 + 12;
const unsigned long e2_THDNUTh = e2_FuncEn0 + 14;
const unsigned long e2_THDNITh = e2_FuncEn0 + 16;
const unsigned long e2_DMACtrl = e2_FuncEn0 + 18;
//------
const unsigned long e2_PLconstH = e2_FuncEn0 + 20;
const unsigned long e2_PLconstL = e2_FuncEn0 + 22;
const unsigned long e2_MMode0 = e2_FuncEn0 + 24;
const unsigned long e2_MMode1 = e2_FuncEn0 + 26;
const unsigned long e2_PStartTh = e2_FuncEn0 + 28;
const unsigned long e2_QStartTh = e2_FuncEn0 + 30;
const unsigned long e2_SStartTh = e2_FuncEn0 + 32;
const unsigned long e2_PPhaseTh = e2_FuncEn0 + 34;
const unsigned long e2_QPhaseTh = e2_FuncEn0 + 36;
const unsigned long e2_SPhaseTh = e2_FuncEn0 + 38;
//------
const unsigned long e2_PoffsetA = e2_FuncEn0 + 40;
const unsigned long e2_QoffsetA = e2_FuncEn0 + 42;
const unsigned long e2_PoffsetB = e2_FuncEn0 + 44;
const unsigned long e2_QoffsetB = e2_FuncEn0 + 46;
const unsigned long e2_PoffsetC = e2_FuncEn0 + 48;
const unsigned long e2_QoffsetC = e2_FuncEn0 + 50;
const unsigned long e2_PQGainA = e2_FuncEn0 + 52;
const unsigned long e2_PhiA = e2_FuncEn0 + 54;
const unsigned long e2_PQGainB = e2_FuncEn0 + 56;
const unsigned long e2_PhiB = e2_FuncEn0 + 58;
const unsigned long e2_PQGainC = e2_FuncEn0 + 60;
const unsigned long e2_PhiC = e2_FuncEn0 + 62;
//------
const unsigned long e2_PoffsetAF = e2_FuncEn0 + 64;
const unsigned long e2_PoffsetBF = e2_FuncEn0 + 66;
const unsigned long e2_PoffsetCF = e2_FuncEn0 + 68;
const unsigned long e2_PGainAF = e2_FuncEn0 + 70;
const unsigned long e2_PGainBF = e2_FuncEn0 + 72;
const unsigned long e2_PGainCF = e2_FuncEn0 + 74;
//------
const unsigned long e2_UgainA = e2_FuncEn0 + 76;
const unsigned long e2_IgainA = e2_FuncEn0 + 78;
const unsigned long e2_UoffsetA = e2_FuncEn0 + 80;
const unsigned long e2_IoffsetA = e2_FuncEn0 + 82;
const unsigned long e2_UgainB = e2_FuncEn0 + 84;
const unsigned long e2_IgainB = e2_FuncEn0 + 86;
const unsigned long e2_UoffsetB = e2_FuncEn0 + 88;
const unsigned long e2_IoffsetB = e2_FuncEn0 + 90;
const unsigned long e2_UgainC = e2_FuncEn0 + 92;
const unsigned long e2_IgainC = e2_FuncEn0 + 94;
const unsigned long e2_UoffsetC = e2_FuncEn0 + 96;
const unsigned long e2_IoffsetC = e2_FuncEn0 + 98;
const unsigned long e2_IgainN = e2_FuncEn0 + 100;
const unsigned long e2_IoffsetN = e2_FuncEn0 + 102;
//------
const unsigned long e2_M90_Config_OK = e2_FuncEn0 + 104;  //Un byte --> Calibrado = 0x01
const unsigned long e2_M90_Adj_OK = e2_FuncEn0 + 105;   //Un byte --> Calibrado = 0x01
const unsigned long e2_M90_Cal_OK = e2_FuncEn0 + 106;   //Un byte --> Calibrado = 0x01
const unsigned long e2_M90_Harm_OK = e2_FuncEn0 + 107;    //Un byte --> Calibrado = 0x01
const unsigned long e2_M90_registrando = e2_FuncEn0 + 108;  //Un byte --> Registrando = 0x01
const unsigned long e2_M90_regHR = e2_FuncEn0 + 109;    //Un byte --> Registrando = 0x01
const unsigned long e2_M90_MaxCompArm = e2_FuncEn0 + 110; //Un byte --> 2<=MaxCompArm<=32
const unsigned long e2_M90_DftScale = e2_FuncEn0 + 111;   //Dos bytes --> DFT_SCALE
const unsigned long e2_M90_regTodo = e2_FuncEn0 + 113;    //Un byte --> Registrando todo = 0x01 
//const unsigned long e2_M90_Libre = e2_FuncEn0 + 114;    //Libre 
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//Definiciones para manejo de registro en RAM de variables del M90E36A
const unsigned int BuffMedicionesMax = 15;          //Largo BUFFER medicones       
String msg[BuffMedicionesMax] = "";                 //Arreglo de Strings para mediciones
unsigned int numeroMedWrite = 0;                    //Puntero de guardado en RAM de mediciones.
unsigned int numeroMedSD = 0;                       //Puntero de guardado en SD de mediciones.
volatile bool regM90_flag = false;                  //Indica el momento de medir
byte registrandoM90_flag;                           //Indica si esta registrando
bool buff_Lleno = false;                            //Indica buffer en RAM lleno => pasar datos a SD
volatile bool check_M90 = false;                    //Indica si tiene que chequear el estado del M90 periódicamente
volatile byte DMA_flag = 0x00;                      //Indica si esta en modo DMA => Flag en true 
//volatile bool DMA_flag = false;                     //Indica si esta en modo DMA => Flag en true 
uint8_t regHR_flag = 0;               //Indicador de registro de armónicos en curso
uint8_t regTodo_flag = 0;               //Indicador de registro de variables y armónicos en curso
uint8_t MaxCompArm = 10;          //Máximo componente armónico a analizar. Default = 10

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
