//
// FAIMSFB
//
// This program controls the FAIMSFB (FAIMS Fly Back transformer design) module for the MIPS system.
// This module uses the ATSAMD21G18A-AUT CPU. The serial EPROM chip used to hold configuration information
// on all MIPS modules is emulated by this CPU. The FAIMSFB module communicates throung the TWI interface,
// the TWI commands are shown in the FAIMSFB.H file. This header file also defines the data structure that
// defines the SEPROM contents, this data structure is duplicated in the MIPS system. The SEPROM emulation
// saves the SEPROM contents to flash so when the FAIMSFB code is updated on this module the SEPROM data
// is lost and set to default values, this means you will lose calibration data. 
//
// In addition to the TWI interface this module also support serial command processing through the USB 
// virtual serial interface. The MIPS system allows you to communcate with this serial interface using
// the TWITALK command. If the base address is set at 0x50 and board select is A then the command
// TWITALK,0,112 will allow you to use this serial interface through the MIPS system.
//
// This implementation requires the use of multiple TWI addresses, the low level TWI drivers did not fully 
// support this mode so the function onAddressMatch was added to the TWI drivers to signal a TWI address change.
// The code emulates the SEPROM and uses the base address for the first 256 byte page and the base address + 1 
// for the second page. Base address + 0x20 for the general command to the FAIMSFB module.
// For example if base address is 0x50, then 0x51 is page 2 and 0x70 is general commands.
//
// The M0 processor uses the Arduino Adafruit Feather M0 bootloader and board configuration, the Arduino development
// IDL is used and the Arduino Feather M0 board type is use.
//
// With version 1.2 and later this code supports uploading new firmware using the FAIMSFB firmware. The new version
// is loaded in the upper half of the FLASH at 0x20000. If a jumper is installed on JP6 pins 4 to 5 the system 
// will look for firmware loaded at 0x20000, if found it will start this firmware. This firmware loaded in the 
// upper FLASH must be built using the Feather M0, upper half board configuration.
//
// The FAIMSFB firmware controls the low level functions on the FAIMSFB module. Three timers are used as defined
// below:
// TCC0 - This timer drives the flyback transformer and generates the FAIMS waveform. The user can contol
//        the frequency and duty cycle to define and tune the waveform.
// TCC2 - This timer set the power level by varying the duty cycle to the power drive FET.
// TC5  - This timer is used to support scanning functions
//
// Software development to do list:
// 1.) Implement the enviormental commands
//
//  V1.0, Nov 25, 2018
//    1.) First release
//  V1.1, Sept 14, 2019
//    1.) Added additional tests to make sure drive level is within range
//  V1.2, March 11, 2020
//    1.) Added support foor uploading new firmware to the upper half of the FLASH
//
// Gordon Anderson
//
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_BME280.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "FAIMSFB.h"
#include "Errors.h"
#include "Serial.h"
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

int8_t       TWIadd = 0x50;
const char   Version[] PROGMEM = "FAIMSFB version 1.2, March 11, 2020";
FAIMSFBdata  faimsfb;
FAIMSFBstate sdata;
int          Eaddress = 0;
int          recAdd;
uint8_t      Ebuf[512];
FAIMSFBdata  *fptr = (FAIMSFBdata *)Ebuf;
bool         GenerateCalibrationTable = false;
bool         ReturnAvalible = false;
bool         InitMonValue = false;
bool         SetVrfFlag = false;
bool         SetVrfUseTable = false;

SerialBuffer sb;

// Monitored values
float        DCB1mon = -1;         // DCB channel 1 monitor
float        DCB2mon = -1;         // DCB channel 2 monitor
// BIASmon = DCB1mon + DCB2mon
// CVmon =   (DCB1mon - DCB2mon) - BIASmon
float        Power;
ReadBacks    rb = {-1,-1,-1,-1,-1};

Adafruit_BME280 bme(BME280_CS);       // hardware SPI

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is lost every time
// the sketch is uploaded on the board.
FlashStorage(flash_FAIMSFBdata, FAIMSFBdata);

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

// Scan control parameters
int       CurrentScanStep;
float     InitialVrf;
float     InitialCV;
float     InitialDrive;
ScanPoint sp;
Stream    *ReportStream;
bool      ASCIIformat;
bool      Scanning = false;
bool      ScanReport = true;

FAIMSFBdata Rev_1_faimsfb = {
                            sizeof(FAIMSFBdata),"FAIMSfb",1,
                            false,0x52,1500000,50,0,0,false,
                            0.0,0.0,
                            20,100,1,
                            // Arc detector sensitivity
                            0.5,
                            // AD5592 ADC channels
                            4,1177.79,31323.65,
                            5,1178.96,31317.82,
                            6,2457.15,0,
                            7,2,0,
                            // AD5592 DAC channels
                            0,1180.97,31349.84,      // DCB1
                            1,1174.87,31347.76,      // DCB2
                            2,26238.31,0,            // DCB ref
                            // Vrf monitor ADC channel on 0
                            0x80 | A0,8.41,-337.66,
                            // Scanning parameters
                            800,800,0,10,1.25,1,1,
                            // Step based scanning
                            10,100,false,A2,
                            // External scan trigger options
                            0,0,
                            {0,100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000},
                            // Electrometer parameters
                            false, 0x11,
                            0,131.07,0,               // ADC0, 0 to 500 pA electrometer pos channel
                            1,131.07,0,               // ADC1, 0 to 500 pA electrometer neg channel
                            2,13107,0,                // DAC2, zero, pos
                            3,13107,0,                // DAC3, zero, neg
                            4,13107,0,                // DAC4, offset, pos
                            5,13107,0,                // DAC5, offset, neg
                            0,0,0,0,
                            //
                            SIGNATURE
                            };

void msTimerIntercept(void);
extern void (*mySysTickHook)(void);
void (*mySysTickHook)(void) = msTimerIntercept;
void msTimerIntercept(void)
{

}

void AddressMatchEvent(void)
{
  recAdd = PERIPH_WIRE.readDataWIRE() >> 1;
}

// This function is called when the master asks for data.
// Send up to 32 bytes from the sb structure
void requestEventProcessor(void)
{
  int num = sb.available();

  if(ReturnAvalible)
  {
    ReturnAvalible = false;
    Wire.write(num & 0x0FF);
    Wire.write((num >> 8) & 0x0FF);
    return;    
  }
  for (int i = 0; i < num; i++)
  {
    if (i >= 30) break;
    Wire.write(sb.read());
  }  
}

// This function is called when the master asks for data.
// Send up to 32 bytes.
void requestEvent(void)
{
  Eaddress &= ~0x0100;    // Reset the page address bit
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd+1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= 0x0100;
  }
  else if(recAdd == (TWIadd | 0x20))
  {
    // True for general commands, process here
    requestEventProcessor();
    return;
  }
  // Always send 32 bytes of data from the SEPROM buffer, send will
  // terminate on NAK if less is wanted.
  // Use the image of the data stored in flash, this image is in the Ebuf
  for(int i=0;i<32;i++) if(Wire.write(Ebuf[Eaddress+i]) == 0) break;
}

// Reads a 16 bit value from the TWI interface, return -1 if two bytes
// were not avalibale
int ReadUnsignedWord(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  if (Wire.available() == 0) return -1;
  i |= Wire.read() << 8;
  return i & 0xFFFF;
}

bool ReadInt(int *i)
{
  if (Wire.available() == 0) return false;
  *i = Wire.read();
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 8;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 16;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 24;
  return true;
}

// Reads a 8 bit value from the TWI interface, return -1 if a byte
// was not avalibale
int ReadUnsignedByte(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  return i & 0xFF;
}

// Reads a 8 bit signed value from the TWI interface, return false if a byte
// was not avalibale or true if ok
bool ReadByte(int8_t *b)
{
  if (Wire.available() == 0) return false;
  *b = Wire.read();
  return true;
}

bool Read16bitInt(int16_t *shortint)
{
  uint8_t *b = (uint8_t *)shortint;

  if (Wire.available() == 0) return false;
  b[0] = Wire.read();
  if (Wire.available() == 0) return false;
  b[1] = Wire.read();
  return true;
}

// Reads a float value from the TWI interface, return false if float
// was not avalibale
bool ReadFloat(float *fval)
{
  int i;
  uint8_t *b;

  b = (uint8_t *)fval;
  for (int j = 0; j < 4; j++)
  {
    if (Wire.available() == 0) return false;
    b[j] = Wire.read();
  }
  return true;
}

void SendByte(byte bval)
{
  sb.write(bval);
}

void SendInt24(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  // Send the 24 bit word to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
}

void SendFloat(float fval)
{
  uint8_t *b;

  b = (uint8_t *)&fval;
  // Send the float to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
  sb.write(b[3]);
}

void receiveEventProcessor(int howMany)
{
  uint8_t cmd;
  int i, j, off, count, startI, stopI;
  int8_t b;
  int16_t shortint;
  float fval;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();
    if (serial == &sb)
    {
      if (cmd == ESC) serial = &Serial;
      else PutCh(cmd);
    }
    else switch (cmd)
    {
      case TWI_SERIAL:
        serial = &sb;
        break;
      case TWI_SET_ENABLE:
        i = ReadUnsignedByte();
        if (i != -1) faimsfb.Enable = i;
        break;
      case TWI_SET_MODE:
        i = ReadUnsignedByte();
        if (i != -1) faimsfb.Mode = i;
        break;
      case TWI_SET_SCNRPT:
        i = ReadUnsignedByte();
        if (i != -1) ScanReport = i;
        break;
      case TWI_SET_FREQ:
        if (!ReadInt(&i)) break;
        faimsfb.Freq = i;
        break;
      case TWI_SET_DUTY:
        if (!ReadByte(&b)) break;
        faimsfb.Duty = b;
        break;
      case TWI_SET_DRIVE:
        if (!ReadFloat(&fval)) break;
        faimsfb.Drive = fval;
        if(faimsfb.Drive > faimsfb.MaxDrive) faimsfb.Drive = faimsfb.MaxDrive;
        if(faimsfb.Drive < 0) faimsfb.Drive = 0;
        break;
      case TWI_SET_VRF:
        if (!ReadFloat(&fval)) break;
        faimsfb.Vrf = fval;
        break;
      case TWI_SET_VRF_NOW:
        if (!ReadFloat(&fval)) break;
        faimsfb.Vrf = fval;
        SetVrfFlag = true;
        break; 
      case TWI_SET_VRF_TABLE:
        if (!ReadFloat(&fval)) break;
        faimsfb.Vrf = fval;
        SetVrfUseTable = true;
        break;     
      case TWI_SET_MAXDRV:
        if (!ReadFloat(&fval)) break;
        faimsfb.MaxDrive = fval;
        break;
      case TWI_SET_MAXPWR:
        if (!ReadFloat(&fval)) break;
        faimsfb.MaxPower = fval;
        break;
      case TWI_SET_CV:
        if (!ReadFloat(&fval)) break;
        faimsfb.CV = fval;
        break;
      case TWI_SET_BIAS:
        if (!ReadFloat(&fval)) break;
        faimsfb.Bias = fval;
        break;
      case TWI_SET_CV_START:
        if (!ReadFloat(&fval)) break;
        faimsfb.CVstart = fval;
        break;
      case TWI_SET_CV_END:
        if (!ReadFloat(&fval)) break;
        faimsfb.CVend = fval;
        break;
      case TWI_SET_VRF_START:
        if (!ReadFloat(&fval)) break;
        faimsfb.VRFstart = fval;
        break;
      case TWI_SET_VRF_END:
        if (!ReadFloat(&fval)) break;
        faimsfb.VRFend = fval;
        break;
      case TWI_SET_DURATION:
        if (!ReadInt(&i)) break;
        faimsfb.StepDuration = i;
        break;
      case TWI_SET_STEPS:
        if (!ReadInt(&i)) break;
        faimsfb.Steps = i;
        break;
      case TWI_SET_EXTSTEP:
        i = ReadUnsignedByte();
        if (i != -1) faimsfb.EnableExtStep = i;
        break;
      case TWI_SET_STPPIN:
        i = ReadUnsignedByte();
        if (i != -1) faimsfb.ExtAdvInput = i;
        break;
      case TWI_SET_STEPSTR:
        InitScan(false);
        // Start a scan
        break;
      case TWI_SET_STEPSTP:
        StopScan();
        // Stop a scan
        break;
      case TWI_SET_FLUSH:
        sb.begin();
        break;
      case TWI_SET_CAL:
        GenerateCalibrationTable = true;
        break;
      case TWI_READ_AVALIBLE:
        // Set flag to return bytes avalible on the next read from TWI
        ReturnAvalible = true;
        break;
      case TWI_READ_READBACKS:
        // Copy the readback structure to the output buffer;
        uint8_t  *bptr;
        bptr = (uint8_t *)&rb;
        for(i=0;i<sizeof(ReadBacks);i++) sb.write(bptr[i]);
        break;
      case TWI_READ_DRIVE:
        SendFloat(faimsfb.Drive);
        break;
      case TWI_READ_VRF:
        SendFloat(faimsfb.Vrf);
        break;
      case TWI_READ_CV:
        SendFloat(faimsfb.CV);
        break;
      default:
        break;
    }
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  if(howMany == 0) return;
  Eaddress = 0;
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd + 1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= 0x0100;
  }
  else if(recAdd == (TWIadd | 0x20))
  {
    // True for general commands, process here
    receiveEventProcessor(howMany);
    return;
  }
  // Here to read data into the SEPROM emulation buffer
  // Read the number of bytes received into the buffer, first byte is the address within the selected page
  Eaddress |= Wire.read();
  if(howMany == 1) return;
  // Copy the working data structure to Ebuf for storage into FLASH
  for(int i=0;i<howMany-1;i++) Ebuf[(Eaddress + i) & 0x1FF] = Wire.read();
  // At this point we should update the FLASH assuming the write protection jumper is
  // installed.
  if((Eaddress + howMany) > 1)
  {
    if(digitalRead(TWIADDWRENA) == LOW)
    {
      // Write data to FLASH
      flash_FAIMSFBdata.write(*fptr);
    }
  }
}

// Initalizes the AD5592 as outlined below, all inputs and outputs are 0 to 2.5V,
// uses internal reference:
//  DAC channel 0, DCB1 output
//  DAC channel 1, DCB2 output
//  DAC channel 2, DCBref output
//
//  ADC channel 4, DCB1 monitor input
//  ADC channel 5, DCB2 monitor input
//  ADC channel 6, Voltage monitor input
//  ADC channel 7, Current monitor input
void FAIMSAD5592init(int8_t addr)
{
   pinMode(addr,OUTPUT);
   digitalWrite(addr,HIGH);
   // General purpose configuration
   AD5592write(addr, 3, 0x0100);
   // Set ext reference
   AD5592write(addr, 11, 0x0200);
   // Set LDAC mode
   AD5592write(addr, 7, 0x0000);
   // Set DO outputs channels
   AD5592write(addr, 8, 0x0006);
   // Set DAC outputs channels
   AD5592write(addr, 5, 0x0007);
   // Set ADC input channels
   AD5592write(addr, 4, 0x00F0);
   // Turn off all pulldowns
   AD5592write(addr, 6, 0x0000);
   
   // Set default values
   // Init DAC channels 0,1,2 to mid range
   AD5592writeDAC(addr, 0, 32767);
   AD5592writeDAC(addr, 1, 32767);
   AD5592writeDAC(addr, 2, 32767);
}

void setup() 
{    
  TWIadd = 0x50;
  if(digitalRead(TWIADD1) == HIGH) TWIadd |= 0x02;
  if(digitalRead(TWIADD2) == HIGH) TWIadd |= 0x04;
  LoadAltRev();
  // Read the flash config contents into Ebuf and test the signature
  *fptr = flash_FAIMSFBdata.read();
  if(fptr->Signature == SIGNATURE) faimsfb = *fptr;
  else faimsfb = Rev_1_faimsfb;
  memcpy(Ebuf,(void *)&faimsfb,sizeof(FAIMSFBdata));
  // Init serial communications
  SerialInit();
  // Init SPI
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(32);
  SPI.setDataMode(SPI_MODE2);
  FAIMSAD5592init(AD5592_CS);
  analogReadResolution(12);
  // Setup the 
 // bme.begin();
  // Setup TWI as slave to communicate with MIPS.
  Wire.begin(TWIadd);              // join i2c bus
  // Set mask register
  // bits 16 thru 23 are the address mask.
  // This code emulates the SEPROM and uses the base address for the first
  // 256 byte page and the base address + 1 for the second page.
  // Base address + 0x20 for the general command to this FAIMSFB module.
  // For example if base address is 0x50, then 0x51 is page 2 and 0x70 is
  // general commands.
  PERIPH_WIRE.disableWIRE();
  SERCOM3->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_ADDRMASK( 0x21ul );
  PERIPH_WIRE.enableWIRE();
  // register events
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Wire.onAddressMatch(AddressMatchEvent);
  initPWM();
  // Configure Threads
  SystemThread.setName("Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  // Print the signon version message
  sdata.update = true;
  serial->println(Version);
}

bool UpdateADCvalue(uint8_t SPIcs, ADCchan *achan, float *value, float filter)
{
  int   val;
  float fval;

  if((achan->Chan & 0x80) != 0)
  {
    // Here if this is a M0 ADC pin
    val = 0;
    for(int i=0;i<16;i++) val += analogRead(achan->Chan & 0x7F);
    //val = analogRead(achan->Chan & 0x7F) << 4;
    fval = Counts2Value(val,achan);
    if(*value == -1) *value = fval;
    else *value = filter * fval + (1 - filter) * *value;
    return true;    
  }
  if((val = AD5592readADC(SPIcs, achan->Chan)) != -1)
  {
    fval = Counts2Value(val,achan);
    if(*value == -1) *value = fval;
    else *value = filter * fval + (1 - filter) * *value;
    return true;
  }
  return false;
}

bool UpdateDACvalue(uint8_t SPIcs, DACchan *dchan, float *value, float *svalue)
{
  if((sdata.update) || (*value != *svalue))
  {
    AD5592writeDAC(SPIcs, dchan->Chan, Value2Counts(*value,dchan));
    *svalue = *value;
    return true;
  }
  return false;
}

// This function adjusts the drive level to achieve the desired Vrf level.
// This function used the drive level lookup table. This function is very fast 
// and depends on the lookup being accurate. The table has 21 entries, every
// MaxDrive/20 points in drive level. If the Max Drive level is changed then
// The table needs to be regenerated.
void SetVrfTable(float Vrf)
{
  int   i;
  float fStep = faimsfb.MaxDrive / 20;

  if(!faimsfb.Enable) return;   // Exit if system is not enabled
  for(i=0;i<21;i++)
  {
    if(Vrf < faimsfb.LUVrf[i])
    {
      if(i==0) return;
      faimsfb.Drive = i * fStep - ((faimsfb.LUVrf[i] - Vrf) / (faimsfb.LUVrf[i] - faimsfb.LUVrf[i-1])) * fStep;
      UpdateDrive(faimsfb.Drive);
      sdata.Vrf = faimsfb.Vrf = Vrf;
      return;
    }
  }
}

// This function adjusts the drive level to achieve the desired Vrf level.
// This operation is done by reading the Vrf level and adjusting the drive
// based on the error between current value and desired value, a control loop.
// This function is not fast.
void SetVrf(float Vrf)
{
   float error;

   if(!faimsfb.Enable) return;   // Exit if system is not enabled
   sdata.Vrf = faimsfb.Vrf = Vrf;
   for(int i=0;i<20;i++)
   {
      // Read the current Vrf level
      rb.Vrf = -1;
      for(int j=0;j<100;j++) UpdateADCvalue(0, &faimsfb.VRFMon, &rb.Vrf);
      // Calculate error and adjust the drive level
      error = Vrf - rb.Vrf;
      if(fabs(error) <= 5) return;
      if(fabs(error) <= Vrf * 0.01) return;
      faimsfb.Drive += error * 0.05;
      if(faimsfb.Drive > faimsfb.MaxDrive) faimsfb.Drive = faimsfb.MaxDrive;
      if(faimsfb.Drive < 0) faimsfb.Drive = 0;  
      UpdateDrive(faimsfb.Drive);
      delay(200);
   }
}

void VRFcontrolLoop(void)
{
  // If Enable is false exit
  if(!faimsfb.Enable) return;
  // If Mode is false exit
  if(!faimsfb.Mode) return;
  // Calculate error between actual and setpoint
  float error = faimsfb.Vrf - rb.Vrf;
  if(fabs(error) < 2.0) return;
  faimsfb.Drive += error * 0.005;
  if(faimsfb.Drive > faimsfb.MaxDrive) faimsfb.Drive = faimsfb.MaxDrive;
  if(faimsfb.Drive < 0) faimsfb.Drive = 0;  
}

// This function is called at 40 Hz and it does all the DAC and ADC updates
void Update(void)
{
  static bool LastEnableState=false;

  if(Power > faimsfb.MaxPower) faimsfb.Drive -= 1.0;
  if(faimsfb.Drive > faimsfb.MaxDrive) faimsfb.Drive = faimsfb.MaxDrive;
  if(faimsfb.Drive < 0) faimsfb.Drive = 0;
  // The following operations are performed reguardless of Enable status
  if((sdata.update) || (sdata.Freq != faimsfb.Freq))
  {
    UpdateFreqDuty();
    sdata.Freq = faimsfb.Freq;
  }
  if((sdata.update) || (sdata.Duty != faimsfb.Duty))
  {
    UpdateFreqDuty();
    sdata.Duty = faimsfb.Duty;
  }
  UpdateDACvalue(AD5592_CS, &faimsfb.DCrefCtrl, &faimsfb.DCref, &sdata.DCref);
  UpdateADCvalue(AD5592_CS, &faimsfb.DCVmon, &rb.V);
  UpdateADCvalue(AD5592_CS, &faimsfb.DCImon, &rb.I);
  if(InitMonValue) {InitMonValue=false; DCB1mon = DCB2mon = rb.Vrf = -1;}
  UpdateADCvalue(AD5592_CS, &faimsfb.DCB1Mon, &DCB1mon);
  UpdateADCvalue(AD5592_CS, &faimsfb.DCB2Mon, &DCB2mon);
  UpdateADCvalue(0, &faimsfb.VRFMon, &rb.Vrf);
  Power = rb.V * rb.I / 1000;
  // Calculate CVmon and BIASmon;
  rb.BIAS = DCB1mon + DCB2mon;
  rb.CV = (DCB1mon - DCB2mon) - rb.BIAS;
  // 
  if(faimsfb.Enable)
  {
    if(!LastEnableState) sdata.update=true;
    if((sdata.update) || (sdata.Drive != faimsfb.Drive))
    {
      UpdateDrive(faimsfb.Drive);
      sdata.Drive = faimsfb.Drive;
    }
    if((sdata.update) || (faimsfb.CV != sdata.CV))
    {
      AD5592writeDAC(AD5592_CS, faimsfb.DCB1Ctrl.Chan, Value2Counts(faimsfb.CV/2 + faimsfb.Bias,&faimsfb.DCB1Ctrl));
      AD5592writeDAC(AD5592_CS, faimsfb.DCB2Ctrl.Chan, Value2Counts(-faimsfb.CV/2 + faimsfb.Bias,&faimsfb.DCB2Ctrl));
      sdata.CV = faimsfb.CV;
    }
    if((sdata.update) || (faimsfb.Bias != sdata.Bias))
    {
      AD5592writeDAC(AD5592_CS, faimsfb.DCB1Ctrl.Chan, Value2Counts(faimsfb.CV/2 + faimsfb.Bias,&faimsfb.DCB1Ctrl));
      AD5592writeDAC(AD5592_CS, faimsfb.DCB2Ctrl.Chan, Value2Counts(-faimsfb.CV/2 + faimsfb.Bias,&faimsfb.DCB2Ctrl));
      sdata.Bias = faimsfb.Bias;
    }
  }
  else
  {
    UpdateDrive(0);
    AD5592writeDAC(AD5592_CS, faimsfb.DCB1Ctrl.Chan, Value2Counts(0,&faimsfb.DCB1Ctrl));
    AD5592writeDAC(AD5592_CS, faimsfb.DCB2Ctrl.Chan, Value2Counts(0,&faimsfb.DCB2Ctrl));
  }
  VRFcontrolLoop();
  sdata.update = false;
  LastEnableState = faimsfb.Enable;
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan = true)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

void loop() 
{
  ProcessSerial();
  control.run();
  if(GenerateCalibrationTable)
  {
    GenerateCalibrationTable = false;
    CalibrateVrf2Drive();
  }
  if(SetVrfFlag)
  {
    SetVrf(faimsfb.Vrf);
    SetVrfFlag = false;
  }
  if(SetVrfUseTable)
  {
    SetVrfTable(faimsfb.Vrf);
    SetVrfUseTable = false;
  }
}

//
// Scanning functions. 
//  Scanning can be controlled by a timer in this program or scanning can be advanced via the MIPS controller and
//  an event is triggered to advance the scan. Scan results are sent back using the serial port or the TWI interface
//  with MIPS. Scan results are sent ASCII using the serial port, the results are sent binary in a scan structure when 
//  using the TWI interface to MIPS.
//
//  Thw TWI scan report can be disabled, by default it is enabled.
//
//  Need functions to:
//    1.) Init the scan
//    2.) Abort / stop a scan
//    3.) Report scan point data, these parameters are reported:
//        - Scan point, 16 bit
//        - Time stamp, 32 bit
//        - Vrf setpoint, float
//        - Vrf readback, float
//        - CV setpoint, float
//        - CV readback, float
//        - 22 bytes (176 bits) total, assume 400KHz TWI this will take 440uS + overhead.
//    4.) Advance parameters to next point
//    5.) Need working variables for:
//        - Current step
//

// This function is called at each scan advancement point. The following actions are taken:
// - Current scan point data is read and ScanPoint structure updated
// - Values updated to next scan point
// - Scan point data reported

uint32_t   ScanStartTime;

void ScanISR(void)
{
  if(!Scanning) return;
// Update ScanPoint structure
  sp.Point = CurrentScanStep + 1;
  sp.TimeStamp = millis() - ScanStartTime;
  sp.Vrf = faimsfb.Vrf;
  sp.CV = faimsfb.CV;
// Advance current point
  CurrentScanStep++;
  if(CurrentScanStep > faimsfb.Steps)
  {
    StopScan();
    return;
  }
  SetScanParameters(CurrentScanStep);
// Report the results
  ReportScanValues(ReportStream,ASCIIformat);
}

// This function starts a scan using internal timing generation and reporting results
// using the serial port. This function is called by the command processor.
void InitScanLocal(void)
{
   InitScan(true);
   SendACK;
}

// Setup the scan system.
void InitScan(bool ascii)
{
   Scanning = true;
// Save current settings
   CurrentScanStep = 0;
   InitialVrf = faimsfb.Vrf;
   InitialDrive = faimsfb.Drive;
   InitialCV = faimsfb.CV;
// Setup variables
   ReportStream = serial;
   ASCIIformat = ascii;
// Set first point
   SetScanParameters(CurrentScanStep);
   ScanStartTime = millis();    // Record the start of scan time
   if(faimsfb.EnableExtStep)
   {
     pinMode(faimsfb.ExtAdvInput,INPUT);
     attachInterrupt(faimsfb.ExtAdvInput, ScanISR, RISING);
   }
   else
   {
     // Start the timer
     tcConfigure(faimsfb.StepDuration,ScanISR);
     tcStartCounter(); //starts the timer    
   }
}

// Called to stop an active scan, this function is called by the command processor
void StopScanLocal(void)
{
    StopScan();
    SendACK;
}

void StopScan(void)
{
   if(!Scanning) return;
   Scanning = false;
//   if(faimsfb.EnableExtStep) detachInterrupt(digitalPinToInterrupt(faimsfb.ExtAdvInput));
   if(faimsfb.EnableExtStep) detachInterrupt(faimsfb.ExtAdvInput);
   else tcReset();
// Restore orginal values, signal the update loop to restore the values
   faimsfb.Drive = InitialDrive;
   sdata.Drive = faimsfb.Drive - 1;
   faimsfb.Vrf = InitialVrf;
   sdata.Vrf = faimsfb.Vrf - 1;
   faimsfb.CV = InitialCV;
   sdata.CV = faimsfb.CV - 1;
}

// This function calculates the CV and Vrf for this point and
// sets the values.
void SetScanParameters(int ScanPoint)
{
  float CVss,VRFss;
  
  if(faimsfb.CVend != faimsfb.CVstart)
  {
     CVss = (faimsfb.CVend - faimsfb.CVstart) / faimsfb.Steps;
     faimsfb.CV = faimsfb.CVstart + CVss * ScanPoint;
     AD5592writeDAC(AD5592_CS, faimsfb.DCB1Ctrl.Chan, Value2Counts(faimsfb.CV/2 + faimsfb.Bias,&faimsfb.DCB1Ctrl));
     AD5592writeDAC(AD5592_CS, faimsfb.DCB2Ctrl.Chan, Value2Counts(-faimsfb.CV/2 + faimsfb.Bias,&faimsfb.DCB2Ctrl));
     sdata.CV = faimsfb.CV;
  }
  if(faimsfb.VRFend != faimsfb.VRFstart)
  {
     VRFss = (faimsfb.VRFend - faimsfb.VRFstart) / faimsfb.Steps;
     faimsfb.Vrf = faimsfb.VRFstart + VRFss * ScanPoint;
     SetVrfTable(faimsfb.Vrf);
     sdata.Vrf = faimsfb.Vrf;
  }
  InitMonValue = true;
}

// This function reports the current scan point settings to the stream pointer passed.
// If ASCII is true then the data is printed in a string otherwise its sent as a binary
// structure, ScanPoint.
void ReportScanValues(Stream *s, bool ASCII)
{
  if(ASCII)
  {
    s->print(sp.Point); s->print(",");
    s->print(sp.TimeStamp); s->print(",");
    s->print(sp.Vrf); s->print(",");
    s->println(sp.CV);
    return;
  }
  if(!ScanReport) return;
  uint8_t  *bptr;
  bptr = (uint8_t *)&sp;
  for(int i=0;i<sizeof(ScanPoint);i++) sb.write(bptr[i]); 
}

//
// Host command functions
//

void SetVrfCmd(char *V)
{
   String sToken;

   sToken = V;
   float Vrf = sToken.toFloat();
   if((Vrf < 0) || (Vrf > 2000))
   {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;    
   }
   SetVrf(Vrf);
   SendACK;
}

void SetVrfTableCmd(char *V)
{
   String sToken;

   sToken = V;
   float Vrf = sToken.toFloat();
   if((Vrf < 0) || (Vrf > 2000))
   {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;    
   }
   SetVrfTable(Vrf);
   SendACK;
}

void SaveSettings(void)
{
  faimsfb.Signature = SIGNATURE;
  flash_FAIMSFBdata.write(faimsfb);
  SendACK;
}

void RestoreSettings(void)
{
  // Read the flash config contents into Ebuf and test the signature
  *fptr = flash_FAIMSFBdata.read();
  if(fptr->Signature == SIGNATURE) faimsfb = *fptr;
  else
  {
    // copy faimsfb to Ebuf if restore failed
    *fptr = faimsfb;
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

void Software_Reset(void)
{
  NVIC_SystemReset();  
}

void FormatFLASH(void)
{
  flash_FAIMSFBdata.write(Rev_1_faimsfb);  
  SendACK;
}

void Debug(int i)
{
}
