#ifndef Hardware_h
#define Hardware_h

// SPI chip selects
#define AD5592_CS     1
#define BME280_CS     4

// DIO lines
#define TWIADD1       7
#define TWIADD2       6
#define TWIADDWRENA   12

#define SEALEVELPRESSURE_HPA (1013.25)

// AD5592 channel assignments
#define DBC1          0           // DAC output channel, DC bias FAIMS plate 1
#define DBC2          1           // DAC output channel, DC bias FAIMS plate 2
#define DBCREF        2           // DAC output channel, DC bias reference voltage
#define DCB1MON       4           // ADC input channel, DC bias FAIMS plate 1 readback
#define DCB2MON       5           // ADC input channel, DC bias FAIMS plate 2 readback
#define VMON          6           // DC voltage into coil, monitor
#define IMON          7           // Current into coil, monitor

// M0 pin assigenments
#define RFLVL         A0          // RF output level monitor, analog input

typedef struct
{
  uint8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M0 ADC channel number
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  int8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Function prototypes
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);
void  AD5592write(int CS, uint8_t reg, uint16_t val);
int   AD5592readWord(int CS);
int   AD5592readADC(int CS, int8_t chan);
int   AD5592readADC(int CS, int8_t chan, int8_t num);
void  AD5592writeDAC(int CS, int8_t chan, int val);
void  printBME280(void);

void  initPWM(void);
void  UpdateFreqDuty(void);
void  UpdateDrive(float drive);

void ProgramFLASH(char * Faddress,char *Fsize);

#endif
