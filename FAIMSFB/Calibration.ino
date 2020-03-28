//
// Need to add calibration function for Vrf readback value.
// Need to add calibration function for Vrf to drive level, this requires
// no user interaction to perform.
//
void CalibrateLoop(void)
{
  ProcessSerial(false);
  control.run();
}

int Calibrate5592point(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float *V)
{
  char   *Token;
  String sToken;

  // Set value and ask for user to enter actual value read
  if(dacchan !=NULL) AD5592writeDAC(SPIcs, dacchan->Chan, Value2Counts(*V,dacchan));
  serial->print("Enter actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  sToken = Token;
  serial->println(Token);
  *V = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
  if(adcchan != NULL) return AD5592readADC(SPIcs, adcchan->Chan, 10); 
  return 0; 
}

// This function is used to calibrate ADC/DAC AD5592 channels. 
void Calibrate5592(uint8_t SPIcs, DACchan *dacchan, ADCchan *adcchan, float V1, float V2)
{
  float  val1,val2,m,b;
  int    adcV1, adcV2;
  int    dacV1, dacV2;

  serial->println("Enter values when prompted.");
  // Set to first voltage and ask for user to enter actual voltage
  val1 = V1;
  adcV1 = Calibrate5592point(SPIcs, dacchan, adcchan, &val1);
  // Set to second voltage and ask for user to enter actual voltage
  val2 = V2;
  adcV2 = Calibrate5592point(SPIcs, dacchan, adcchan, &val2);
  // Calculate calibration parameters and apply
  dacV1 = Value2Counts(V1, dacchan);
  dacV2 = Value2Counts(V2, dacchan);
  m = (float)(dacV2-dacV1) / (val2-val1);
  b = (float)dacV1 - val1 * m;
  serial->println("DAC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  dacchan->m = m;
  dacchan->b = b;
  if(adcchan == NULL) return;
  m = (float)(adcV2-adcV1) / (val2-val1);
  b = (float)adcV1 - val1 * m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  adcchan->m = m;
  adcchan->b = b;
}

void CalibrateREF(void)
{
  serial->println("Calibrate DCB reference output, monitor with a voltmeter.");
  Calibrate5592(AD5592_CS, &faimsfb.DCrefCtrl, NULL, 0.0, 1.25);
  AD5592writeDAC(AD5592_CS, faimsfb.DCrefCtrl.Chan, Value2Counts(1.25,&faimsfb.DCrefCtrl));
 
}

void CalibrateDCB1(void)
{
  serial->println("Calibrate DCB1 output, monitor with a voltmeter.");
  Calibrate5592(AD5592_CS, &faimsfb.DCB1Ctrl, &faimsfb.DCB1Mon, 0.0, 12.0);
  AD5592writeDAC(AD5592_CS, faimsfb.DCB1Ctrl.Chan, Value2Counts(faimsfb.CV/2,&faimsfb.DCB1Ctrl));
}

void CalibrateDCB2(void)
{
  serial->println("Calibrate DCB2 output, monitor with a voltmeter.");
  Calibrate5592(AD5592_CS, &faimsfb.DCB2Ctrl, &faimsfb.DCB2Mon, 0.0, 12.0);
  AD5592writeDAC(AD5592_CS, faimsfb.DCB2Ctrl.Chan, Value2Counts(-faimsfb.CV/2,&faimsfb.DCB2Ctrl));
}

void CalibrateVrf(void)
{
  char   *Token;
  String sToken;
  float Vrf1, Vrf2;
  int   ADC1,ADC2;

  serial->println("Calibrate Vrf readback, monitor Vrf with a scope.");
// Set drive level to 10% and ask for Vrf voltage
  UpdateDrive(10);
  serial->print("Enter Vrf actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  ADC1 = 0;
  for(int i = 0;i<1000;i++)
  {
    ADC1 += analogRead(A0);
    delay(1);
  }
  ADC1 /= 1000;
  ADC1 <<= 4;
  sToken = Token;
  serial->println(Token);
  Vrf1 = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
// Set drive level to 50% and ask for Vrf voltage
  UpdateDrive(50);
  serial->print("Enter Vrf actual value: ");
  while((Token = GetToken(true)) == NULL) CalibrateLoop();
  ADC2 = 0;
  for(int i = 0;i<1000;i++)
  {
    ADC2 += analogRead(A0);
    delay(1);
  }
  ADC2 /= 1000;
  ADC2 <<= 4;
  sToken = Token;
  serial->println(Token);
  Vrf2 = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) CalibrateLoop(); 
// Calculate the calibration parameters
  // ADC1 = Vrf1*m + b
  // ADC2 = Vrf2*m + b
  // m = (ADC1 - ADC2) / (Vrf1-Vrf2)
  // b = ADC2 - Vrf2 * m;
  faimsfb.VRFMon.m =  (ADC1 - ADC2) / (Vrf1-Vrf2);
  faimsfb.VRFMon.b =  ADC2 - Vrf2 * faimsfb.VRFMon.m;
  serial->println("ADC channel calibration parameters.");
  serial->print("m = ");
  serial->println(faimsfb.VRFMon.m);
  serial->print("b = ");
  serial->println(faimsfb.VRFMon.b);
// Restore drive level
  UpdateDrive(faimsfb.Drive);
  delay(100);
  rb.Vrf = -1;
  for(int i=0;i<1000;i++) UpdateADCvalue(0, &faimsfb.VRFMon, &rb.Vrf);
}

// Scan through drive level 0, to Max Drive in 11 steps and record the Vrf
// voltage. This will be used in the step function to quickly set the
// desired Vrf level.
void CalibrateVrf2Drive(void)
{
  if(!faimsfb.Enable) return;   // Exit if system is not enabled
  for(int i=0;i<21;i++)
  {
    // Set the drive level
    UpdateDrive(i*(faimsfb.MaxDrive/20));
    // Wait for things to stabalize
    delay(250);
    // Read the Vrf level
    rb.Vrf = -1;
    for(int j=0;j<100;j++) UpdateADCvalue(0, &faimsfb.VRFMon, &rb.Vrf);
    // Save data in lookup table
    faimsfb.LUVrf[i] = rb.Vrf;
    //serial->println(rb.Vrf);
  }
// Restore drive level
  UpdateDrive(faimsfb.Drive);
  delay(100);
  rb.Vrf = -1;
  for(int i=0;i<100;i++) UpdateADCvalue(0, &faimsfb.VRFMon, &rb.Vrf);
}
