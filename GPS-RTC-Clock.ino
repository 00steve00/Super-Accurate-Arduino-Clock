
#include <Wire.h> //for RTC comms over I2C
#include <SPI.h>  //for LED output to Max7219 module

//THIS VERSION FOR ARDUINO NANO OR UNO
// Commit Date: March 4, 2020

//Nano pins: D10-D13, SPI for Max 7219 8 digit 7 seg LED module;  A4(SDA) and A5(SCL) for I2C to DS3231 RTC; D0-D1 for serial.
//           D2 for IR receiver interrupt. D4 for RTC-SQW. D5 for GPS-PPS.

//STATE MACHINE SETUP//
  enum { DEBUG, BOOTUP, REG_OPS, TOGGLE_DISPLAY, GPS_INIT, GPS_PPS_SYNC, GPS_NMEA_SYNC} StateMachine;

//PIN DEFS & ADDRESSES//
  const byte RTC_SQW_Pin = 4; 
  const byte GPS_PPS_Pin = 5;
  const byte ir_pin = 2;    //Interrupt pin for IR receiver data pin.
  const byte ChipSelectPin = 10;  // For Max7219. We set the SPI Chip Select/Slave Select Pin here. 10 for uno/nano. 53 for mega
  
  const int RTC_I2C_ADDRESS = 0x68;  // Must be type [int] to keep wire.h happy. Sets RTC DS3231 RTC i2C address. 
  

//TIMERS AND EDGE DETECTORS//
  unsigned long GPS_INIT_t0, t0, t1, t2; //for timers
  
  byte RTC_SQW_Current = LOW;   
  byte RTC_SQW_Prev = LOW;
  byte GPS_PPS_Current = HIGH;
  byte GPS_PPS_Prev = HIGH;

//ISR HANDLERS//
  volatile unsigned int pulseChangeTime;  
  volatile byte pulseFlag = 0;


//UTC offset handlers//
  bool UTC_offset_enable = true; // False for UTC time. True for local time.
  
  const char offsetStandardHr = -5; 
  const char offsetStandardMin = 0;
  const char offsetDSTHr = -4; 
  const char offsetDSTMin = 0;

  const byte startDST[4] = {2,0,3,2}; // {nth,day of week,month,hh} [0..6] for [Sunday...Saturday]
  const byte startStandard[4] = {1,0,11,2}; // {nth,day of week,month,hh} [0..6] for [Sunday...Saturday]
                                 // set n=5 for 5th or last. Assume DST starts/stops at 2am local time
 
  const byte days[] = {0,31,28,31,30,31,30,31,31,30,31,30,31}; // mapping days of the month

  /* globals that store our offset*/
  int  offYYYY;
  byte offMO;
  byte offDD;
  byte offHH;
  byte offMM;
  byte offSS;

//RTC date+time holders//
  byte ssRTC, mmRTC, hhRTC, dowRTC, ddRTC, moRTC, ctyRTC, yyRTC; 

//GPS UTC date+time handlers//
  byte hhGPS, mmGPS, ssGPS, ddGPS, moGPS;
  int yyyyGPS; 
  
  bool newGPS_dateAvail = false;
  bool newGPS_timeAvail = false;
  bool NMEA_processFlag = false;
  bool GGA_msg = false;
  bool RMC_msg = false;
  bool ZDA_msg = false;
  bool msgStart = false;
  
  bool GPS_sec_primed = false;
  bool PPS_done = false;
         
  byte byteIndex = 0;


// SONY REMOTE CONTROL HANDLERS //
unsigned int IR_start_bit = 2000;                 //Start bit threshold (Microseconds) 2408
unsigned int IR_1 = 1000;                      //Binary 1 threshold (Microseconds) 1184-1240
unsigned int IR_0 = 400;                       //Binary 0 threshold (Microseconds) 556-640
unsigned int IR_timeout = 2700;


/* ----------------------- MAX STUFF ALL HERE ---------------------------vvvvvv
ICSP BLOCK PINOUT
5 3 1
6 4 2

===MAPPING SPI HARDWARE AND ARDUINO'S TO MAX7219 MODULES===

7219 MODULE:  DIN          LOAD(CS)     CLK         (*n/a*)
SPI:          MOSI         SS           SCK          MISO
UNO/NANO:     11/ICSP-4    10           13/ICSP-3    12/ICSP-1
MEGA:         51/ICSP-4    53           52/ICSP-3    50/ICSP-1
* 
[CS(SS) can actually be any unused PIN on the uController, but these
are the typical conventions.]
*/


// Max7219 digit bitmaps for digit registers 0x01 to 0x08
  
  const byte dp     = 0b10000000; // Decimal Point. Do bitwise OR to combine with any digit to add decimal point

  const byte blank  = 0b00000000;
  const byte hyphen = 0b00000001;
  const byte excl   = 0b10100000;  //exclamation point!
  
    //           0b0abcdefg
  const byte A = 0b01110111;
  const byte B = 0b00011111;
  const byte C = 0b01001110;
  const byte D = 0b00111101;
  const byte E = 0b01001111;
  const byte F = 0b01000111;
  const byte G = 0b01111011;
  const byte H = 0b00110111;
  const byte I = 0b00110000;
  const byte J = 0b00111100;
  const byte L = 0b00001110;
  const byte N = 0b00010101;
  const byte O = 0b00011101;
  const byte P = 0b01100111;
  const byte Q = 0b01110011;  
  const byte R = 0b00000101;
  const byte S = 0b01011011;
  const byte T = 0b00001111;
  const byte U = 0b00011100;
  const byte M, K, V, W, X 
               = 0b00000000;
  const byte Y = 0b00111011;
  const byte Z = 0b01101101;


  byte char_library[28] = {blank,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,hyphen};

  //array of digits 0-9 with corresponding segment bit maps
    byte digit[10] = {0b01111110,  //0
                      0b00110000,  //1
                      0b01101101,  //2
                      0b01111001,  //3
                      0b00110011,  //4
                      0b01011011,  //5
                      0b01011111,  //6
                      0b01110000,  //7
                      0b01111111,  //8
                      0b01110011}; //9 

        //            0b0abcdefg
        //      -- a
        //  f  |  |  b
        //    g --
        //   e |  | c
        //    d --
        //0b0abcdefg

// Max7219 Address Registers 
    const byte reg_nonop = 0x00;  // generally not used
    const byte reg_d1 = 0x01; // "Digit0" in the datasheet
    const byte reg_d2 = 0x02; // "Digit1" in the datasheet
    const byte reg_d3 = 0x03; // "Digit2" in the datasheet
    const byte reg_d4 = 0x04; // "Digit3" in the datasheet
    const byte reg_d5 = 0x05; // "Digit4" in the datasheet
    const byte reg_d6 = 0x06; // "Digit5" in the datasheet
    const byte reg_d7 = 0x07; // "Digit6" in the datasheet
    const byte reg_d8 = 0x08; // "Digit7" in the datasheet
    const byte reg_decode = 0x09; // 0x00 no decode, to 0xFF decode all; use a bit map to toggle, ie 0b00000010
    const byte reg_intensity = 0x0A; // min 0x00, max 0x0F... 16 duty-cycle options. 0x07 middle.
    const byte reg_scanlimit = 0x0B; // from 0x00 to 0x07 (sets number of digits being scanned)
    const byte reg_shutdown = 0x0C; // 0x00 shutdown mode, 0x01 normal ops
    const byte reg_displaytest = 0X0F; // 0x00 normal ops, 0x01 display test mode
    
void SPIwrite (byte reg_address, byte regdata) {  
    //Writes 2 bytes to SPI. This is optimized for Max7219 Comms.
    
    digitalWrite(ChipSelectPin,LOW);  // take the CS/SS pin low to select the chip
    SPI.transfer(reg_address);
    SPI.transfer(regdata);
    digitalWrite(ChipSelectPin,HIGH); // take the CS/SS pin high to deselect the chip
  
} //end of SPIwrite

void setAllDigitsTo (byte set_digit) { 
  //this sets all the digits to set_digit
  
  for (byte i=1;i<=8;i++) {
      SPIwrite(i,set_digit);
  } //end for
  
} //end of setAllDigitsTo()


void initializeMax7219() {

    //reset the Max by activating shutdown mode   
    SPIwrite(reg_shutdown,0x00); // 0x00 shutdown, 0x01 normal ops
  
    //initialize each digit with known values
    setAllDigitsTo(hyphen);
  
    //set intensity
    SPIwrite(reg_intensity, 0x07); // min 0x00, max 0x0F... 16 duty-cycle options. 0x07 middle.
  
    //set scan limit
    SPIwrite(reg_scanlimit, 0x07); // from 0x00 to 0x07 (sets number of digits being scanned)
    
    //set decode
    SPIwrite(reg_decode, 0b00000000); // built-in decode, from 0x00 [all off] to 0xFF [all on] (bit map toggles digits 1-8).

    //flash a display test
    SPIwrite(reg_displaytest, 0x01); // 0x00 normal ops, 0x01 display test mode
    delay(100);

    //exit test
    SPIwrite(reg_displaytest, 0x00); // 0x00 normal ops, 0x01 display test mode
    delay(100);

    //exit shutdown mode. resume normal ops
    SPIwrite(reg_shutdown,0x01); // 0x00 shutdown, 0x01 normal ops
    delay (100);
    
} //end of initializeMax7219

void maxDisplay(byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
  SPIwrite(8,a);
  SPIwrite(7,b);
  SPIwrite(6,c);
  SPIwrite(5,d);
  SPIwrite(4,e);
  SPIwrite(3,f);
  SPIwrite(2,g);
  SPIwrite(1,h);
}


//**** ISR ROUTINE ****//

void ISR_pulse_detected() {
  pulseChangeTime = micros();
  pulseFlag = 1;
} //end of ISR_pulse_detected()


//**** SONY REMOTE ROUTINES  ****//

void processSonyIR(int code) {

  if (code == 0xCD15) {
    displayRTCDate();
    StateMachine = TOGGLE_DISPLAY;
    t1 = millis(); //sets t1 for date display in TOGGLE_DISPLAY state machine
  } //end if

  if (code == 0xCD61) {
    UTC_offset_enable = !UTC_offset_enable;
  } //end if

  if (code == 0xCD10) {
    if (UTC_offset_enable) {
      maxDisplay(L,O,C,A,L,blank,blank,blank);
    } //end if
    else {
      maxDisplay(blank,U,T,C,blank,blank,blank,blank);
    } //end else
    StateMachine = TOGGLE_DISPLAY;
    t1 = millis(); //sets t1 for date display in TOGGLE_DISPLAY state machine  
  } //end if
} //end processSonyIR()

void SonyIR_analyzer() {
  static bool startFlag = false;
  static byte bitCount = 99; 
  static bool valid = false;
  static byte state = 0;  // {0 startup, 1 initial, 2 process}
  
  static unsigned int lastTime = 0;
  static unsigned int delta = 0;
  static unsigned int IRvalue = 0;  //could experiment with this being 16. might work fine.

  delta = pulseChangeTime - lastTime;

  pulseFlag = 0; // reset the global volatile that got us here

  if (delta > IR_start_bit && delta < IR_timeout) { //we test for a startbit first
     state = 1;
  } //end if
  
  switch (state) {
    case 1: 
      startFlag = true;
      bitCount = 0;
      valid = false; //  even is false, odd is true for bitCount
      IRvalue = 0;
      state = 2;
      break; //end case 1

    case 2:
        if (startFlag && valid) {  //we have started, and valid bit -- ie, not a spacer.
          bitCount++;
          IRvalue = IRvalue | ( (delta > IR_1) << (bitCount - 1));
          valid = !valid;
          if (bitCount == 20) {   //bitfield full. Do final process on IRvalue.
            processSonyIR(IRvalue);  //action we take with IRvalue.
            startFlag = false;
            IRvalue = 0;
            state = 0;
            bitCount = 99;
          } //end if
        } //end if
        else if (startFlag && !valid) {
          valid = !valid;
        } //end elseif
        break; //end case 2
  }// end switch

  lastTime = pulseChangeTime;
  
} //end SonyIR_analyzer()


//**** UNIVERSAL FUNCTIONS ****//

byte bcd2dec(byte n) {  
  // Converts binary coded decimal to normal decimal numbers
  return ((n/16 * 10) + (n % 16));
} //end of bcd2dec()

byte dec2bcd(byte n) {  //n must be in range [0..99] incl
  // Converts normal decimal to binary coded decimal. Speed optimized.
  // return ((n / 10 * 16) + (n % 10));  <----- slower method.
  // see https://forum.arduino.cc/index.php?topic=185235.msg1372439#msg1372439
  
  uint16_t a = n;
  byte b = (a*103) >> 10;
  return  n + b*6;
  
} //end of dec2bcd()
    

void clearSerialInputBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }//end while
} //end of clearSerialInputBuffer()

 
//****  DISPLAY HANDLERS **** //

void displayRTC_timeOnMax(byte rtc_h, byte rtc_m, byte rtc_s) { //receiving [0-99] decimals from caller

  maxDisplay(digit[(rtc_h/10)%10],digit[rtc_h%10],hyphen,
             digit[(rtc_m/10)%10],digit[rtc_m%10],hyphen,
             digit[(rtc_s/10)%10],digit[rtc_s%10]);
  
} //end of displayRTC_timeOnMax()


//**** GPS HANDLERS **** //

bool PPS_detect() {
    GPS_PPS_Prev = GPS_PPS_Current;
    GPS_PPS_Current = digitalRead(GPS_PPS_Pin);
    return (GPS_PPS_Prev == LOW && GPS_PPS_Current == HIGH); //returns true if PPS has gone high!
} // end of PPS_detect()

void processNMEA() {
  byte inByte;
  if (Serial.available() > 0) {  //do all of this only if byte ready to read
    inByte = Serial.read();
    byteIndex++;    // we only increment index if we read a byte
  
    switch (byteIndex) {
      case 1 ... 3:
        break;
      case 4:
         GGA_msg = (inByte == 'G');
         RMC_msg = (inByte == 'R');
         ZDA_msg = (inByte == 'Z');
         break;
      case 5:
         GGA_msg = (GGA_msg && inByte == 'G');
         RMC_msg = (RMC_msg && inByte == 'M');
         ZDA_msg = (ZDA_msg && inByte == 'D');
         break;
      case 6:
         GGA_msg = (GGA_msg && inByte == 'A');
         RMC_msg = (RMC_msg && inByte == 'C');
         ZDA_msg = (ZDA_msg && inByte == 'A');
         NMEA_processFlag = (GGA_msg || RMC_msg || ZDA_msg); // if we have any of these, we keep processing
         break;
      case 7:
        break;
      case 8:   // hour tens
         hhGPS = (inByte - '0')*10;
         break;
      case 9:   //hour units
         hhGPS += (inByte - '0');
         break;
      case 10:  // min tens
         mmGPS = (inByte - '0')*10;
         break;
      case 11:  //min units
         mmGPS += (inByte - '0');
         break;
      case 12:  // sec tens
         ssGPS = (inByte - '0')*10;
         break;
      case 13:  //sec units
         ssGPS += (inByte - '0');  
         //**AT THIS POINT WE HAVE GPS TIME**// CAN ACTUALLY UPDATE RTC!
         newGPS_timeAvail = true;
         NMEA_processFlag = ZDA_msg;  //if we have ZDA, this is true and we keep processing.
         break;
      case 14 ... 17:  //** WILL NEED TO TUNE THIS TO ACTUAL OUTPUT OF NEO 6M
         break;
      case 18:    //day tens
         ddGPS = (inByte - '0')*10;  
         break;
      case 19:    //day units
         ddGPS += (inByte - '0');
         break;
      case 20:
         break;
      case 21:    //mo tens
         moGPS = (inByte - '0')*10;
         break;
      case 22:    //mo units
         moGPS += (inByte - '0');
         break;
      case 23:
         break;
      case 24:
         yyyyGPS = (inByte - '0')*1000;
         break;
      case 25:
         yyyyGPS += (inByte - '0')*100;
         break;      
      case 26:
         yyyyGPS += (inByte - '0')*10;
         break;      
      case 27:
         yyyyGPS += (inByte - '0');
         newGPS_dateAvail = true;   //all date fields now read
         NMEA_processFlag = false;  //and no need to process further
         break;
      default:
         NMEA_processFlag = false;
      
    } //end switch byteIndex
  } //end if Serial available
} //end of processNMEA

/*                  //      123456789012345678901234567|||  [27] GPS unit gives 2 decimals on time
                  //        $GPGGA,hhmmss.tt,...
    //  /* $GPGGA, $GPRMC:  $GPRMC,hhmmss.tt,...
    //     $GPZDA:          $GPZDA,hhmmss.tt,dd,mm,yyyy,...
*/

//**** UTC TIMEZONE OFFSET AND DST HANDLERS ****//

void offsetAdj(int y, byte mo, byte d, byte h, byte m, char offsetHr, char offsetMin) {
  offMM = (m + offsetMin + 120) % 60;
  if (m + offsetMin > 59) {
    offsetHr ++;
  }
  if (m + offsetMin < 0) {
    offsetHr --;
  }
  offHH = (h + offsetHr + 24) % 24;
  offDD = d;
  offYYYY = y;
  offMO = mo;
  if (offsetHr + h < 0) {
     //Do a decrement
     if (d>1) {
      offDD--;
     }
     else { //rollover
      offDD = days[((mo+11-1)%12+1)] + (mo==2 && isLeap(y));
      offMO--;
      if (mo == 1) {
        offMO = 12;
        offYYYY = y - 1;
      }//end if
     }
  } //end if for decrement day
  if (offsetHr + h >= 24) {
   // Do an increment
    if (d == (days[mo] + (mo == 2 && isLeap(y)))) { //ie, if d is last day of month
      offDD = 1;
      offMO++;
      if (mo == 12) {
        offMO = 1;
        offYYYY = y + 1;
      }
    }//end if
    else {
      offDD = d+1;
    }
  } //end if increment day
} //end offsetAdj

bool isLeap (byte y) {
  return ((y%4==0) || (!(y%100==0) &&  (y%400==0)));
} //end isLeap()

byte dowDate(int y, byte n, byte dow_target, byte m) { // returns date number in month of nth day of week in month
  char temp = 1; // char because could be negative. Set at 1 for 1st of month.
  byte maxDays = days[m]; // number of days in given month
  if (m==2) {
    maxDays = days[m] + isLeap(y); 
  }
  byte startDOW = dow(y,m,1);  //gets dow of 1st of given month 
  temp += dow_target - dow(y,m,1);
  if (temp < 0) {
    temp += 7;
  } //end if
  temp += 7*(n-1);
  if (temp > maxDays) {
    temp -= 7;
  }
  return temp;
} //end dowDate()
                    
byte dow(int y, byte m, byte d) { //pass non-leading zero values
  static byte t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
  y -= m < 3;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
  // returns [0..6] for [Sunday...Saturday]
} //end of dow()
  
void getLocalTime(int y, byte mo, byte d, byte h, byte m) {
  // **this function sets offMO,offDD, offYY, offHH, offMM to local time**
  // we first get a baseline offset
  offsetAdj(y,mo,d,h,m,offsetStandardHr,offsetStandardMin);

  // next we do our DST checks, and recalc offset if DST is in effect
 
  if (offMO > startDST[2] && offMO < startStandard[2]) {
      offsetAdj(y,mo,d,h,m,offsetDSTHr,offsetDSTMin);
  } //end if
  else if (offMO == startDST[2]) {
      byte dowDateDST = dowDate(y,startDST[0],startDST[1],startDST[2]);
      if (offDD > dowDateDST) {
        offsetAdj(y,mo,d,h,m,offsetDSTHr,offsetDSTMin);
      } //end if
      else if (offDD == dowDateDST && offHH >= startDST[3]) {
        offsetAdj(y,mo,d,h,m,offsetDSTHr,offsetDSTMin);
      } //end else if
  } //end else if
  else if (offMO == startStandard[2]) {
      byte dowDateStd = dowDate(y,startStandard[0],startStandard[1],startStandard[2]);
      if (offDD < dowDateStd) {
        offsetAdj(y,mo,d,h,m,offsetDSTHr,offsetDSTMin);
      }// end if
      else if (offDD == dowDateStd && offHH < (startStandard[3]-1)) {
        offsetAdj(y,mo,d,h,m,offsetDSTHr,offsetDSTMin);  
      }// end else if
  } //end else if
} //end getLocalTime()


//**** RTC HANDLERS ****//

void getRTC() { //reads all current time/date data from DS3231 chip via I2C
  // ssRTC, mmRTC, hhRTC, dowRTC, ddRTC, moRTC, ctyRTC, yyRTC;

  byte temp_buffer;

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(0x00);   //set register points to address 00h on DS3231
  Wire.endTransmission();
  Wire.requestFrom(RTC_I2C_ADDRESS, 7); // need 7 reads to clear this.
  ssRTC = bcd2dec(Wire.read()); // read reg 0  [range 00-59]
  mmRTC = bcd2dec(Wire.read()); // read reg 1 [range 00-59]
  hhRTC = bcd2dec(Wire.read() & 0b00111111); // read reg 2 and mask out BITS 7-8 
  dowRTC = bcd2dec(Wire.read()); // read reg 3 [range 1-7]
  ddRTC = bcd2dec(Wire.read());  // read reg 4 [range 01-31]
  temp_buffer = bcd2dec(Wire.read()); // read reg 5
  moRTC = bcd2dec(temp_buffer & 0b00011111); 
  ctyRTC = temp_buffer >> 7;
  yyRTC = bcd2dec(Wire.read()); //read reg 6 [range 00-99]
  
} //end of getRTC()

bool RTC_detect() { //Detects DS3231 SQW falling edge 
  RTC_SQW_Prev = RTC_SQW_Current;
  RTC_SQW_Current = digitalRead(RTC_SQW_Pin);
  return (RTC_SQW_Prev == HIGH && RTC_SQW_Current == LOW);
} //end of detect_RTC

void displayRTCDate() {
  getRTC();
  displayRTC_timeOnMax(ddRTC,moRTC,yyRTC);
} //end displayRTCDate

void displayRTC() { //updates display if new RTC time. Detects DS3231 SQW falling edge then trigger display of time update
  
  RTC_SQW_Prev = RTC_SQW_Current;
  RTC_SQW_Current = digitalRead(RTC_SQW_Pin);
      
  if (RTC_SQW_Prev == HIGH && RTC_SQW_Current == LOW) { //test for falling edge
    getRTC(); // updates ssRTC, mmRTC, hhRTC, dowRTC, ddRTC, moRTC, ctyRTC, yyRTC;
    if (UTC_offset_enable == false) {  // flag requests UTC time
      displayRTC_timeOnMax(hhRTC,mmRTC,ssRTC);
    } //end if
    else {                             // flag requests local offset time
      getLocalTime(yyRTC,moRTC,ddRTC,hhRTC,mmRTC); 
      displayRTC_timeOnMax(offHH,offMM,ssRTC);
    } //end else
  } //end if
} // end of displayRTC()

void displayRTC_now() { //immediate retrieve and display of RTC time registers
  getRTC(); // updates ssRTC, mmRTC, hhRTC, dowRTC, ddRTC, moRTC, ctyRTC, yyRTC;
  displayRTC_timeOnMax(hhRTC,mmRTC,ssRTC);
} // end of displayRTC()

void sendRTC(byte reg_addr, byte byte_data) {
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(reg_addr);   //set register pointer to address on DS3231
  Wire.write(byte_data);
  Wire.endTransmission();
} //end of sendRTC()

byte getSingleRTC(byte reg_addr) {    //returns a single raw byte from the provided address register
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(reg_addr);   //set to reg address on DS3231
  Wire.endTransmission();
  Wire.requestFrom(RTC_I2C_ADDRESS, 1);
  return (Wire.read());
} //end of getRTC_BCD()

void setRTC_Time(byte hh, byte mm, byte ss) { //must be [0-99]
  //  example use: setRTC_Time(23,49,50);     //hh,mm,ss

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(0x00);   //set register pointer to address on DS3231
  Wire.write(dec2bcd(ss)); //set seconds
  Wire.write(dec2bcd(mm)); //set minutes
  Wire.write(dec2bcd(hh)); //set hours. Bit 6 low keeps at 24hr mode. So can leave as is.  
  Wire.endTransmission();
} //end of setRTC_Time()

void setRTC_Date(int yyyy, byte mo, byte dd) { 
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(0x04);   //set register pointer to address on DS3231
  Wire.write(dec2bcd(dd)); //set date. sending the last 2 digits only.
  Wire.write(dec2bcd(mo)); //set month. ignore century as don't have any use for that.
  Wire.write(dec2bcd(yyyy % 100)); //set year. sending the 2 LS digits only.  
  Wire.endTransmission();
} //end of setRTC_Time()

//****  STATE MACHINE **** //

void RunStateMachine() {
  byte temp_buffer;
  
  switch (StateMachine) {

    case DEBUG: //remember, it's looping!

        if (RTC_detect()) {
          t1=micros();
        }
        if (PPS_detect()) {
          t2=micros();
        }
        break; //end DEBUG case

        
 // ------------------------------    

    case BOOTUP:
        sendRTC(0x0E,0x00); // enables the 1Hz pulse on RTC DS3231's SQW pin
        delay(50);
        temp_buffer = getSingleRTC(0x02); //get hour byte from addr 0x02. Bit 6: LOW (0) = 24 hr mode. HIGH (1) = 12 hr.
        if ((temp_buffer & 0b01000000) != 0) {   //if Bit 6 is HIGH, i.e., if 24 hour time is *not* set, then...
           getRTC(); // grab time 
           if (mmRTC > 58 && ssRTC > 58) { // checking to make sure not near an hours rollover. 
              break;  //keep breaking until we roll over the seconds
           } //end if
           else {
            sendRTC(0x02,temp_buffer ^ 0b01000000); //set BIT 6 low to enable 24 hr time
           } //end else
        } //end if

   //StateMachine = DEBUG; // >>>> State Change! <<<<//
    StateMachine = REG_OPS; // >>>> State Change! <<<<//
        t0=micros();
        break;
     //end BOOTUP case

 // ------------------------------    
        
    case REG_OPS:
        displayRTC();  //keep the trains running
        if (mmRTC == 15 && ssRTC == 0) {   //every hour at minute 5, do a GPS_INIT check to prep for GPS>RTC update
          GPS_INIT_t0 = millis(); //sets our timer to allow a timeout in the next state
          StateMachine = GPS_INIT; // >>>> State Change! <<<<//
          break;
        } //end if
        if (pulseFlag == 1) {
          detachInterrupt(digitalPinToInterrupt(ir_pin)); //stop interrupt while we process
          SonyIR_analyzer();
          attachInterrupt(digitalPinToInterrupt(ir_pin), ISR_pulse_detected, CHANGE);
        } // end if
        break;
        //end REG_OPS case

 // ------------------------------    

    case TOGGLE_DISPLAY:  //holds the display before resuming regular ops
        if (millis() - t1 < 3500) {
           break;
        }
        else {
           StateMachine = REG_OPS;
           break;
        }
          
        //end TOGGLE_DISPLAY case

 // ------------------------------   
        
    case GPS_INIT:
        displayRTC();  // keeps the trains running on the display!
     
       if (PPS_detect()) {  //means PPS is detected on the GPS unit
         StateMachine = GPS_PPS_SYNC; // >>>> State Change! <<<<//
         clearSerialInputBuffer(); //purges old GPS data in serial buffer 
         break;
       } //end if
        
        if (millis() - GPS_INIT_t0 > 4000) {  //timeout this state after 4 secs if no PPS detected
          StateMachine = GPS_NMEA_SYNC; // >>>> State Change! <<<<//
          clearSerialInputBuffer(); //purges old GPS data in serial buffer
          break; 
        } //end if

        break;
        //end GPS_INIT case

 // ------------------------------   
    case GPS_PPS_SYNC: 
         
         byte ssGPS_incr;

         displayRTC(); //keep the trains running while reading serial input!
       
         if (newGPS_timeAvail && !GPS_sec_primed) { //here we prime the GPS seconds by incrementing 1s as we wait for a PPS signal
            ssGPS_incr = (ssGPS + 1) % 60;
            GPS_sec_primed = true;
            newGPS_timeAvail = false; // set this false to force another read post PPS. 
          } //end if

        if (PPS_detect() && GPS_sec_primed) {  // if PPS detected send only the primed second to RTC
           sendRTC(0x00,dec2bcd(ssGPS_incr));  // here is where we would add delay for insanity mode
           PPS_done = true;
        } //end if
        
        if (PPS_done && GPS_sec_primed && newGPS_timeAvail) { // send minutes and hours of next NMEA time read after PPS
          sendRTC(0x01,dec2bcd(mmGPS));
          sendRTC(0x02,dec2bcd(hhGPS));
          GPS_sec_primed = false;
          newGPS_timeAvail = false;
        } //end if
        
        if (newGPS_dateAvail && PPS_done) {
          setRTC_Date(yyyyGPS,moGPS,ddGPS);
          newGPS_dateAvail = false; //this completed the date and time update.
          PPS_done = false;
          StateMachine = REG_OPS; // >>>> State Change! <<<<//
          break;
        } //end if

        //need a timeout where after 30 seconds of no updates it goes back to REG_OPS
        
        if (NMEA_processFlag) {
           processNMEA();
        } //end if
        else {
          if (Serial.available() > 0) {
            if (Serial.read() == '$') {  //start of NMEA message
                NMEA_processFlag = true;  //we only want to start processing at beginning of message
                GGA_msg = false;
                RMC_msg = false;
                ZDA_msg = false;
                byteIndex = 1;
            } //end if
          } //end if
        } //end else

        if (millis() - GPS_INIT_t0 > 14000) {  //timeout this state after 10 secs if no PPS detected
            StateMachine = REG_OPS; // >>>> State Change! <<<<//
        }
       
        break;
        
  //end GPS_PPS_SYNC case

 // ------------------------------    

    case GPS_NMEA_SYNC:
    
        displayRTC(); //keep the trains running while reading serial input!
        
        if (newGPS_timeAvail) {
          setRTC_Time(hhGPS,mmGPS,ssGPS);
          newGPS_timeAvail = false;
        } //end if

        if (newGPS_dateAvail) {
          setRTC_Date(yyyyGPS,moGPS,ddGPS);
          newGPS_dateAvail = false; //this completed the date and time update.
          StateMachine = REG_OPS; // >>>> State Change! <<<<//
          break;
        } //end if

        if (NMEA_processFlag) {
           processNMEA();
        } //end if
        else {
          if (Serial.available() > 0) {
            if (Serial.read() == '$') {  //start of NMEA message
                NMEA_processFlag = true;
                GGA_msg = false;
                RMC_msg = false;
                ZDA_msg = false;
                byteIndex = 1;
            } //end if
          } //end if
        } //end else

        if (millis() - GPS_INIT_t0 > 14000) {  //timeout this state after 10 secs if no PPS detected
            StateMachine = REG_OPS; // >>>> State Change! <<<<//
        }
        
        break;
        //end GPS_NMEA_SYNC case
     
  } //end switch StateMachine
} //end of RunStateMachine()


void setup() {
  // put your setup code here, to run once:

  delay(1000); //give system time to stabilize

  //start up SPI for Max7219
      SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); //per Max7219 Datasheet, 10MHz, MSB
      pinMode(ChipSelectPin, OUTPUT);  //sets the output pin for SS/CS
      digitalWrite(ChipSelectPin,HIGH); // take the CS/SS pin high to deselect the chip
      SPI.begin(); //initialize the SPI bus using our settings from above
      
  initializeMax7219();  //sets all startup parameters for Max7219 chip

  delay(100);
  
  pinMode(RTC_SQW_Pin, INPUT_PULLUP); // SQW is open drain on DS3231, so need internal pullup enabled.
  pinMode(GPS_PPS_Pin, INPUT);
  pinMode(ir_pin, INPUT_PULLUP);
  
  Wire.setClock(400000);  //i2C 100kHz typical. 400kHz fast mode. DS3231 RTC supports fast mode.
  delay(100); //more stabilizing
  
  Serial.begin(9600);  // for the GPS unit. Later set to Serial when debugging done.
  delay(100); //more stabilizing
  
  attachInterrupt (digitalPinToInterrupt(ir_pin), ISR_pulse_detected, CHANGE); 
  
  StateMachine = BOOTUP;  //set the initial state
}

void loop() {
  // put your main code here, to run repeatedly:

  RunStateMachine();

}
