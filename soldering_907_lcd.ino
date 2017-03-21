#include <LiquidCrystal.h>
#include <TimerOne.h>
#include <EEPROM.h>


// The LCD 0802 parallel interface
const byte LCD_RS_PIN     = 13;
const byte LCD_E_PIN      = 12;
const byte LCD_DB4_PIN    = 5;
const byte LCD_DB5_PIN    = 6;
const byte LCD_DB6_PIN    = 7;
const byte LCD_DB7_PIN    = 8;

// Rotary encoder interface
const byte R_MAIN_PIN = 2;                      // Rotary Encoder main pin (right)
const byte R_SECD_PIN = 4;                      // Rotary Encoder second pin (left)
const byte R_BUTN_PIN = 3;                      // Rotary Encoder push button pin

const byte probePIN  = A0;                      // Thermometer pin from soldering iron
const byte heaterPIN = 10;                      // The soldering iron heater pin
const byte buzzerPIN = 11;                      // The simple buzzer to make a noise

const uint16_t temp_minC = 180;                 // Minimum temperature in degrees of celsius
const uint16_t temp_maxC = 400;                 // Maximum temperature in degrees of celsius
const uint16_t temp_minF = (temp_minC *9 + 32*5 + 2)/5;
const uint16_t temp_maxF = (temp_maxC *9 + 32*5 + 2)/5;

//------------------------------------------ class FastPWDdac --------------------------------------------------
/*
FastPWMdac
Copyright (C) 2015  Albert van Dalen http://www.avdweb.nl
*/

class FastPWMdac {
  public:
    void init(byte _timer1PWMpin, byte resolution);
    void analogWrite8bit(byte value8bit);
    void analogWrite10bit(int value10bit);
  private:
    byte timer1PWMpin;
};


void FastPWMdac::init(byte _timer1PWMpin, byte resolution){
  timer1PWMpin = _timer1PWMpin;
  if(resolution == 8) Timer1.initialize(32);
  if(resolution == 10) Timer1.initialize(128);
  Timer1.pwm(timer1PWMpin, 0);                  // dummy, required before setPwmDuty()
}

void FastPWMdac::analogWrite8bit(byte value8bit){
  Timer1.setPwmDuty(timer1PWMpin, value8bit*4); // faster than pwm()
}

void FastPWMdac::analogWrite10bit(int value10bit) {
  Timer1.setPwmDuty(timer1PWMpin, value10bit); // faster than pwm()
}

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
  uint32_t ID                           each time increment by 1
  struct cfg                            config data, 8 bytes
  byte CRC                              the checksum
*/
struct cfg {
  uint16_t temp_min;                            // The minimum temperature (180 centegrees)
  uint16_t temp_max;                            // The temperature for 400 centegrees
  uint16_t temp;                                // The temperature of the iron to be start
  byte     off_timeout;                         // The Automatic switch-off timeout in minutes [0 - 30]
  bool     celsius;                             // Temperature units: true - celsius, false - farenheit
};

class CONFIG {
  public:
    CONFIG() {
      can_write = is_valid = false;
      buffRecords = 0;
      rAddr = wAddr = 0;
      eLength = 0;
      nextRecID = 0;
    }
    void init();
    bool load(void);
    bool isValid(void)       { return is_valid; }
    uint16_t temp(void)      { return Config.temp; }
    byte getOffTimeout(void) { return Config.off_timeout; }
    bool getTempUnits(void)  { return Config.celsius; }
    bool saveTemp(uint16_t t);
    void saveConfig(byte off, bool cels);
    void saveCalibrationData(uint16_t t_max, uint16_t t_min);
    void getCalibrationData(uint16_t& t_max, uint16_t& t_min);
    void setDefaults(bool Write = false);
  private:
    struct cfg Config;
    bool readRecord(uint16_t addr, uint32_t &recID);
    bool save(void);
    bool can_write;                             // Tha flag indicates that data can be saved
    bool is_valid;                              // Whether tha data was loaded
    byte buffRecords;                           // Number of the records in the outpt buffer
    uint16_t rAddr;                             // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                             // Address in the EEPROM to start write new record
    uint16_t eLength;                           // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                         // next record ID
    const byte record_size = 16;                // The size of one record in bytes
    const uint16_t def_min = 554;               // Default minimum temperature
    const uint16_t def_max = 916;               // Default maximun temperature
    const uint16_t def_set = 629;               // Default setup temperature
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  eLength = EEPROM.length();
  byte t, p ,h;
  uint32_t recID;
  uint32_t minRecID = 0xffffffff;
  uint16_t minRecAddr = 0;
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;
  byte records = 0;

  setDefaults();
  nextRecID = 0;

  // read all the records in the EEPROM find min and max record ID
  for (uint16_t addr = 0; addr < eLength; addr += record_size) {
    if (readRecord(addr, recID)) {
      ++records;
      if (minRecID > recID) {
        minRecID = recID;
        minRecAddr = addr;
      }
      if (maxRecID < recID) {
        maxRecID = recID;
        maxRecAddr = addr;
      }
    } else {
      break;
    }
  }

  if (records == 0) {
    wAddr = rAddr = 0;
    can_write = true;
    return;
  }

  rAddr = maxRecAddr;
  if (records < (eLength / record_size)) {      // The EEPROM is not full
    wAddr = rAddr + record_size;
    if (wAddr > eLength) wAddr = 0;
  } else {
    wAddr = minRecAddr;
  }
  can_write = true;
}

bool CONFIG::saveTemp(uint16_t t) {
  if (t == Config.temp) return true;
  Config.temp = t;
  return save();  
}

void CONFIG::saveConfig(byte off, bool cels) {
  if (off > 30) off = 0;
  Config.off_timeout = off;
  Config.celsius = cels;
  save();                                       // Save new data into the EEPROM
}

void CONFIG::saveCalibrationData(uint16_t t_max, uint16_t t_min) {
  Config.temp_max  = t_max;
  Config.temp_min  = t_min;
  save();
}

void CONFIG::getCalibrationData(uint16_t& t_max, uint16_t& t_min) {
  t_max = Config.temp_max;
  t_min = Config.temp_min;
}

bool CONFIG::save(void) {
  if (!can_write) return can_write;
  if (nextRecID == 0) nextRecID = 1;

  uint16_t startWrite = wAddr;
  uint32_t nxt = nextRecID;
  byte summ = 0;
  for (byte i = 0; i < 4; ++i) {
    EEPROM.write(startWrite++, nxt & 0xff);
    summ <<=2; summ += nxt;
    nxt >>= 8;
  }
  byte* p = (byte *)&Config;
  for (byte i = 0; i < sizeof(struct cfg); ++i) {
    summ <<= 2; summ += p[i];
    EEPROM.write(startWrite++, p[i]);
  }
  summ ++;                                      // To avoid empty records
  EEPROM.write(wAddr+record_size-1, summ);

  rAddr = wAddr;
  wAddr += record_size;
  if (wAddr > EEPROM.length()) wAddr = 0;
  return true;
}

bool CONFIG::load(void) {

  is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
  if (is_valid) {
    if (Config.temp_min >= Config.temp_max) {
      setDefaults();
    }
    if ((Config.temp > Config.temp_max) || (Config.temp < Config.temp_min)) Config.temp = def_set;
    if ((Config.off_timeout > 30) ) Config.off_timeout = 0;
  }
  return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
  byte Buff[16];

  for (byte i = 0; i < 16; ++i) 
    Buff[i] = EEPROM.read(addr+i);
  
  byte summ = 0;
  for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {

    summ <<= 2; summ += Buff[i];
  }
  summ ++;                                      // To avoid empty fields
  if (summ == Buff[15]) {                       // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[i];
    }
    recID = ts;
    byte i = 4;
    memcpy(&Config, &Buff[4], sizeof(struct cfg));
    return true;
  }
  return false;
}

void CONFIG::setDefaults(bool Write) {          // Restore default values
  Config.temp        = def_set;
  Config.temp_min    = def_min;
  Config.temp_max    = def_max;
  Config.off_timeout = 0;                       // Default autometic switch-off timeout (disabled)
  Config.celsius     = true;                    // Default use celsius
  if (Write) {
    save();
  }
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
    BUZZER(byte buzzerP)  { buzzer_pin = buzzerP; }
    void init(void);
    void shortBeep(void)  { tone(buzzerPIN, 3520, 160); }
    void lowBeep(void)    { tone(buzzerPIN,  880, 160); }
    void doubleBeep(void) { tone(buzzerPIN, 3520, 160); delay(300); tone(buzzerPIN, 3520, 160); }
    void failedBeep(void) { tone(buzzerPIN, 3520, 160); delay(170);
                            tone(buzzerPIN,  880, 250); delay(260);
                            tone(buzzerPIN, 3520, 160);
                          }
  private:
    byte buzzer_pin;
};

void BUZZER::init(void) {
  pinMode(buzzer_pin, OUTPUT);
  noTone(buzzer_pin);
}

//------------------------------------------ class BUTTON ------------------------------------------------------
class BUTTON {
  public:
    BUTTON(byte ButtonPIN, unsigned int timeout_ms = 3000) {
      pt = tickTime = 0;
      buttonPIN = ButtonPIN;
      overPress = timeout_ms;
    }
    void init(void) { pinMode(buttonPIN, INPUT_PULLUP); }
    void setTimeout(uint16_t timeout_ms = 3000) { overPress = timeout_ms; }
    byte intButtonStatus(void) { byte m = mode; mode = 0; return m; }
    void cnangeINTR(void);
    byte buttonCheck(void);
    bool buttonTick(void);
  private:
    volatile byte     mode;                     // The button mode: 0 - not pressed, 1 - pressed, 2 - long pressed
    uint16_t          overPress;                // Maxumum time in ms the button can be pressed
    volatile uint32_t pt;                       // Time in ms when the button was pressed (press time)
    uint32_t          tickTime;                 // The time in ms when the button Tick was set
    byte              buttonPIN;                // The pin number connected to the button
    const uint16_t    tickTimeout = 200;        // Period of button tick, while tha button is pressed 
    const uint16_t    shortPress = 900;         // If the button was pressed less that this timeout, we assume the short button press
};

void BUTTON::cnangeINTR(void) {                 // Interrupt function, called when the button status changed
  
  bool keyUp = digitalRead(buttonPIN);
  unsigned long now_t = millis();
  if (!keyUp) {                                 // The button has been pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t; 
  } else {
    if (pt > 0) {
      if ((now_t - pt) < shortPress) mode = 1;  // short press
        else mode = 2;                          // long press
      pt = 0;
    }
  }
}

byte BUTTON::buttonCheck(void) {                // Check the button state, called each time in the main loop

  mode = 0;
  bool keyUp = digitalRead(buttonPIN);          // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp) {                                 // The button is pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t;
  } else {
    if (pt == 0) return 0;
    if ((now_t - pt) > shortPress)              // Long press
      mode = 2;
    else
      mode = 1;
    pt = 0;
  } 
  return mode;
}

bool BUTTON::buttonTick(void) {                 // When the button pressed for a while, generate periodical ticks

  bool keyUp = digitalRead(buttonPIN);          // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp && (now_t - pt > shortPress)) {    // The button have been pressed for a while
    if (now_t - tickTime > tickTimeout) {
       tickTime = now_t;
       return (pt != 0);
    }
  } else {
    if (pt == 0) return false;
    tickTime = 0;
  } 
  return false;
}

//------------------------------------------ class ENCODER ------------------------------------------------------
class ENCODER {
  public:
    ENCODER(byte aPIN, byte bPIN, int16_t initPos = 0) {
      pt = 0; mPIN = aPIN; sPIN = bPIN; pos = initPos;
      min_pos = -32767; max_pos = 32766; channelB = false; increment = 1;
      changed = 0;
      is_looped = false;
    }
    void init(void) {
      pinMode(mPIN, INPUT_PULLUP);
      pinMode(sPIN, INPUT_PULLUP);
    }
    void    set_increment(byte inc)             { increment = inc; }
    byte    get_increment(void)                 { return increment; }
    int16_t read(void)                          { return pos; }
    void    reset(int16_t initPos, int16_t low, int16_t upp, byte inc = 1, byte fast_inc = 0, bool looped = false);
    bool    write(int16_t initPos);
    void    cnangeINTR(void);
  private:
    int32_t           min_pos, max_pos;
    volatile uint32_t pt;                       // Time in ms when the encoder was rotaded
    volatile uint32_t changed;                  // Time in ms when the value was changed
    volatile bool     channelB;
    volatile int16_t  pos;                      // Encoder current position
    byte              mPIN, sPIN;               // The pin numbers connected to the main channel and to the socondary channel
    bool              is_looped;                // Whether the encoder is looped
    byte              increment;                // The value to add or substract for each encoder tick
    byte              fast_increment;           // The value to change encoder when in runs quickly
    const uint16_t    fast_timeout = 300;       // Time in ms to change encodeq quickly
    const uint16_t    overPress = 1000;
};

bool ENCODER::write(int16_t initPos) {
  if ((initPos >= min_pos) && (initPos <= max_pos)) {
    pos = initPos;
    return true;
  }
  return false;
}

void ENCODER::reset(int16_t initPos, int16_t low, int16_t upp, byte inc, byte fast_inc, bool looped) {
  min_pos = low; max_pos = upp;
  if (!write(initPos)) initPos = min_pos;
  increment = fast_increment = inc;
  if (fast_inc > increment) fast_increment = fast_inc;
  is_looped = looped;
}

void ENCODER::cnangeINTR(void) {                // Interrupt function, called when the channel A of encoder changed
  
  bool rUp = digitalRead(mPIN);
  unsigned long now_t = millis();
  if (!rUp) {                                   // The channel A has been "pressed"
    if ((pt == 0) || (now_t - pt > overPress)) {
      pt = now_t;
      channelB = digitalRead(sPIN);
    }
  } else {
    if (pt > 0) {
      byte inc = increment;
      if ((now_t - pt) < overPress) {
        if ((now_t - changed) < fast_timeout) inc = fast_increment;
        changed = now_t;
        if (channelB) pos -= inc; else pos += inc;
        if (pos > max_pos) { 
          if (is_looped)
            pos = min_pos;
          else 
            pos = max_pos;
        }
        if (pos < min_pos) {
          if (is_looped)
            pos = max_pos;
          else
            pos = min_pos;
        }
      }
      pt = 0; 
    }
  }
}

//------------------------------------------ class lcd DSPLay for soldering iron -----------------------------
class DSPL : protected LiquidCrystal {
  public:
    DSPL(byte RS, byte E, byte DB4, byte DB5, byte DB6, byte DB7) : LiquidCrystal(RS, E, DB4, DB5, DB6, DB7) { }
    void init(void);
    void clear(void) { LiquidCrystal::clear(); }
    void tSet(uint16_t t, bool celsuis);        // Show the temperature set
    void tCurr(uint16_t t);                     // Show The current temperature
    void pSet(byte p);                          // Show the power set
    void tempLim(byte indx, uint16_t temp);     // Show the upper or lower temperature limit
    void timeToOff(byte sec);                   // Show the time to automatic off the iron
    void msgNoIron(void);                       // Show 'No iron' message
    void msgReady(void);                        // Show 'Ready' message
    void msgWorking(void);                      // Show 'Working' message
    void msgOn(void);                           // Show 'On' message
	  void msgOff(void);                          // Show 'Off' message
	  void msgCold(void);                         // Show 'Cold' message
	  void msgFail(void);                         // Show 'Fail' message
	  void msgTune(void);                         // Show 'Tune' message
	  void msgCelsius(void);                      // Show 'Cels.' message
	  void msgFarneheit(void);                    // Show 'Faren.' message
    void msgUpper(void);                        // Show 'setting upper temperature' process
    void msgLower(void);                        // Show 'setting lower temperature' process
    void msgDefault();                          // Show 'default' message (load default configuratuin)
    void msgCancel(void);                       // Show 'cancel' message
    void msgApply(void);                        // Show 'save message'
    void setupMode(byte mode, byte p = 0);      // Show the configureation mode [0 - 2]
    void show(void) {}                          // No animation ever
    void percent(byte Power);                   // Show the percentage
  private:
    bool full_second_line;                      // Whether the second line is full with the message
};

void DSPL::init(void) {
  LiquidCrystal::begin(8, 2);
  LiquidCrystal::clear();
  full_second_line = false;
}

void DSPL::tSet(uint16_t t, bool celsius) {
  char buff[5];
  char units = 'C';
  if (!celsius) units = 'F';
  LiquidCrystal::setCursor(0, 0);
  sprintf(buff, "%3d%c", t, units);
  LiquidCrystal::print(buff);
}

void DSPL::tCurr(uint16_t t) {
  char buff[4];
  LiquidCrystal::setCursor(0, 1);
  if (t < 1000) {
    sprintf(buff, "%3d", t);
  } else {
    LiquidCrystal::print(F("xxx"));
    return;
  }
  LiquidCrystal::print(buff);
  if (full_second_line) {
    LiquidCrystal::print(F("    "));
    full_second_line = false;
  }
}

void DSPL::pSet(byte p) {
  char buff[6];
  sprintf(buff, "P:%3d", p);
  LiquidCrystal::setCursor(0, 0);
  LiquidCrystal::print(buff);
}

void DSPL::tempLim(byte indx, uint16_t temp) {
  char buff[9];
  if (indx == 0) {
    buff[0] = 'u';
    buff[1] = 'p';
  } else {
    buff[0] = 'l';
    buff[1] = 'o';
  }
  sprintf(&buff[2], ": %3d ", temp);
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(buff);
}

void DSPL::timeToOff(byte sec) {
  char buff[5];
  if (sec >= 1000) return;
  sprintf(buff, " %3d", sec);
  LiquidCrystal::setCursor(4, 0);
  LiquidCrystal::print(buff);
}

void DSPL::msgNoIron(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F("no iron "));
  full_second_line = true;
}

void DSPL::msgReady(void) {
  LiquidCrystal::setCursor(4, 0);
  LiquidCrystal::print(F(" rdy"));
}

void DSPL::msgWorking(void) {
  LiquidCrystal::setCursor(4, 0);
  LiquidCrystal::print(F(" wrk"));
}

void DSPL::msgOn(void) {
  LiquidCrystal::setCursor(4, 0);
  LiquidCrystal::print(F("  ON"));
}

void DSPL::msgOff(void) {
  LiquidCrystal::setCursor(4, 0);
  LiquidCrystal::print(F(" OFF"));
}

void DSPL::msgCold(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F("  cold  "));
  full_second_line = true;
}

void DSPL::msgFail(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" Failed "));
}

void DSPL::msgTune(void) {
  LiquidCrystal::setCursor(0, 0);
  LiquidCrystal::print(F("Tune"));
}

void DSPL::msgCelsius(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F("Celsius "));
}

void DSPL::msgFarneheit(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F("Faren.  "));
}

void DSPL::msgUpper(void) {
  LiquidCrystal::setCursor(6, 0);
  LiquidCrystal::print(F("up"));
}

void DSPL::msgLower(void) {
  LiquidCrystal::setCursor(6, 0);
  LiquidCrystal::print(F("lo"));
}

void DSPL::msgDefault() {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" default"));
}

void DSPL::msgCancel(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" cancel "));
}

void DSPL::msgApply(void) {
  LiquidCrystal::setCursor(0, 1);
  LiquidCrystal::print(F(" save   "));
}

void DSPL::setupMode(byte mode, byte p) {
  char buff[5];
  LiquidCrystal::clear();
  LiquidCrystal::print(F("setup"));
  LiquidCrystal::setCursor(1,1);
  switch (mode) {
    case 0:
	  LiquidCrystal::print(F("off:"));
    if (p > 0) {
      sprintf(buff, "%2dm", p);
      LiquidCrystal::print(buff);
    } else {
      LiquidCrystal::print(" NO");
    }
	  break;
    case 1:
      LiquidCrystal::print(F("units"));
      LiquidCrystal::setCursor(7,1);
      if (p)
        LiquidCrystal::print("C");
      else
        LiquidCrystal::print("F");
      break;
    case 2:
      LiquidCrystal::print(F("tune"));
      break;
  }
}

void DSPL::percent(byte Power) {
  char buff[6];
  sprintf(buff, " %3d%c", Power, '%');
  LiquidCrystal::setCursor(3, 1);
  LiquidCrystal::print(buff);
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
class HISTORY {
  public:
    HISTORY(void)                               { len = 0; }
    void     init(void)                         { len = 0; }
    bool     isFull(void)                       { return len == H_LENGTH; }
    uint16_t last(void);
    uint16_t top(void)                          { return queue[0]; }
    void     put(uint16_t item);                // Put new entry to the history
    uint16_t average(void);                     // calcilate the average value
    float    dispersion(void);                  // calculate the math dispersion
    float    gradient(void);                    // calculate the gradient of the history values
  private:
    uint16_t queue[H_LENGTH];
    byte len;                                   // The number of elements in the queue
    byte index;                                 // The current element position, use ring buffer
};

void HISTORY::put(uint16_t item) {
  if (len < H_LENGTH) {
    queue[len++] = item;
  } else {
    queue[index ] = item;
    if (++index >= H_LENGTH) index = 0;         // Use ring buffer
  }
}

uint16_t HISTORY::last(void) {
  byte i = H_LENGTH - 1;
  if (index)
    i = index - 1;
  return queue[i];
}

uint16_t HISTORY::average(void) {
  uint32_t sum = 0;
  if (len == 0) return 0;
  if (len == 1) return queue[0];
  for (byte i = 0; i < len; ++i) sum += queue[i];
  sum += len >> 1;                              // round the average
  sum /= len;
  return uint16_t(sum);
}

float HISTORY::dispersion(void) {
  if (len < 3) return 1000;
  uint32_t sum = 0;
  uint32_t avg = average();
  for (byte i = 0; i < len; ++i) {
    long q = queue[i];
    q -= avg;
    q *= q;
    sum += q;
  }
  sum += len << 1;
  float d = (float)sum / (float)len;
  return d;
}

// approfimating the history with the line (y = ax+b) using method of minimum square. Gradient is parameter a
float HISTORY::gradient(void) {
  if (len < 2) return 0;
  long sx, sx_sq, sxy, sy;
  sx = sx_sq = sxy = sy = 0;
  for (byte i = 1; i <= len; ++i) {
    sx    += i;
    sx_sq += i*i;
    sxy   += i*queue[i-1];
    sy    += queue[i-1];
  }
  long numerator   = len * sxy - sx * sy;
  long denominator = len * sx_sq - sx * sx;
  float a = (float)numerator / (float)denominator;
  return a;
}

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algoritm 
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formulae is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 */
class PID {
  public:
    PID(void) {
      Kp = 768;
      Ki =  30;
      Kd = 196;
    }
    void resetPID(int temp = -1);               // reset PID algoritm history parameters
    // Calculate the power to be applied
    int reqPower(int temp_set, int temp_curr);
    int changePID(byte p, int k);
  private:
    void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
    int   temp_h0, temp_h1;                     // previously measured temperature
    int   temp_diff_iterate;                    // The temperature difference to start iterate process
    bool  pid_iterate;                          // Whether the iterative process is used
    long  i_summ;                               // Ki summary multiplied by denominator
    long  power;                                // The power iterative multiplied by denominator
    long  Kp, Ki, Kd;                           // The PID algorithm coefficients multiplied by denominator
    const byte denominator_p = 8;               // The common coefficeient denominator power of 2 (8 means divide by 256)
};

void PID::resetPID(int temp) {
  temp_h0 = 0;
  power  = 0;
  i_summ = 0;
  pid_iterate = false;
  temp_diff_iterate = 30;//(temp + 5) / 10;
  if (temp_diff_iterate < 30) temp_diff_iterate = 30;
  if ((temp > 0) && (temp < 1000))
    temp_h1 = temp;
  else
    temp_h1 = 0;
}

int PID::changePID(byte p, int k) {
  switch(p) {
    case 1:
      if (k >= 0) Kp = k;
      return Kp;
    case 2:
      if (k >= 0) Ki = k;
      return Ki;
    case 3:
      if (k >= 0) Kd = k;
      return Kd;
    default:
      break;
  }
  return 0;
}

int PID::reqPower(int temp_set, int temp_curr) {
  if (temp_h0 == 0) {
    // When the temperature is near the preset one, reset the PID and prepare iterative formulae                        
    if ((temp_set - temp_curr) < temp_diff_iterate) {
      if (!pid_iterate) {
        pid_iterate = true;
        power = 0;
        i_summ = 0;
      }
    }
    i_summ += temp_set - temp_curr;             // first, use the direct formulae, not the iterate process
    power = Kp*(temp_set - temp_curr) + Ki*i_summ;
    // If the temperature is near, prepare the PID iteration process
  } else {
    long kp = Kp * (temp_h1 - temp_curr);
    long ki = Ki * (temp_set - temp_curr);
    long kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
    long delta_p = kp + ki + kd;
    //debugPID(temp_set, temp_curr, kp, ki, kd, delta_p);
    power += delta_p;                           // power keeped multiplied by denominator!
  }
  if (pid_iterate) temp_h0 = temp_h1;
  temp_h1 = temp_curr;
  long pwr = power + (1 << (denominator_p-1));  // prepare the power to delete by denominator, roud the result
  pwr >>= denominator_p;                        // delete by the denominator
  return int(pwr);
}

void PID::debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p) {
  Serial.print(t_set-t_curr); Serial.print(": ");
  Serial.print("[ "); Serial.print(temp_h0);
  Serial.print(", "); Serial.print(temp_h1);
  Serial.print(", "); Serial.print(t_curr);
  Serial.print(" ] kp = "); Serial.print(kp);
  Serial.print(", ki = "); Serial.print(ki);
  Serial.print(", kd = "); Serial.print(kd);
  Serial.print("; DP = "); Serial.println(delta_p);
}

//------------------------------------------ class soldering iron ---------------------------------------------
class IRON : protected PID {
  public:
    IRON(byte heater_pin, byte sensor_pin) {
      hPIN = heater_pin;
      sPIN = sensor_pin;
      on = false;
      unit_celsius = true;
      fix_power = false;
      unit_celsius = true;
      no_iron = true;
    }
    void     init(uint16_t t_max, uint16_t t_min);
    void     switchPower(bool On);
    bool     isOn(void)                         { return on; }
    bool     isCold(void)                       { return (h_temp.last() < temp_cold); }
    bool     noIron(void)                       { return no_iron; }
    void     setTempUnits(bool celsius)         { unit_celsius = celsius; }
    bool     getTempUnits(void)                 { return unit_celsius; }
    uint16_t getTemp(void)                      { return temp_set; }
    uint16_t getCurrTemp(void)                  { return h_temp.last(); }
    uint16_t tempAverage(void)                  { return h_temp.average(); }
    uint16_t tempDispersion(void)               { return h_temp.dispersion(); }
    uint16_t powerDispersion(void)              { return h_power.dispersion(); }
    byte     getMaxFixedPower(void)             { return max_fixed_power; }
    int      changePID(byte p, int k)           { return PID::changePID(p, k); }
    void     setTemp(int t);                    // Set the temperature to be keeped
    // Set the temperature to be keeped in human readable units (celsius or farenheit)
    void     setTempHumanUnits(int t);
    // Translate internal temperature to the celsius or farenheit
    uint16_t temp2humanUnits(uint16_t temp);
    byte     getAvgPower(void);                 // Average applied power
    byte     appliedPower(void);                // Power applied to the solder [0-100%]
    byte     hotPercent(void);                  // How hot is the iron (used in the idle state)
    void     keepTemp(void);                    // Main solder iron loop
    bool     fixPower(byte Power);              // Set the specified power to the the soldering iron
    bool     used(void);                        // Whether the iron was previously used (Hot)
  private:
    uint16_t   temp(void);                      // Read the actual temperature of the soldering iron
    void       applyPower(void);                // Check the the power limits and apply power to the heater
    FastPWMdac fastPWMdac;                      // Power the irom using fastPWMdac
    uint32_t   checkMS;                         // Milliseconds to measure the temperature next time
    byte       hPIN, sPIN;                      // The heater PIN and the sensor PIN
    int        power;                           // The soldering station power
    byte       actual_power;                    // The power supplied to the iron
    bool       on;                              // Whether the soldering iron is on
    bool       fix_power;                       // Whether the soldering iron is set the fix power
    bool       no_iron;                         // Whether the iron is connected
    bool       unit_celsius;                    // Human readable units for the temparature (celsius or farenheit)
    int        temp_set;                        // The temperature that should be keeped
    bool       iron_checked;                    // Whether the iron works
    int        temp_start;                      // The temperature when the solder was switched on
    uint32_t   elapsed_time;                    // The time elipsed from the start (ms)
    uint16_t   temp_min;                        // The minimum temperature (180 centegrees)
    uint16_t   temp_max;                        // The maximum temperature (400 centegrees)
    HISTORY    h_power;
    HISTORY    h_temp;
    const uint16_t temp_cold   = 340;           // The cold temperature to touch the iron safely
    const uint16_t temp_no_iron = 980;          // Sensor reading when the iron disconnected
    const byte max_power       = 180;           // maximum power to the iron (220)
    const byte max_fixed_power = 120;           // Maximum power in fiexed power mode
    const uint16_t period      = 200;           // The period to check the soldering iron temperature, ms
    const int check_time       = 10000;         // Time in ms to check Whether the solder is heating
    const int heat_expected    = 10;            // The iron should change the temperature at check_time
};

void IRON::setTemp(int t) {
  if (on) resetPID();
  temp_set = t;
}

void IRON::setTempHumanUnits(int t) {
  int temp;
  if (unit_celsius) {
    if (t < temp_minC) t = temp_minC;
    if (t > temp_maxC) t = temp_maxC;
    temp = map(t+1, temp_minC, temp_maxC, temp_min, temp_max);
  } else {
    if (t < temp_minF) t = temp_minF;
    if (t > temp_maxF) t = temp_maxF;
    temp = map(t+2, temp_minF, temp_maxF, temp_min, temp_max);
  }
  for (byte i = 0; i < 10; ++i) {
    int tH = temp2humanUnits(temp);
    if (tH <= t) break;
    --temp;
  }
  setTemp(temp);
}

uint16_t IRON::temp2humanUnits(uint16_t temp) {
  if (!unit_celsius)  return map(temp, temp_min, temp_max, temp_minF, temp_maxF);
  return map(temp, temp_min, temp_max, temp_minC, temp_maxC);  
}

byte IRON::getAvgPower(void) {
  int p = h_power.average();
  return p & 0xff;  
}

byte IRON::appliedPower(void) {
  byte p = getAvgPower(); 
  return map(p, 0, max_power, 0, 100);  
}

byte IRON::hotPercent(void) {
  uint16_t t = h_temp.average();
  char r = map(t, temp_cold, temp_set, 0, 100);
  if (r < 0) r = 0;
  return r;
}

void IRON::init(uint16_t t_max, uint16_t t_min) {
  pinMode(sPIN, INPUT);
  fastPWMdac.init(hPIN, 8);                     // initialization for 8 bit resolution
  fastPWMdac.analogWrite8bit(0);                // sawtooth output, period = 31.25Khz
  on = false;
  fix_power = false;
  power = 0;
  actual_power = 0;
  checkMS = 0;

  elapsed_time = 0;
  temp_start = analogRead(sPIN);
  iron_checked = false;
  temp_max = t_max; temp_min = t_min;

  resetPID();
  h_power.init();
  h_temp.init();
}

void IRON::switchPower(bool On) {
  on = On;
  if (!on) {
    fastPWMdac.analogWrite8bit(0);
    fix_power = false;
    return;
  }

  resetPID(analogRead(sPIN));
  h_power.init();
  checkMS = millis();
}

void IRON::keepTemp(void) {
  if (checkMS > millis()) return;
  checkMS = millis() + period;

  if (!on) {                                    // If the soldering iron is set to be switched off
    if (!fix_power)
      fastPWMdac.analogWrite8bit(0);            // Surely power off the iron
  }

  int16_t temp = 0;
  if (actual_power > 0)
    fastPWMdac.analogWrite8bit(0);              // switch-off supplied power to be more accurate
  delay(20);
  int16_t t1 = analogRead(sPIN);
  delayMicroseconds(50);
  int16_t t2 = analogRead(sPIN);
  if (actual_power > 0)                         // restore the power after measurement of the temperature
    fastPWMdac.analogWrite8bit(actual_power);

  if (abs(t1 - t2) < 10) {                      // use average of two samples if they are simiral
    t1 += t2 + 1;
    t1 >>= 1;                                   // average of two measurements
    temp = t1;
  } else {                                      // use sample that is near to the previous
    int tprev = h_temp.last();
    if (abs(t1 - tprev) < abs(t2 - tprev))
      temp = t1;
    else
      temp = t2;
  }
  
  // Check Whether the iron can be heated
  if (!iron_checked) {
    elapsed_time += period;
    if (elapsed_time >= check_time) {
      if ((abs(temp_set - temp) < 100) || ((temp - temp_start) > heat_expected)) {
        iron_checked = true;
      } else {
        switchPower(false);                     // Prevent the iron damage
        elapsed_time = 0;
        temp_start = analogRead(sPIN);
        iron_checked = false;
      }
    }
  }

  // If the power is off and no iron detected, do not put the temperature into the history
  if (!on && !fix_power && (temp > temp_no_iron)) {
    no_iron = true;
  } else {
    no_iron = false;
    h_temp.put(temp);
  }

  // Use PID algoritm to calculate power to be applied
  power = reqPower(temp_set, temp);
  applyPower();
}

void IRON::applyPower(void) {
  int p = power;
  if (p < 0) p = 0;
  if (p > max_power) p = max_power;

  if (h_temp.last() > (temp_set + 8)) p = 0;
  if (p == 0) actual_power = 0;
  if (on) actual_power = p & 0xff;
  h_power.put(p);
  fastPWMdac.analogWrite8bit(actual_power);
}

bool IRON::fixPower(byte Power) {
  if (Power == 0) {                             // To switch off the iron, set the power to 0
    fix_power = false;
    actual_power = 0;
    fastPWMdac.analogWrite8bit(0);
    return true;
  }

  if (Power > max_fixed_power) {
    actual_power = 0;
    return false;
  }

  if (!fix_power) {
    fix_power = true;
    power = Power;
    actual_power = power & 0xff;
  } else {
    if (power != Power) {
      power = Power;
      actual_power = power & 0xff;
    }
  }
  fastPWMdac.analogWrite8bit(actual_power);
  return true;
}

bool IRON::used(void) {
  uint16_t temp = h_temp.last();
  return ((temp > temp_cold) && (temp < temp_no_iron));
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
  public:
    SCREEN* next;                               // Pointer to the next screen
    SCREEN* nextL;                              // Pointer to the next Level screen, usually, setup
    SCREEN* main;                               // Pointer to the main screen
    SCREEN() {
      next = nextL = main = 0;
      force_redraw = true;
      scr_timeout = 0;
      time_to_return = 0;
    }
    virtual void    init(void)                  { }
    virtual void    show(void)                  { }
    virtual SCREEN* menu(void)                  {if (this->next != 0) return this->next; else return this; }
    virtual SCREEN* menu_long(void)             { if (this->nextL != 0) return this->nextL; else return this; }
    virtual void    rotaryValue(int16_t value)  { }
    virtual SCREEN* returnToMain(void);         // Return to the main screen in the menu tree
    bool            isSetup(void)               { return (scr_timeout != 0); }
    void            forceRedraw(void)           { force_redraw = true; }
    void            resetTimeout(void);         // Reset automatic return timeout
    void            setSCRtimeout(uint16_t t)   { scr_timeout = t; resetTimeout(); } 
    bool            wasRecentlyReset(void);     // Whether the return timeout was reset in the last 15 seconds
  protected:
    bool force_redraw;
    uint16_t scr_timeout;                       // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;                    // Time in ms to return to main screen
};

SCREEN* SCREEN::returnToMain(void) {
  if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
    scr_timeout = 0;
    return main;
  }
  return this;
}

void SCREEN::resetTimeout(void) {
  if (scr_timeout > 0)
    time_to_return = millis() + (uint32_t)scr_timeout*1000;
}

bool SCREEN::wasRecentlyReset(void) {
  uint32_t to = (time_to_return - millis()) / 1000;
  return((scr_timeout - to) < 15);
}

//---------------------------------------- class mainSCREEN [the soldering iron is OFF] ------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(IRON* Iron, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD = DSP;
      pEnc = ENC;
      pBz = Buzz;
      pCfg = Cfg;
      is_celsius = true;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON*    pIron;                             // Pointer to the iron instance
    DSPL*    pD;                                // Pointer to the DSPLay instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    BUZZER*  pBz;                               // Pointer to the simple buzzer instance
    CONFIG*  pCfg;                              // Pointer to the configuration instance
    uint32_t update_screen;                     // Time in ms to switch information on the display
    bool     used;                              // Whether the iron was used (was hot)
    bool     cool_notified;                     // Whether there was cold notification played
    bool     is_celsius;                        // The temperature units (Celsius or Farenheit)
	  const uint16_t period = 1000;               // The period to update the screen
};

void mainSCREEN::init(void) {
  pIron->switchPower(false);
  uint16_t temp_set = pIron->getTemp();
  is_celsius = pCfg->getTempUnits();
  pIron->setTempUnits(is_celsius);
  uint16_t tempH = pIron->temp2humanUnits(temp_set);
  if (is_celsius)
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->reset(tempH, temp_minF, temp_maxF, 1, 5);
  update_screen = 0;
  pD->clear();
  used = pIron->used();
  cool_notified = !used;
  if (used) {                                   // the iron was used, we should save new data in EEPROM
    pCfg->saveTemp(temp_set);
  }
}

void mainSCREEN::rotaryValue(int16_t value) {
  update_screen = millis() + period;
  pIron->setTempHumanUnits(value);
  pD->tSet(value, is_celsius);
}

void mainSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;

  uint16_t temp_set = pIron->getTemp();
  temp_set = pIron->temp2humanUnits(temp_set);
  pD->tSet(temp_set, is_celsius);
  pD->msgOff();
  
  if (pIron->noIron()) {                        // No iron connected
    pD->msgNoIron();
    return;
  }

  uint16_t temp = pIron->tempAverage();
  temp = pIron->temp2humanUnits(temp);
  if (pIron->isCold()) {
    if (used)
      pD->msgCold();
    else
      pD->tCurr(temp);
    if (!cool_notified) {
      pBz->lowBeep();
      cool_notified = true;
    }
  } else {
    pD->tCurr(temp);
  }
}

//---------------------------------------- class workSCREEN [the soldering iron is ON] -------------------------
class workSCREEN : public SCREEN {
  public:
    workSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD    = DSP;
      pBz   = Buzz;
      pEnc  = Enc;
      pCfg  = Cfg;
      ready = false;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* returnToMain(void);
  private:
    uint32_t update_screen;                     // Time in ms to update the screen
    IRON*    pIron;                             // Pointer to the iron instance
    DSPL*    pD;                                // Pointer to the DSPLay instance
    BUZZER*  pBz;                               // Pointer to the simple Buzzer instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    CONFIG*  pCfg;                              // Pointer to the configuration instance
    bool     ready;                             // Whether the iron is ready
    uint32_t  auto_off_notified;                // The time (in ms) when the automatic power-off was notified
    HISTORY  idle_power;                        // The power supplied to the iron when it is not used
	  const uint16_t period = 1000;               // The period to update the screen (ms)
};

void workSCREEN::init(void) {
  uint16_t temp_set = pIron->getTemp();
  bool is_celsius = pIron->getTempUnits();
  uint16_t tempH = pIron->temp2humanUnits(temp_set);
  if (is_celsius)
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
  else
    pEnc->reset(tempH, temp_minF, temp_maxF, 1, 5);
  pIron->switchPower(true);
  ready = false;
  pD->clear();
  pD->tSet(tempH, is_celsius);
  pD->msgOn();
  forceRedraw();
  uint16_t to = pCfg->getOffTimeout() * 60;
  this->setSCRtimeout(to);
  idle_power.init();
  auto_off_notified = 0;
}

void workSCREEN::rotaryValue(int16_t value) {
  ready = false;
  pD->msgOn();
  update_screen = millis() + period;
  pIron->setTempHumanUnits(value);
  pD->tSet(value, pIron->getTempUnits());
  idle_power.init();
  SCREEN::resetTimeout();
}

void workSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + period;

  int temp = pIron->tempAverage();
  int temp_set = pIron->getTemp();
  int tempH = pIron->temp2humanUnits(temp);
  pD->tCurr(tempH);
  byte p = pIron->appliedPower();
  pD->percent(p);

  uint16_t td = pIron->tempDispersion();
  uint16_t pd = pIron->powerDispersion();
  int ip      = idle_power.average();
  int ap      = pIron->getAvgPower();
  if ((temp <= temp_set) && (temp_set - temp <= 3) && (td <= 3) && (pd <= 4)) {
    idle_power.put(ap);
  }
  if (ap - ip >= 2) {    // The iron was used
    SCREEN::resetTimeout();
    if (ready) {
      idle_power.init();
      idle_power.put(ip+1);
    }
  }

  if ((abs(temp_set - temp) < 3) && (pIron->tempDispersion() <= 3))  {
    if (!ready) {
      idle_power.put(ap);
      pBz->shortBeep();
      pD->msgReady();
      ready = true;
    } else {
      if (ready && SCREEN::wasRecentlyReset()) {
        pD->msgWorking();
        auto_off_notified = 0;
      } else {
        uint32_t to = (time_to_return - millis()) / 1000;
        if ((scr_timeout > 0) && (to < 100)) {
          pD->timeToOff(to);
          if (!auto_off_notified || ((millis() - auto_off_notified) > 300000)) {
            pBz->shortBeep();
            auto_off_notified = millis();
          }
        } else
          pD->msgReady();
      }
    }
  }
}

SCREEN* workSCREEN::returnToMain(void) {
  if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
    scr_timeout = 0;
    pBz->doubleBeep();
    return main;
  }
  return this;
}

//---------------------------------------- class errorSCREEN [the soldering iron error detected] ---------------
class errorSCREEN : public SCREEN {
  public:
    errorSCREEN(DSPL* DSP, BUZZER* BZ) {
      pD  = DSP;
      pBz = BZ;
    }
    virtual void init(void) { pD->clear(); pD->msgFail(); pBz->failedBeep(); }
  private:
    DSPL*   pD;                                 // Pointer to the display instance
    BUZZER* pBz;                                // Pointer to the buzzer instance
};

//---------------------------------------- class powerSCREEN [fixed power to the iron] -------------------------
class powerSCREEN : public SCREEN {
  public:
    powerSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc) {
      pIron = Iron;
      pD = DSP;
      pEnc = Enc;
      on = false;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
  private:
    IRON* pIron;                                // Pointer to the iron instance
    DSPL* pD;                                   // Pointer to the DSPLay instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    uint32_t update_screen;                     // Time in ms to update the screen
    bool on;                                    // Whether the power of soldering iron is on
};

void powerSCREEN::init(void) {
  byte p = pIron->getAvgPower();
  byte max_power = pIron->getMaxFixedPower();
  pEnc->reset(p, 0, max_power, 1);
  on = true;                                    // Do start heating immediately
  pIron->switchPower(false);
  pIron->fixPower(p);
  pD->clear();
  pD->pSet(p);
}

void powerSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;

  uint16_t temp = pIron->tempAverage();
  temp = pIron->temp2humanUnits(temp);
  pD->tCurr(temp);
  update_screen = millis() + 500;
}

void powerSCREEN::rotaryValue(int16_t value) {
  pD->pSet(value);
  if (on)
    pIron->fixPower(value);
  update_screen = millis() + 1000;
}

SCREEN* powerSCREEN::menu(void) {
  on = !on;
  if (on) {
    uint16_t pos = pEnc->read();
    on = pIron->fixPower(pos);
	  pD->clear();
    pD->pSet(pos);
	  update_screen = 0;
  } else {
    pIron->fixPower(0);
	  pD->clear();
	  pD->pSet(0);
	  pD->msgOff();
  }
  return this;
}

SCREEN* powerSCREEN::menu_long(void) {
  pIron->fixPower(0);
  if (nextL) {
    pIron->switchPower(true);
    return nextL;
  }
  return this;
}

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
  public:
    configSCREEN(IRON* Iron, DSPL* DSP, ENCODER* Enc, CONFIG* Cfg) {
      pIron = Iron;
      pD = DSP;
      pEnc = Enc;
      pCfg = Cfg;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
  private:
    IRON* pIron;                                // Pointer to the iron instance
    DSPL* pD;                                   // Pointer to the DSPLay instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    CONFIG*  pCfg;                              // Pointer to the config instance
    uint32_t update_screen;                     // Time in ms to update the screen
    byte mode;                                  // Which parameter to change: 0 - off timeout, 1 - C/F, 2 - tuneSCREEN, 3 cancel
    bool tune;                                  // Whether the parameter is modifiying
    bool changed;                               // Whether some configuration parameter has been changed
    bool cels;                                  // Current celsius/farenheit;
    byte off_timeout;                           // Automatic switch-off timeout in minutes
};

void configSCREEN::init(void) {
  mode = 0;
  pEnc->reset(mode, 0, 4, 1, 0, true);          // 0 - off-timeout, 1 - C/F, 2 - tuneSCREEN, 3 - save, 4 - cancel
  tune        = false;
  changed     = false;
  cels        = pCfg->getTempUnits();
  off_timeout = pCfg->getOffTimeout();
  pD->clear();
  pD->setupMode(0);
  this->setSCRtimeout(30);
}

void configSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;
  force_redraw = false;
  update_screen = millis() + 10000;
  switch (mode) {
    case 0:
      pD->setupMode(mode, off_timeout);
      break;
    case 1:
      if (tune) {
        if (cels)
          pD->msgCelsius();
        else
          pD->msgFarneheit();
      } else {
        pD->setupMode(mode, cels);
      }
      break;
    case 2:
      pD->setupMode(mode, cels);
      break;
    case 3:
      pD->msgApply();
      break;
    case 4:
      pD->msgCancel();
      break;
    default:
      break;
  }
}

void configSCREEN::rotaryValue(int16_t value) {
  update_screen = millis() + 10000;
  if (tune) {                                   // tune the temperature units
    changed = true;
    switch (mode) {
      case 0:                                   // tuning the switch-off timeout
        if (value > 0) value += 2;              // The minimum timeout is 3 minutes
        off_timeout = value;
        break;
      case 1:                                   // tunung the temperature units
        cels = value;
        break;
      default:
        break;
    }
  } else {
    mode = value;
  }
  force_redraw = true;
}

SCREEN* configSCREEN::menu(void) {
  if (tune) {
    tune = false;
    pEnc->reset(mode, 0, 4, 1, 0, true);        // The value has been tuned, return to the menu list mode
  } else {
    int v = off_timeout;
    switch (mode) {
      case 0:                                   // automatic switch-off timeout
        if (v > 0) v -= 2;
        pEnc->reset(v, 0, 28, 1, 0, false);
        break;
      case 1:                                   // Celsius / Farenheit
        pEnc->reset(cels, 0, 1, 1, 0, true);
        break;
      case 2:                                   // Calibration
        if (next) return next;
        break;
      case 3:                                   // Save configuration data
        menu_long();
      case 4:                                   // Return to the main menu
        if (main) return main;
        return this;
    }
    tune = true;
  }
  force_redraw = true;
  return this;
}

SCREEN* configSCREEN::menu_long(void) {
  if (nextL) {
    if (changed) {
      pCfg->saveConfig(off_timeout, cels);
      pIron->setTempUnits(cels);
    }
    return nextL;
  }
  return this;
}

//---------------------------------------- class tuneSCREEN [tune the register and calibrating the iron] -------
class tuneSCREEN : public SCREEN {
  public:
    tuneSCREEN(IRON* Iron, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, CONFIG* Cfg) {
      update_screen = 0;
      pIron = Iron;
      pD = DSP;
      pEnc = ENC;
      pBz  = Buzz;
      pCfg = Cfg;
    }
    virtual void init(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON* pIron;                                // Pointer to the iron instance
    DSPL* pD;                                   // Pointer to the display instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    BUZZER* pBz;                                // Pointer to the simple Buzzer instance
    CONFIG* pCfg;                               // Pointer to the configuration class
    byte mode;                                  // Which temperature to tune [0-3]: select, up temp, low temp, defaults
    bool arm_beep;                              // Whether beep is armed
    byte max_power;                             // Maximum possible power to be applied
    uint32_t update_screen;                     // Time in ms to switch information on the display
    uint16_t tul[2];                            // upper & lower temp
    byte pul[2];                                // upper and lower power
};

void tuneSCREEN::init(void) {
  max_power = pIron->getMaxFixedPower();
  mode = 0;                                     // select the element from the list
  pul[0] = 75; pul[1] = 20;
  pEnc->reset(0, 0, 4, 1, 1, true);             // 0 - up temp, 1 - low temp, 2 - defaults, 3 - save, 4 - cancel
  update_screen = millis();
  arm_beep = false;
  tul[0] = tul[1] = 0;
  pD->clear();
  pD->msgTune();
  pD->tempLim(0, 0);
  forceRedraw();
}

void tuneSCREEN::rotaryValue(int16_t value) {
  if (mode == 0) {                              // No limit is selected, list the menu
    switch (value) {
      case 2:
        pD->msgDefault();
        break;
      case 3:
        pD->msgApply();
        break;
      case 4:
        pD->msgCancel();
        break;
      default:
       pD->tempLim(value, tul[value]);
       break;
    }
  } else {
    pIron->fixPower(value);
    force_redraw = true;
  }
  update_screen = millis() + 1000;
}

void tuneSCREEN::show(void) {
  if ((!force_redraw) && (millis() < update_screen)) return;

  force_redraw = false;
  update_screen = millis() + 1000;
  if (mode != 0) {                              // Selected upper or lower temperature
    int16_t temp = pIron->tempAverage();
    pD->tCurr(temp);
    byte power = pEnc->read();                  // applied power
    power = map(power, 0, max_power, 0, 100);
    pD->percent(power);
    if (mode == 1)
      pD->msgUpper();
    else
      pD->msgLower();
  }
  if (arm_beep && (pIron->tempDispersion() < 5)) {
    pBz->shortBeep();
    arm_beep = false;
  }
}
  
SCREEN* tuneSCREEN::menu(void) {                // The rotary button pressed
  if (mode == 0) {                              // select upper or lower temperature limit
    int val = pEnc->read();
    if (val == 2) {                             // load defaults
      pCfg->setDefaults(true);                  // Write default config to the EEPROM           
      if (main) return main;                    // Return to the main screen
    }
    if (val == 3) {
      menu_long();                              // Save the configuration to the EEPROM
      if (next) return next;
      return this;
    }
    if (val == 4) {
      if (next) return next;                    // Return to the previous menu
      return this;
    }
    mode = val + 1;
    pD->clear();
    pD->msgTune();
    switch (mode) {
      case 1:                                   // upper temp
        pD->msgUpper();
        break;
      case 2:                                   // lower temp
        pD->msgLower();
        break;
      default:
        break;
    }
    pEnc->reset(pul[mode-1], 0, max_power, 1, 5);
    pIron->fixPower(pul[mode-1]);               // Switch on the soldering iron
    arm_beep = true;
  } else {                                      // upper or lower temperature limit just setup     
    pul[mode-1] = pEnc->read();                 // The supplied power
    tul[mode-1] = pIron->tempAverage();
    pD->clear();
    pD->msgTune();
    pEnc->reset(mode-1, 0, 4, 1, 1, true);      // 0 - up temp, 1 - low temp, 2 - defaults, 3 - save, 4 - cancel
    mode = 0;
    pIron->fixPower(0);
  }
  force_redraw = true;
  return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
  pIron->fixPower(0);                           // switch off the power
  bool all_data = true;
  for (byte i = 0; i < 2; ++i) {
    if (!tul[i]) all_data = false;
  }
  if (all_data) {                               // save calibration data. Config will be written to the EEPROM later on
    pCfg->saveCalibrationData(tul[0], tul[1]);
    pIron->init(tul[0], tul[1]);
  }
  if (next) return next;
  return this;
}

//---------------------------------------- class pidSCREEN [tune the PID coefficients] -------------------------
class pidSCREEN : public SCREEN {
  public:
    pidSCREEN(IRON* Iron, ENCODER* ENC) {
      pIron = Iron;
      pEnc = ENC;
    }
    virtual void init(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
  private:
    IRON* pIron;                                // Pointer to the iron instance
    ENCODER* pEnc;                              // Pointer to the rotary encoder instance
    byte mode;                                  // Which temperature to tune [0-3]: select, Kp, Ki, Kd
    uint32_t update_screen;                     // The time in ms when the screen should be updated
    int temp_set;
    const uint16_t period = 500;                // The period the screen to be updated
};

void pidSCREEN::init(void) {
  temp_set = pIron->getTemp();
  mode = 0;                                     // select the element from the list
  pEnc->reset(1, 1, 4, 1, 1, true);             // 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp 
  Serial.println("Select the coefficient (Kp)");
  update_screen = 0;
}

void pidSCREEN::rotaryValue(int16_t value) {
  if (mode == 0) {                              // No limit is selected, list the menu
    Serial.print("[");
    for (byte i = 1; i < 4; ++i) {
      int k = pIron->changePID(i, -1);
      Serial.print(k, DEC);
      if (i < 3) Serial.print(", ");
    }
    Serial.print("]; ");
    switch (value) {
      case 1:
        Serial.println("Kp");
        break;
      case 2:
        Serial.println("Ki");
        break;
      case 4:
        Serial.println(F("Temp"));
        break;
      case 3:
      default:
       Serial.println("Kd");
       break;
    }
  } else {
    switch (mode) {
      case 1:
        Serial.print(F("Kp = "));
        pIron->changePID(mode, value);
        break;
      case 2:
        Serial.print(F("Ki = "));
        pIron->changePID(mode, value);
        break;
      case 4:
        Serial.print(F("Temp = "));
        temp_set = value;
        pIron->setTemp(value);
        break;
      case 3:
      default:
        Serial.print(F("Kd = "));
        pIron->changePID(mode, value);
        break;
    }
    Serial.println(value);
  }
}

void pidSCREEN::show(void) {
  if (millis() < update_screen) return;
  update_screen = millis() + period;
  if (pIron->isOn()) {
    char buff[60];
    int temp    = pIron->getCurrTemp();
    uint16_t td = pIron->tempDispersion();
    uint16_t pd = pIron->powerDispersion();
    sprintf(buff, "%3d: td = %3d, pd = %3d --- ", temp_set - temp, td, pd);
    Serial.println(buff);
    //if ((temp_set - temp) > 30) Serial.println("");
  }
}

SCREEN* pidSCREEN::menu(void) {                 // The rotary button pressed
  if (mode == 0) {                              // select upper or lower temperature limit
    mode = pEnc->read();
    if (mode != 4) {
      int k = pIron->changePID(mode, -1);
      pEnc->reset(k, 0, 5000, 1, 5);
    } else {
      pEnc->reset(temp_set, 0, 970, 1, 5);
    }
  } else {                                      // upper or lower temperature limit just setup     
    mode = 0;
    pEnc->reset(1, 1, 4, 1, 1, true);           // 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp
  }
  return this;
}

SCREEN* pidSCREEN::menu_long(void) {
  bool on = pIron->isOn();
  pIron->switchPower(!on);
  if (on)
    Serial.println("The iron is OFF");
  else
    Serial.println("The iron is ON");
  return this;
}
//=================================== End of class declarations ================================================

DSPL       disp(LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN);
ENCODER    rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON     rotButton(R_BUTN_PIN);
IRON       iron(heaterPIN, probePIN);
CONFIG     ironCfg;
BUZZER     simpleBuzzer(buzzerPIN);

mainSCREEN   offScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
workSCREEN   wrkScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
errorSCREEN  errScr(&disp, &simpleBuzzer);
powerSCREEN  powerScr(&iron, &disp, &rotEncoder);
configSCREEN cfgScr(&iron, &disp, &rotEncoder, &ironCfg);
tuneSCREEN   tuneScr(&iron, &disp, &rotEncoder, &simpleBuzzer, &ironCfg);
//pidSCREEN    pidScr(&iron, &rotEncoder);

SCREEN *pCurrentScreen = &offScr;
//SCREEN *pCurrentScreen = &pidScr;

// the setup routine runs once when you press reset:
void setup() {
  //Serial.begin(9600);
  disp.init();

  // Load configuration parameters
  ironCfg.init();
  bool is_cfg_valid = ironCfg.load();
  uint16_t temp_min, temp_max;
  ironCfg.getCalibrationData(temp_max, temp_min);

  iron.init(temp_max, temp_min);
  uint16_t temp = ironCfg.temp();
  iron.setTemp(temp);

  // Initialize rotary encoder
  rotEncoder.init();
  rotButton.init();
  delay(500);
  attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_BUTN_PIN), rotPushChange,  CHANGE);

  // Initialize SCREEN hierarchy
  offScr.next    = &wrkScr;
  offScr.nextL   = &cfgScr;
  wrkScr.next    = &offScr;
  wrkScr.nextL   = &powerScr;
  wrkScr.main    = &offScr;
  errScr.next    = &offScr;
  errScr.nextL   = &offScr;
  powerScr.nextL = &wrkScr;
  cfgScr.next    = &tuneScr;
  cfgScr.nextL   = &offScr;
  cfgScr.main    = &offScr;
  tuneScr.next   = &cfgScr;
  tuneScr.main   = &offScr;
  pCurrentScreen->init();

}

void rotEncChange(void) {
  rotEncoder.cnangeINTR();
}

void rotPushChange(void) {
  rotButton.cnangeINTR();
}

// The main loop
void loop() {
  static int16_t old_pos = rotEncoder.read();
  iron.keepTemp();                                // First, read the temperature

  bool iron_on = iron.isOn();
  if ((pCurrentScreen == &wrkScr) && !iron_on) {  // the soldering iron failed
    pCurrentScreen = &errScr;
    pCurrentScreen->init();
  }

  SCREEN* nxt = pCurrentScreen->returnToMain();
  if (nxt != pCurrentScreen) {                  // return to the main screen by timeout
    pCurrentScreen = nxt;
    pCurrentScreen->init();
  }

  byte bStatus = rotButton.intButtonStatus();
  switch (bStatus) {
    case 2:                                     // long press;
      nxt = pCurrentScreen->menu_long();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 1:                                     // short press
      nxt = pCurrentScreen->menu();
      if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
      } else {
        if (pCurrentScreen->isSetup())
         pCurrentScreen->resetTimeout();
      }
      break;
    case 0:                                     // Not pressed
    default:
      break;
  }

  int16_t pos = rotEncoder.read();
  if (old_pos != pos) {
    pCurrentScreen->rotaryValue(pos);
    old_pos = pos;
    if (pCurrentScreen->isSetup())
     pCurrentScreen->resetTimeout();
  }

  pCurrentScreen->show();
   
  disp.show();
  delay(10);
}

