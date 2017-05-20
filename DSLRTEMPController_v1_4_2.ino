#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <Bounce2.h>

#include "types.h"

#undef _DALLAS_CAL
#undef _FANHALL      // optional feature, not tested

#ifdef _DALLAS_CAL
#include <OneWire.h>             // solo para calibrar
#include <DallasTemperature.h>   // solo para calibrar
#define DALLAS_PIN A3            // solo para calibrar
#endif


// OJO:  The ATMEGA328P-AU has two analog pins that have no digital functions: A6, A7
// TQFP package (AU) has 2 more ADC’s than the DIP package (P)
// but (Arduino) A6 and A7 are only analog inputs, they cannot be used for outputs
// – nor do they have any pull-ups on them.

#define VERSION        "1.4.2"
#define CONFIG_VERSION "002"

#define COLDSIDE_DIODE_PIN   A4
#define HOTSIDE_DIODE_PIN    A5
#define AUXTEMP_DIODE_PIN    A6  // only analog input
#define COLDFINGER_DIODE_PIN A7  // only analog input

#define PWM_PELTIER_PIN   9   // Phase Correct PWM (490.196Hz)
#define PWM_HEATER_PIN    6   // Fast PWM (976.5625Hz)

#define ENCODER_A_PIN     A0  // as digital input
#define ENCODER_B_PIN     A1  // as digital input
#define BUTTON_PIN        A2  // as digital input

#define BEEPER_PIN   10    // PWM

#define FAN_PIN       3    // PWM
#ifdef _FANHALL
#define FANHALL_PIN   2    // RPM interrupt - optional feature
#endif

#define RS_LCD_PIN  12
#define EN_LCD_PIN  11
#define D4_LCD_PIN  4
#define D5_LCD_PIN  5
#define D6_LCD_PIN  8
#define D7_LCD_PIN  7

#define SETUP_MODE_TIMEOUT 10 // seconds
#define BEEP_FREQ 2400

#define PID_AGR_DELTA  7 // define el delta de temperatura para aplicar parametros agresivos

#define MAX_REG_TEMP  20   // maxima temperatura regulada
#define MIN_REG_TEMP -15   // minima temperatura regulada

#define MAX_TEMP_HOT_SIDE 60 // a partir de esta temperatura se genera una alarma y se corta la alimentacion del peltier
#define MAX_HEATER_VALUE 128 // valor maximo PWM del heater

#define BUTTON_DEBOUNCEDELAY   15
#define ENCODER_DEBOUNCEDELAY   5
#define BUTTON_DOUBLECLICK    500

#define EE_CONFIG_START 32
struct EEpromStore {
  char version[4];
  double PIDSetpoint;
  boolean heaterModeAuto;
  byte heaterValue;
} 
eepromConfigData = {
  CONFIG_VERSION,25,true,0};

static logprio_type logprio_limit=log_info;

volatile boolean button_lastvalue = HIGH;
volatile boolean encoder_a_lastvalue = HIGH;
volatile int encoder_value = 0;
volatile unsigned long lastDebounceTime = 0;
volatile boolean loop_halfsecond_ready = true;
volatile boolean loop_20Hz_ready = true;
volatile boolean beeping = false;
volatile boolean loop_beep_ready = true;
volatile boolean lcd_power_blink_on = false, lcd_power_blink = false;

volatile lcd_mode_type lcd_mode;
volatile long setup_mode_time;

volatile byte heaterValue = 0; // 0% ... 100%
volatile boolean heaterModeAuto = false;

volatile int  FAN_rpm = 0;
volatile long oldFANRPMtime = 0;
volatile int  FAN_pwm = 255;
#ifdef _FANHALL
volatile byte HALLPulses = 0; // optional feature
#endif

volatile boolean peltierOFF = false;

// for inputs
volatile int    lastHotSideTemp_Value,lastColdSideTemp_Value,lastFingerTemp_Value,lastAuxTemp_Value;
volatile double lastHotSideTemp,lastColdSideTemp,lastFingerTemp,lastAuxTemp;

// for filters
int    xFingerTempValues[4],xAuxTempValues[4],xHotSideTempValues[4],xColdSideTempValues[4];
double yFingerTempValues[4],yAuxTempValues[4],yHotSideTempValues[4],yColdSideTempValues[4];

// for PID Controller
double PIDSetpoint, PIDInput, PIDOutput; // target (ºC), coldfinger temp (ºC), pwm peltier power (0..255)
double  aggKp=16,   aggKi=4,  aggKd=2;
double consKp=1.5, consKi=0.2, consKd=0.5;

byte lcdCustomChars[10][8] =
{
  {
    0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00001 }
  ,
  {
    0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00011,0b00001 }
  ,
  {
    0b00000,0b00000,0b00000,0b00000,0b00000,0b00111,0b00011,0b00001 }
  ,
  {
    0b00000,0b00000,0b00000,0b00000,0b01111,0b00111,0b00011,0b00001 }
  ,
  {
    0b00000,0b00000,0b00000,0b11111,0b01111,0b00111,0b00011,0b00001 }
  ,
  {
    0b00000,0b00000,0b00000,0b11111,0b01111,0b00111,0b00011,0b00001 }
  ,
  {
    0b00000,0b00000,0b11111,0b11111,0b11111,0b01111,0b00111,0b00011 }
  ,
  {
    0b00000,0b11111,0b11111,0b11111,0b11111,0b11111,0b01111,0b00111 }
  ,
  {
    0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b01111 }
  ,
  {
    0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111 }
};



//----

LiquidCrystal lcd(RS_LCD_PIN, EN_LCD_PIN, D4_LCD_PIN, D5_LCD_PIN, D6_LCD_PIN, D7_LCD_PIN);

//PID links and initial tuning parameters
PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint, aggKp, aggKi, aggKd, REVERSE);

// Debouncers
Bounce button    = Bounce();
Bounce encoder_a = Bounce();
Bounce encoder_b = Bounce(); 


void SerialPrintLog(char* string, logprio_type prio) {
  char * prio_literal[] = {"ERROR", "WARNING", "INFO", "DEBUG"};

  prio = constrain(prio,log_error,log_debug);
  if (prio <= logprio_limit) {
   Serial.print(millis());
   Serial.print(":");
   Serial.print(prio_literal[prio]);
   Serial.print(":");
   if (string != 0L)
     Serial.println(string);
  }
}

void SerialPrintLogHead(logprio_type prio) {
  SerialPrintLog(0L,prio);
}

void doBip() {
  tone(BEEPER_PIN,BEEP_FREQ);
  delay(100);
  noTone(BEEPER_PIN);
  delay(20);
}

void doBipBip() {
  tone(BEEPER_PIN,BEEP_FREQ);
  delay(30);
  noTone(BEEPER_PIN);
  delay(20);
  tone(BEEPER_PIN,BEEP_FREQ);
  delay(30);
  noTone(BEEPER_PIN);
  delay(20);
}

void doBipBipBip() {
  tone(BEEPER_PIN,BEEP_FREQ);
  delay(50);
  noTone(BEEPER_PIN);
  delay(20);
  tone(BEEPER_PIN,BEEP_FREQ);
  delay(50);
  noTone(BEEPER_PIN);
  delay(20);
  tone(BEEPER_PIN,BEEP_FREQ);
  delay(50);
  noTone(BEEPER_PIN);
  delay(20);
}

void doSUPERBip() {
  tone(BEEPER_PIN,BEEP_FREQ);
  delay(10000);
  noTone(BEEPER_PIN);
  delay(500);
}

int medianFilter (int xValues[], int value) {
  if (xValues[3] < 0 ) { // inicializacion
    xValues[0] = xValues[1] = xValues[2] = xValues[3] = value;
  }
  xValues[0] = xValues[1];
  xValues[1] = xValues[2];
  xValues[2] = xValues[3];
  xValues[3] = value;

  int result=0;
  for (int i=0;i<4;i++)
    result += xValues[i];
  result /= 4;

  return (result);
}

int IIR (int xValues[], double yValues[], int value) {
  xValues[0] = xValues[1]; 
  xValues[1] = xValues[2];
  xValues[2] = xValues[3];
  xValues[3] = value;

  // IIR Butterworth filter with cutoff frequency 0.033*sample frequency
  // http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
  yValues[0] = yValues[1];
  yValues[1] = yValues[2];
  yValues[2] = yValues[3];
  yValues[3] = (xValues[0] + 3 *(xValues[1] + xValues[2]) + xValues[3] ) / 1.092799972e+03 +
    (0.6600489526 * yValues[0]) + -2.2533982563 * yValues[1] + 2.5860286592 * yValues[2];

  return(yValues[3]);
}


// modelado lineal del coomportamiento de los diodos
// V=0V    RawADC=0
// V=1.1V  RAWADC=1024
// V = b - mT


// redondeo al decimal
double diodeL(int RawADC, double b, double m) {
  return (int)( ((b-RawADC) / m )*10 ) / 10.0;
}

double HotSideTempDiode(int RawADC) { // HOT
  static double b = 501.76; 
  static double m = 2.2150604;
  return diodeL (RawADC,b,m);
}

double ColdSideTempDiode(int RawADC) { // COLD
  static double b = 502.49;
  static double m = 2.1704242;
  return diodeL (RawADC,b,m);
}

double ColdFingerTempDiode(int RawADC) { // FINGER
  static double b = 503.45;
  static double m = 2.15791;
  return diodeL (RawADC,b,m);
}

double AuxTempDiode(int RawADC) { // AUX
  static double b = 503.47;
  static double m = 2.15329;
  return diodeL (RawADC,b,m);
}

lcd_mode_type lcdMode(int encoder_value) {
  switch (encoder_value % 6) {
  case 0: 
    return temp_coldfinger;
  case 1: 
    return temp_hotside;
  case 2: 
    return temp_coldside;
  case 3: 
    return temp_cam;
  case 4: 
    return pwm_peltier;
  case 5: 
    return heat_heater;
  }
}

#ifdef _DALLAS_CAL
OneWire ourWire(DALLAS_PIN);
DallasTemperature dallas(&ourWire);
#endif

void saveConfigToEEprom() {
  eepromConfigData.PIDSetpoint = peltierOFF ? MAX_REG_TEMP+1 : PIDSetpoint;
  eepromConfigData.heaterModeAuto = heaterModeAuto;
  eepromConfigData.heaterValue = heaterValue;
  for (unsigned int t=0; t<sizeof(eepromConfigData); t++)
    EEPROM.write(EE_CONFIG_START + t, *((char*)&eepromConfigData + t));
  SerialPrintLog("SAVE Config",log_debug);
}

boolean restoreConfigFromEEprom() {
  boolean isOK = false;

  if (EEPROM.read(EE_CONFIG_START + 0) == CONFIG_VERSION[0] &&
    EEPROM.read(EE_CONFIG_START + 1) == CONFIG_VERSION[1] &&
    EEPROM.read(EE_CONFIG_START + 2) == CONFIG_VERSION[2]) {

    for (unsigned int t=0; t<sizeof(eepromConfigData); t++)
      *((char*)&eepromConfigData + t) = EEPROM.read(EE_CONFIG_START + t);

    if (eepromConfigData.PIDSetpoint > MAX_REG_TEMP) {
      PIDSetpoint = MAX_REG_TEMP;
      peltierOFF = true;
    } else
      PIDSetpoint = eepromConfigData.PIDSetpoint;
    heaterModeAuto = eepromConfigData.heaterModeAuto;
    heaterValue = eepromConfigData.heaterValue;
    SerialPrintLog("RESTORE Config OK",log_debug);
    
    isOK = true;
  } 
  else {
    SerialPrintLog("RESTORE Config FAIL: bad format",log_error);
    isOK = false;
  }
  return (isOK);
}

#ifdef _FANHALL
void FANInterrupt() // optional feature
{
  HALLPulses++;
}
#endif

void refreshLCD() {
  unsigned long currentTime = millis();

  lcd.clear();
  switch (lcd_mode) {
  case temp_coldfinger:
    lcd.print("C.FINGER");
    lcd.setCursor(0,1);
    lcd.print(lastFingerTemp,1);
    lcd.write(0xdf);
    lcd.print("C");
    break;

  case temp_hotside:
    lcd.print("HOT SIDE");
    lcd.setCursor(0,1);
    lcd.print(lastHotSideTemp,1);
    lcd.write(0xdf);
    lcd.print("C");
    break;

  case temp_coldside:
    lcd.print("COLD S.");
    lcd.setCursor(0,1);
    lcd.print(lastColdSideTemp,1);
    lcd.write(0xdf);
    lcd.print("C");
    break;

  case temp_cam:
    lcd.print("AUX TEMP");
    lcd.setCursor(0,1);
    lcd.print(lastAuxTemp,1);
    lcd.write(0xdf);
    lcd.print("C");
    break;

  case pwm_peltier:
    lcd.print("PELTIER");
    lcd.setCursor(0,1);
    if (peltierOFF)
        lcd.print("OFF");
    else {
        lcd.print((PIDOutput*100)/255,0);
        lcd.print("%");
    }
    break;

  case heat_heater:
    lcd.print("HEATER");
    lcd.setCursor(0,1);
    if (heaterValue == 0)
      lcd.print("OFF");
    else {
      lcd.print(heaterValue);
      lcd.print("%");
    }
    if (heaterModeAuto) {
      lcd.setCursor(5,1);
      lcd.print("A");
    }
    break;

  case setup_temp:
    lcd.print("SET TEMP");
    lcd.setCursor(0,1);
    if (peltierOFF)
        lcd.print("OFF");
    else {
        lcd.print(PIDSetpoint,0);
        lcd.write(0xdf);
        lcd.print("C");
    }
    if (currentTime > setup_mode_time + SETUP_MODE_TIMEOUT*1000)
      lcd_mode = temp_coldfinger;
    break;

  case setup_heater:
    lcd.print("HEATER");
    lcd.setCursor(0,1);
    if (heaterModeAuto)
      lcd.print("AUTO");
    else if (heaterValue == 0)
      lcd.print("OFF");
    else {
      lcd.print(heaterValue,1);
      lcd.print("%");
    }
    if (currentTime > setup_mode_time + SETUP_MODE_TIMEOUT*1000)
      lcd_mode = temp_coldfinger;
    break;

  case setup_save:
    lcd.print(" SAVE TO");
    lcd.setCursor(0,1);
    lcd.print("EEPROM ?");
    if (currentTime > setup_mode_time + SETUP_MODE_TIMEOUT*1000/2) // espero la mitad antes de salir del modo
      lcd_mode = temp_coldfinger;
    break;

  case alarm:
    lcd.print("ALARM!!!");
    lcd.setCursor(0,1);
    lcd.print("HOT HOT!");
    break;

  default:
    lcd.print("CURSOR");
    lcd.setCursor(0,1);
    lcd.print(encoder_value);
    break;
  }

  if ( ! (lcd_power_blink && lcd_power_blink_on )) {
    lcd.setCursor(7,1);
    //lcd.print(map(PIDOutput,0,255,0,9));
    lcd.write((uint8_t)map(PIDOutput,0,255,0,9));  // Potencia del peltier
  }

  if (lcd_power_blink)
    lcd_power_blink_on = !lcd_power_blink_on;
  else
    lcd_power_blink_on = true;

}

void setup() {
  analogReference(INTERNAL);        // 1.1 Volt reference

  delay(500);

  for (int c=0;c<10;c++)
    lcd.createChar(c,lcdCustomChars[c]);

  Serial.begin(9600);

#ifdef _DALLAS_CAL
  dallas.begin();
#endif

  lcd.begin(8, 2);

  if (!restoreConfigFromEEprom()) { // carga valores de la eeprom, si falla pongo los de por defecto
    heaterValue = 10;   // calefactor al 10%
    PIDSetpoint = 10;   // temperatura objetivo a 10ºC
  }

  pinMode (BUTTON_PIN,INPUT_PULLUP);
  pinMode (ENCODER_A_PIN,INPUT_PULLUP);
  pinMode (ENCODER_B_PIN,INPUT_PULLUP);

  button.attach(BUTTON_PIN);
  button.interval(BUTTON_DEBOUNCEDELAY);
  encoder_a.attach(ENCODER_A_PIN);
  encoder_a.interval(ENCODER_DEBOUNCEDELAY);
  encoder_b.attach(ENCODER_B_PIN);
  encoder_b.interval(ENCODER_DEBOUNCEDELAY);

#ifdef _FANHALL
  pinMode (FANHALL_PIN,INPUT_PULLUP);     // optional feature
#endif

  FAN_pwm = 255;  // ventilador a tope
  pinMode (FAN_PIN,OUTPUT);
  analogWrite(FAN_PIN, FAN_pwm);

  pinMode (PWM_HEATER_PIN, OUTPUT);
  analogWrite(PWM_HEATER_PIN, 0); // desconecto la calefaccion. Mas tarde se pondra al valor correcto

  pinMode (BEEPER_PIN,OUTPUT);

  // filters setup
  for (int i=0;i<4;i++) {
    xFingerTempValues[i] = -1;
    xColdSideTempValues[i] = -1;
    xHotSideTempValues[i] = -1;
    xAuxTempValues[i] = -1;
  }
  // PID SETUP -----------------------------------------
  PIDInput =  ColdFingerTempDiode(analogRead(COLDFINGER_DIODE_PIN));
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
  // END PID SETUP -----------------------------------------

#ifdef _FANHALL
  attachInterrupt(0, FANInterrupt, RISING); // Interrupt 0 => PIN 2 !!!! // optional feature // NOT TESTING
  HALLPulses = 0;    // optional feature
#endif
  FAN_rpm = 0;       // optional feature
  oldFANRPMtime = 0; // optional feature

  lcd.clear();
  lcd.print("WELLCOME");
  lcd.setCursor(0,1);
  lcd.print("v");
  lcd.print(VERSION);

  SerialPrintLogHead(log_info);
  Serial.print("Startup v");
  Serial.println(VERSION);

  doBipBip();
  beeping = false;

  pinMode (13,OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  delay(500);
}

void loop() {
  int v;
  unsigned long currentTime = millis();

  // DO every 1/20 seconds (20Hz)
  if (currentTime/200 % 2) { 
    if (loop_20Hz_ready) {

      //lastHotSideTemp = DiodeB(IIR(xHotSideTempValues,yHotSideTempValues,analogRead(HOTSIDE_DIODE_PIN)));
      //lastColdSideTemp = DiodeA(IIR(xColdSideTempValues,yColdSideTempValues,analogRead(COLDSIDE_DIODE_PIN)));
      //lastFingerTemp = DiodeA(IIR(xFingerTempValues,yFingerTempValues,analogRead(COLDFINGER_DIODE_PIN)));
      //lastAuxTemp = DiodeA(IIR(xAuxTempValues,yAuxTempValues,analogRead(AUXTEMP_DIODE_PIN)));

      v = analogRead(HOTSIDE_DIODE_PIN);
      v = medianFilter(xHotSideTempValues,v);
      lastHotSideTemp = HotSideTempDiode(v);
      lastHotSideTemp_Value = v;

      v = analogRead(COLDSIDE_DIODE_PIN);
      v = medianFilter(xColdSideTempValues,v);
      lastColdSideTemp = ColdSideTempDiode(v);
      lastColdSideTemp_Value = v;

      v = 0;
      for (int i=0; i<25; i++)
        v += analogRead(COLDFINGER_DIODE_PIN); // tomo 25 medidas seguidas y obtengo la mediana
      v /= 25;
      v = medianFilter(xFingerTempValues,v); // filtro valores
      lastFingerTemp = ColdFingerTempDiode(v);
      lastFingerTemp_Value = v;

      v = analogRead(AUXTEMP_DIODE_PIN);
      v = medianFilter(xAuxTempValues,v);
      lastAuxTemp = AuxTempDiode(v);
      lastAuxTemp_Value = v;

      // PID Loop Control -----------------------------------------
      // Compute() should be caled once every loop();
      PIDInput = lastFingerTemp; // temperature computed every 1/20 sec
      
      if ( ! peltierOFF && lastHotSideTemp > MAX_TEMP_HOT_SIDE) { // ALARM -- DANGER --- CHECK PELTIER TEMPERATURE
        PIDInput = 0;   // SAFE
        FAN_pwm = 255;  // SAFE
        analogWrite(PWM_PELTIER_PIN, 0); // CUTOFF PELTIER SUPPLY
        analogWrite(FAN_PIN, 255); // ventilador a tope
        doSUPERBip();
        beeping = false;
        lcd_mode = alarm;
      }
      // End Loop Control 

      // Antidew heater loop  -------------------------------------
      if (heaterModeAuto) {
        // ajuste lineal segun la temperatura target
        heaterValue = constrain(map((long)PIDInput,0,15,75,10),0,100);
      } 
      else
        heaterValue = constrain(heaterValue,0,100);  

      analogWrite(PWM_HEATER_PIN, map(heaterValue,0,100,0,MAX_HEATER_VALUE));

      // end Heater Loop  -------------------------------------

      refreshLCD();

      loop_20Hz_ready = false;
    }
  }
  else
    loop_20Hz_ready = true;

  // DO every 1/2 second
  if ( currentTime / 500 % 2)  { 
    if (loop_halfsecond_ready) {
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      
      //SerialPrintLog("COLDFINGER;PELTIERHOT;PELTIERCOLD;AUXTemp;FANPWM;FANRPM;HEATER;PIDSetpoint;PIDInput;PIDOutput",log_info);
      SerialPrintLogHead(log_info);
      Serial.print("STATUS:");
      Serial.print(lastFingerTemp);
      Serial.print(";"); 
      Serial.print(lastFingerTemp_Value);
      Serial.print(";"); 
      Serial.print(lastHotSideTemp);
      Serial.print(";"); 
      Serial.print(lastHotSideTemp_Value);
      Serial.print(";"); 
      Serial.print(lastColdSideTemp); 
      Serial.print(";"); 
      Serial.print(lastColdSideTemp_Value); 
      Serial.print(";"); 
      Serial.print(lastAuxTemp);
      Serial.print(";"); 
      Serial.print(lastAuxTemp_Value);
      Serial.print(";"); 
      Serial.print(FAN_pwm,DEC);
      Serial.print(";"); 
      Serial.print(FAN_rpm,DEC);
      Serial.print(";"); 
      Serial.print(heaterValue,DEC);
      Serial.print(";"); 
      Serial.print(PIDSetpoint,2);
      Serial.print(";"); 
      Serial.print(PIDInput,2);
      Serial.print(";"); 
      Serial.print(PIDOutput,2);      
#ifdef _DALLAS_CAL
      Serial.print(";");
      dallas.requestTemperatures();
      Serial.print(dallas.getTempCByIndex(0));
#endif
      Serial.println();
      // End Serial Debug

      if (abs(PIDSetpoint - PIDInput) > PID_AGR_DELTA) {
        myPID.SetTunings(aggKp, aggKi, aggKd);
        doBipBip();
        lcd_power_blink = true;
          SerialPrintLog("PID AGRESIVE",log_info);
      } 
      else {
        myPID.SetTunings(consKp, consKi, consKd);
        lcd_power_blink = false;
      }

      loop_halfsecond_ready = false;
    }
  } 
  else {
    digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
    loop_halfsecond_ready = true;
  }

  // maneja el beeper
  if ( beeping && loop_beep_ready && (currentTime/250 % 2) ) { // cada 1/4 segundo beep on/off
    tone(BEEPER_PIN,BEEP_FREQ); // will interfere with PWM output on pins 3 and 11 
    loop_beep_ready = false;
  } 
  else if (!loop_beep_ready) {
    noTone(BEEPER_PIN);
    loop_beep_ready = true;
  }

  // maneja la pulsacion del boton del encoder
  if (button.update()) {
    unsigned char button_value = button.read();
    if ((button_lastvalue == HIGH) && (button_value == LOW)) { 
      SerialPrintLog("PUSH",log_debug);
      if (currentTime < setup_mode_time + BUTTON_DOUBLECLICK) { // click-click (1/4 seg)
        lcd_mode = setup_save;
        setup_mode_time = currentTime;
      } 
      else {
        switch (lcd_mode) {
        case setup_temp: // el modo anterior anterior era setup_temp, el siguiente sera setup_heater
          lcd_mode = setup_heater;
          if (heaterModeAuto)
            encoder_value = -1;
          else
            encoder_value = heaterValue; // 0..100

          setup_mode_time = currentTime;
          break;

        case setup_heater: // el anterior era setup_heater, salgo del modo setup
          lcd_mode = temp_coldfinger;
          encoder_value = 0;
          break;

        case setup_save: // el anterior era setup_save, salvo datos en la eeprom
          saveConfigToEEprom();
          lcd_mode = temp_coldfinger;
          encoder_value = 0;
          break;

        default: // el anterior era un modo normal
          lcd_mode = setup_temp;
          encoder_value = PIDSetpoint;
          setup_mode_time = currentTime;
          break;
        }
      }
      refreshLCD();
      doBip();
    }
    button_lastvalue = button_value;
  }

  // gestiona el movimiento del mando del encoder


  boolean encoder_a_change = encoder_a.update();
  boolean encoder_b_change = encoder_b.update();
  if (encoder_a_change || encoder_b_change ) {
    unsigned char encoder_a_value = encoder_a.read();
    unsigned char encoder_b_value = encoder_b.read();

    if ((encoder_a_lastvalue == LOW) && (encoder_a_value == HIGH)) {
      if (encoder_b_value == LOW) {
        encoder_value--;
      }
      else {
        encoder_value++;
      }
      switch (lcd_mode) {
      case setup_temp: // estoy ajstando la temperatura objetivo
        if (encoder_value > MAX_REG_TEMP)
          peltierOFF = true;
        else
          peltierOFF = false;
          
        PIDSetpoint = constrain(encoder_value,MIN_REG_TEMP,MAX_REG_TEMP);
        encoder_value = PIDSetpoint;
        setup_mode_time = currentTime;
        break;

      case setup_heater: // estoy ajustando el calefactor
        heaterModeAuto = encoder_value<0 ? true : false;
        if (!heaterModeAuto) {
          heaterValue = constrain(encoder_value*5,0,100);
          encoder_value = heaterValue/5;
        } 
        else
          encoder_value=-1;

        setup_mode_time = currentTime;
        break;

      case setup_save: // cancelo modo save2eeprom
        lcd_mode = temp_coldfinger;
        break;

      default: // estoy visualizando parametros con el LCD
        encoder_value = constrain(encoder_value,temp_coldfinger,heat_heater);
        lcd_mode = lcdMode(encoder_value);
        break;
      }
      refreshLCD();
    }
    encoder_a_lastvalue = encoder_a_value;
  }
  
  // PID Loop -----------------------------------------
  // Compute() should be caled once every loop();
  PIDInput = lastFingerTemp; // temperature is computed every 1/20 sec (50ms), default PID sample time is 200ms.
  if ( myPID.Compute() ) {
    PIDOutput = lastHotSideTemp < MAX_TEMP_HOT_SIDE ? PIDOutput : 0;
    PIDOutput = peltierOFF ? 0 : PIDOutput; // peltieroff => PIDOutput = 0
    analogWrite(PWM_PELTIER_PIN, PIDOutput);
  }
  // End PID Loop -------------------------------------

#ifdef _FANHALL
  if (HALLPulses >= 25) { // optional feature
    FAN_rpm = 30*1000/(millis() - oldFANRPMtime)*HALLPulses;
    oldFANRPMtime = millis();
    HALLPulses = 0;
  }
#endif

}














