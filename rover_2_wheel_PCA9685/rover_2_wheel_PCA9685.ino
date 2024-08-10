
#include <Adafruit_INA260.h>
#include <Adafruit_IS31FL3731.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include <PWMServo.h>
#include <String.h>
#include <Wire.h>
#include <rxToSrvo.h>
#include <serialPrint.h>
#include <vec3.h>
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <SPI.h>
#include <ina260_wrapper.h>
#include <InternalTemperature.h>
#include <led_pulse_train.h>

#define SVO_0_PIN 0
#define SVO_1_PIN 1
#define SVO_2_PIN 2
#define SVO_3_PIN 3
#define SVO_4_PIN 4
#define SVO_5_PIN 5
#define SVO_6_PIN 6
#define SVO_7_PIN 7
#define SVO_8_PIN 8
#define SVO_9_PIN 9
#define SVO_10_PIN 10
#define SVO_11_PIN 11

#define RX_THR_PIN  5
#define RX_AIL_PIN  6
#define RX_ELE_PIN  7
#define RX_RUD_PIN  8
#define RX_GEAR_PIN 9
#define RX_AUX_PIN  10

#define CAM_LED_PIN 1
#define NAV_LED_PIN 0
#define FLOOD_LED_PIN 2
#define ACC_PWR_PIN 3

#define NUM_BYTES 32
#define STEER_RUDDER_GAIN -.30
#define INB_STR_DIFF_GAIN -.10

#define MIN_ABS_PULSE_WIDTH 1000
#define MIN_THIRD_PULSE 1350
#define MID_PULSE_WIDTH 1500
#define MAX_THIRD_PULSE 1650
#define MAX_ABS_PULSE_WIDTH 2000

#define MAG_MIN -200
#define MAG_MAX 200

#define THETA_MAX 360
#define THETA_MIN -360

#define CAL_DURATION 350
#define RAD 180 / 3.1415792

#define SP_OFFSET_LF 7.0
#define SP_OFFSET_RF 6.0

#define FLASH_SS      8 // and FLASH SS on D8
#define VOLT_SUB_USB  4200
#define VOLT_LOW_USB  4500
#define VOLT_USB      5000
#define VOLT_SUB_BATT 11700

enum ledMatrixModes {
  LED_MTX_IDLE,
  LED_MTX_DEF,
  LED_MTX_OFF,
  LED_MTX_ON,
  LED_MTX_BUF_00,
  LED_MTX_BUF_01,
  LED_MTX_BUF_02,
  LED_MTX_BUF_03,
  LED_MTX_BUF_04,
  LED_MTX_BUF_05,
  LED_MTX_BUF_06,
  LED_MTX_BUF_07,
  NUM_LED_MTX
  };

String ledMatrixModeStrs[] = {
    "MTX_IDLE",
    "MTX_DEF",
    "MTX_OFF",
    "MTX_BUF_ON",
    "MTX_BUF_00",
    "MTX_BUF_01",
    "MTX_BUF_02",
    "MTX_BUF_03",
    "MTX_BUF_04",
    "MTX_BUF_05",
    "MTX_BUF_06",
    "MTX_BUF_07",
  };

enum serPrntModes {
  SP_MODE_COMPASS,
  SP_MODE_DOUT,
  SP_MODE_I2C,
  SP_MODE_9DOF,
  SP_MODE_LL,
  SP_MODE_OP,
  SP_MODE_WH_SVO_OUT,
  SP_MODE_ST_SVO_OUT,
  SP_MODE_SVO_OFFSET,
  SP_MODE_POWER,
  SP_MODE_SVO_TEST,
  NUM_SP_MODES,
  };

String serPrntModeStrs[] = {
    "COMPASS",
    "DOUT",
    "I2C",
    "9dof",
    "LL",
    "OP",
    "ws SVO out",
    "st SVO out",
    "SVO offsets",
    "POWER",
    "SVO Test",
  };



enum opModes {
  OPM_BOOT,
  OPM_START,
  OPM_LINK_NOM,
  OPM_INIT_LL,
  OPM_LL,
  OPM_ILL_LL,
  OPM_USB_LINK_NOM,
  OPM_USB_INIT_LL,
  OPM_USB_LL,
  OPM_USB_ILL_LL,
  OPM_SLEEP,
  OPM_SVO_TEST,
  NUM_OP_MODES,
  };

String opModeStrs[NUM_OP_MODES] = {
  "BOOT",
  "Start",
  "Link Ok",
  "INIT LL",
  "LL",
  "INIT LL & LL"
  "USB Link Ok",
  "USB INIT LL",
  "USB LL",
  "USB INIT LL & LL"
  "SLEEP",
  "SVO Test",
  };

enum powerStates {
  PWR_STATE_BOOT,
  PWR_STATE_OFLINE,
  PWR_STATE_SUB_USB,
  PWR_STATE_LOW_USB,
  PWR_STATE_USB,
  PWR_STATE_SUB_BATT,
  PWR_STATE_NOM_BATT,
  NUM_PWR_STATES,
  };

String powerStateStrs[] = {
  "    BOOT",
  " OFFLINE",
  " SUB_USB",
  " LOW_USB",
  "     USB",
  "SUB_BATT",
  "NOM_BATT",
  };

// prototype
void isrThr();
void isrAil();
void isrEle();
void isrRudder();
void isrGear();
void isrAux();

bool rxLostLink = false;
bool rxInitLostLink = false;
bool rxProbableLL = false;
bool rxPitchStale = false;
bool rxRollStale = false;
bool rxAuxStale = false;

bool camLEDs = false;
bool floodLEDs = false;
bool spComp = true;
bool whSpAdjMode = false;

bool pin0 = false;
bool pin1 = false;
bool pin2 = false;
bool pin3 = false;

bool pin15 = false;
bool pin14 = false;
bool pin13 = false;

bool serBC = true;
bool serCompassBC = false;
bool compassOnline = false;
bool pwrSensorOnline = false;
bool ledMatrixOnline = false;
bool flashOnline = false;
bool imuOnline = false;
bool spiOnline = false;
bool strInLeft = false;
bool strInRight = false;

bool newData = false;

bool lowBattVolt = false;

char inByteBuffer[NUM_BYTES] = {};
int gearSwPos = 0;
int auxSwPos = 0;


uint8_t opMd;
uint8_t powerState = PWR_STATE_BOOT;
uint8_t ledMtxMode = LED_MTX_DEF;
uint8_t serPrntMode = SP_MODE_OP;



int iCount;
int strState = 0;
int spState = 0;
int panState = 0;
int tiltState = 0;


int byteCnt;

String inStr = "";

sensors_event_t event;

float dSp_LF = 0;
float dSp_RF = 0;


float dPan = 0;
float dTilt = 0;

float thrD = 0;
float strD = 0;
float intTempC = 0;

int mainBattCurrent = 0;
int mainBattV = 0;
int sysPower = 0;

int currentMin = -20000;
int voltMin = 24000;
int powerMin = 0;

int currentMax = 0;
int voltMax = 0;
int powerMax = 0;

rxToSrvo rtsThrottle;
rxToSrvo rtsAil;
rxToSrvo rtsEle;
rxToSrvo rtsRudder;
rxToSrvo rtsGear;
rxToSrvo rtsAux;

ina260_wrapper i2w;

Adafruit_IS31FL3731_Wing ledmatrix = Adafruit_IS31FL3731_Wing();
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef void (*pInterruptFunc)(void);

float svoCmdWhLF;
float svoCmdWhRF;

float svoCmdPan;
float svoCmdTilt;

uint16_t svoCmdWhLF_pwm;
uint16_t svoCmdWhRF_pwm;


//////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//////////////////////////////////////////
SPIFlash flash(FLASH_SS, 0xEF30);

//-----------------------------------------------------------------------------
void Blink(bool nStart, byte nBlinks) {
  static bool nStart_shadow = false;
  static int _blink_cnt = 0;

  if (iCount % 5000) {
    if (nStart_shadow != nStart && nStart == true) {
      _blink_cnt = nBlinks;
      }

    if (_blink_cnt > 0) {
      pin13 = !pin13;
      _blink_cnt--;
      }
    nStart_shadow = nStart;
    }
  }

//-----------------------------------------------------------------------------
void setup() {
  iCount = 0;
  opMd = OPM_BOOT;
  byteCnt = 0;

  Serial.begin(9600);


  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  //---Initialize rx input instances and make pin assignments
  rtsThrottle = rxToSrvo("Throttle", RX_THR_PIN);
  rtsAil = rxToSrvo("Ail", RX_AIL_PIN);
  rtsRudder = rxToSrvo("Rudder", RX_RUD_PIN);
  rtsEle = rxToSrvo("Ele", RX_ELE_PIN);
  rtsGear = rxToSrvo("Gear", RX_GEAR_PIN);
  rtsAux = rxToSrvo("Aux", RX_AUX_PIN);
  // pin 19 - i2c scl
  // pin 18 - i2c sda



  // serPrntNL("init ina260..");
  // pwrSensorOnline = i2w.init();
  // if (pwrSensorOnline) {
  //   serPrntNL("ina260 OK!");
  //   }
  // else {
  //   Serial.println("Init ina260 FAIL!");
  //   }


  // if (!ledmatrix.begin()) {
  //   serPrntNL("IS31 not found");
  //   ledMatrixOnline = false;
  //   }
  // else {
  //   ledMatrixOnline = true;
  //   serPrntNL("IS31 found!");
  //   }

  // if (!bno.begin()) {
  //   serPrntNL("No BNO055 found");
  //   imuOnline = false;
  //   }
  // else {
  //   serPrntNL("BNO055 found!");
  //   imuOnline = true;
  //   bno.setExtCrystalUse(true);
  //   serPrntSensorDetails();
  //   }


  //--Attach interrupts to rx signal pins
  serPrntNL("attach ISRs");
  attachInterrupt(digitalPinToInterrupt(rtsThrottle.rtsRxPinIn), (pInterruptFunc)isrThr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rtsAil.rtsRxPinIn), (pInterruptFunc)isrAil, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rtsEle.rtsRxPinIn), (pInterruptFunc)isrEle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rtsRudder.rtsRxPinIn), (pInterruptFunc)isrRudder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rtsGear.rtsRxPinIn), (pInterruptFunc)isrGear, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rtsAux.rtsRxPinIn), (pInterruptFunc)isrAux, CHANGE);
  serPrntNL("attach ISRs: done");

  // delay(250);

  serPrntNL("setup dig outs");
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  //---pins 18-23 are used for rx inputs

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  serPrntNL("setup dig outs: done");

  opMd = OPM_START;
  byteCnt = 0;
  inStr = "";

  }

//-----------------------------------------------------------------------------
void serPrntSensorDetails(void) {
  // sensor_t sensor;
  // bno.getSensor(&sensor);
  // Serial.println("------------------------------------");
  // Serial.print("Sensor:       "); Serial.println(sensor.name);
  // Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  // Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  // Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  // Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  // Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  // Serial.println("------------------------------------");
  // Serial.println("");
  // delay(500);
  }


//-----------------------------------------------------------------------------
void serPrnt9DOF() {
  /* The processing sketch expects data as roll, pitch, heading */

  // Serial.print(F("Orientation: x"));
  // Serial.print((float)event.orientation.x);
  // Serial.print(F(" y"));
  // Serial.print((float)event.orientation.y);
  // Serial.print(F(" z"));
  // Serial.print((float)event.orientation.z);
  // Serial.print(F(" "));

  // /* Also send calibration data for each sensor. */
  // uint8_t sys, gyro, accel, mag = 0;
  // bno.getCalibration(&sys, &gyro, &accel, &mag);
  // Serial.print(F(" Cal qual (0 to 3): sys"));
  // Serial.print(sys, DEC);
  // Serial.print(F(" gyro:"));
  // Serial.print(gyro, DEC);
  // Serial.print(F(" accel:"));
  // Serial.print(accel, DEC);
  // Serial.print(F(" mag:"));
  // Serial.print(mag, DEC);
  }


//-----------------------------------------------------------------------------
void serPrntRtsObj(rxToSrvo& nRts) {
  serPrntVNL("<rtsName", nRts.rtsName, "");
  serPrntVNL("   rtsRxPinIn", nRts.rtsRxPinIn, "");
  serPrntVNL("   rtsSvoPinOut", nRts.rtsSvoPinOut, "");
  serPrntVNL("   rtsMode", nRts.rtsMode, "");
  serPrntVNL("   rtsMid", nRts.rtsMid, "");
  serPrntVNL("   rtsStart", nRts.rtsStart, "");
  serPrntVNL("   rtsDrtsDWidthStart", nRts.rtsDWidth, "");
  serPrntVNL("   rtsWidth", nRts.rtsWidth, "");
  serPrntVNL("   rtsWidth_Shadow", nRts.rtsWidth_Shadow, "");
  serPrntVNL("   rtsDegrees", nRts.rtsDegrees, "");
  serPrntVNL("   rtsRxVal", nRts.rtsRxVal, "");
  serPrntVNL("   rtsSvoCmdPreMix", nRts.rtsSvoCmdPreMix, "");
  serPrntVNL("   rtsMixComponent", nRts.rtsMixComponent, "");
  serPrntVNL("   rtsSvoCmd", nRts.rtsSvoCmd, "");
  serPrntVNL("   rtsGain", nRts.rtsGain, "");
  serPrntVNL("   rtsOffset", nRts.rtsOffset, "");
  serPrntVNL("   rtsDir", nRts.rtsDir, "");
  serPrntVNL("   rtsDataStale", nRts.rtsDataStale, ">");
  }

//-----------------------------------------------------------------------------
void serPrntRtsWidth() {
  serPrnt(" Rx:TAERG1");
  serPrntV("gearSwPos", gearSwPos, "");
  serPrntV("gearSwPos", gearSwPos, "");
  serPrntV("auxSwPos", auxSwPos, "");
  serPrntV(" t", rtsThrottle.rtsWidth, "");
  serPrntV(" a", rtsAil.rtsWidth, "");
  serPrntV(" e", rtsEle.rtsWidth, "");
  serPrntV(" r", rtsRudder.rtsWidth, "");
  serPrntV(" ax", rtsAux.rtsWidth, "");
  serPrntV(" g", rtsGear.rtsWidth, "");
  }


//-----------------------------------------------------------------------------
void taskSerialOut() {
  String _tmpStr = "";

  if (iCount % 500 == 0) {
    serPrntV("ic", iCount, "");
    serPrntV("opMd", opMd, "");
    serPrntV("prnt", serPrntModeStrs[serPrntMode], " ");

    switch (serPrntMode) {
        case SP_MODE_DOUT:
          serPrntV("LED CTRL: gSwPos", gearSwPos, "");
          serPrntV("aSwPos", auxSwPos, "|");

          serPrntV("do0", pin0);
          serPrntV("do1", pin1);
          serPrntV("do2", pin2);
          serPrntV("do3", pin3);
          serPrntV("do13", pin13);
          serPrntV("do14", pin14);
          serPrntV("do15", pin15);
          serPrntNL();
          break;

        case SP_MODE_I2C:
          serPrnt("I2C:");
          serPrntV("pwrSensorOnline", pwrSensorOnline, ">");
          serPrntV("compassOnline", compassOnline, ">");
          serPrntV("ledMatrixOnline", ledMatrixOnline, ">");
          serPrntV("imuOnline", imuOnline, ">");
          serPrntNL();
          break;


        case SP_MODE_9DOF:
          serPrnt9DOF();
          serPrntNL();

          break;

        default:
        case SP_MODE_OP:

          _tmpStr += "OP:iLL:",
          _tmpStr += rxInitLostLink;
          _tmpStr += " LL:",
          _tmpStr += rxLostLink;
          // _tmpStr += " pLL:",
          // _tmpStr += rxProbableLL;
          _tmpStr += " V:";
          _tmpStr += mainBattV;
          _tmpStr += " pwr";
          _tmpStr += powerStateStrs[powerState];
          _tmpStr += " taer:";
          _tmpStr += rtsThrottle.rtsWidth;
          _tmpStr += "|",
          _tmpStr += rtsAil.rtsWidth;
          _tmpStr += "|";
          _tmpStr += rtsEle.rtsWidth;
          _tmpStr += "|";
          _tmpStr += rtsRudder.rtsWidth;

          _tmpStr += " <do 0:";
          _tmpStr += pin0;
          _tmpStr += " 1:";
          _tmpStr += pin1;
          _tmpStr += " 2:";
          _tmpStr += pin2;
          _tmpStr += " 3:";
          _tmpStr += pin3;
          _tmpStr += " 13:";
          _tmpStr += pin13;
          _tmpStr += " 14:";
          _tmpStr += pin14;
          _tmpStr += " 15:";
          _tmpStr += pin15;
          _tmpStr += ">";
          serPrnt(_tmpStr);
          serPrntNL("");
          break;

        case SP_MODE_WH_SVO_OUT:
          serPrnt("SPEED:");
          serPrntV("thr cmd", thrD, "|");
          serPrntV("str cmd", strD, "|");

          serPrntVF("dLF", dSp_LF);
          serPrntVF("dRF", dSp_RF);
          serPrntNL();
          break;

        case SP_MODE_POWER:
          // serPrntV("CPU TEMP", intTempC, "C");
          // serPrntV("low batt:", lowBattVolt, "");
          // i2w.serPrntPowerData(I2W_PRNT_MD_FULL);
          serPrntNL();
          break;

        case SP_MODE_SVO_TEST:
          serPrnt("wheel:");
          serPrntVF("th", rtsAil.rtsWidth);
          serPrntVF("ail", rtsThrottle.rtsWidth);
          serPrntVF("L", svoCmdWhLF_pwm);
          serPrntVF("R", svoCmdWhRF_pwm);
          serPrntV("thr cmd", thrD, "|");
          serPrntV("str cmd", strD, "|");
          serPrntNL();

          serPrnt("st:");
          break;
      }
    }
  }

//-----------------------------------------------------------------------------
String recvWithEndMarker() {
  static int bCnt = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0) {
    rc = Serial.read();
    if (bCnt < NUM_BYTES - 1) {
      if (rc != endMarker) {
        inByteBuffer[bCnt] = rc;
        }
      bCnt++;
      }
    else
      serPrntNL("buffer overflow");
    }

  if (bCnt > 0) {
    serPrntVNL("Rx'ed ", bCnt, " bytes");
    inByteBuffer[bCnt] = '\0';
    inStr = inByteBuffer;
    }
  else
    inStr = "";

  bCnt = 0;

  return inStr;
  }

//-----------------------------------------------------------------------------
void taskHandleDoutCmds() {
  // serPrntVNL("handle dout cmds", inStr, "");
  if (inStr == "do0") {
    serPrntNL("do0: toggle pin0");
    pin0 = !pin0;
    }
  else if (inStr == "do1") {
    serPrntNL("do1: toggle pin1");
    pin1 = !pin1;
    }
  else if (inStr == "do2") {
    serPrntNL("do2: toggle pin2");
    pin2 = !pin2;
    }
  else if (inStr == "do3") {
    serPrntNL("do3: toggle pin3");
    pin3 = !pin3;
    }
  else if (inStr == "do13") {
    serPrntNL("do13: toggle pin13");
    pin13 = !pin13;
    }
  else if (inStr == "do14") {
    serPrntNL("do14: toggle pin14");
    pin14 = !pin14;
    }
  else if (inStr == "do15") {
    serPrntNL("do15: toggle pin15");
    pin15 = !pin15;
    }

  else if (inStr == "dall") {
    serPrntNL("dall: toggle all");
    pin0 = !pin0;
    pin1 = !pin1;
    pin2 = !pin2;
    pin3 = !pin3;

    pin14 = !pin14;
    pin15 = !pin15;

    }
  }

//-----------------------------------------------------------------------------
//  Handles data recieved from the serial port and execute command, if
//  present
//-----------------------------------------------------------------------------
void taskHandleSerIn() {
  if (recvWithEndMarker() > "") {
    if (inStr == "m+") {
      serPrntNL("m+: inc mode");
      opMd++;
      }
    else if (inStr == "m-") {
      serPrntNL("m+: inc mode");
      opMd--;
      }
    else if (inStr == "help") {
      serPrntNL("help: print out handled serial terminalm commands");
      opMd--;
      }
    else if (inStr == "prts") {
      serPrntNL("prts: Print rts obj params");
      serPrntRtsObj(rtsThrottle);
      serPrntRtsObj(rtsAil);
      serPrntRtsObj(rtsEle);
      serPrntRtsObj(rtsRudder);
      serPrntRtsObj(rtsGear);
      serPrntRtsObj(rtsAux);

      }

    else if (inStr == "spmp") {
      serPrntNL("spmp: serial print POWER mode");
      serPrntMode = SP_MODE_COMPASS;
      }

    else if (inStr == "spm+") {
      serPrntNL("spm+: inc serial print mode");
      serPrntMode++;
      if (serPrntMode >= NUM_SP_MODES)
        serPrntMode = SP_MODE_COMPASS;
      }
    else if (inStr == "spm-") {
      serPrntNL("spm-: dec serial print mode");
      serPrntMode--;
      if (serPrntMode < 0)
        serPrntMode = NUM_SP_MODES;

      }
    else if (inStr == "w_lf+")
      svoCmdWhLF += 5;
    else if (inStr == "w_lf-")
      svoCmdWhLF -= 5;

    else {
      taskHandleDoutCmds();
      }

    inStr = "";
    }
  }

//-----------------------------------------------------------------------------
// OPM_BOOT,
// OPM_START,
// OPM_LINK_NOM,
// OPM_INIT_LL,
// OPM_LL,
// OPM_ILL_LL,
// OPM_USB_LINK_NOM,
// OPM_USB_INIT_LL,
// OPM_USB_LL,
// OPM_USB_ILL_LL,
// OPM_SLEEP,
// NUM_OP_MODES,
//-----------------------------------------------------------------------------
void taskOpMode() {
  //---enfore op mode
  if (iCount % 5000) {
    // serPrntNL("taskOpMode()");
    switch (opMd) {
        default:
        case OPM_BOOT:
        case OPM_START:
          break;

        case OPM_LINK_NOM:

          break;

        case OPM_INIT_LL:

          break;

        case OPM_LL:

          break;

        case OPM_ILL_LL:

          break;

        case OPM_USB_LINK_NOM:

          break;

        case OPM_USB_INIT_LL:

          break;

        case OPM_USB_LL:

          break;

        case OPM_USB_ILL_LL:

          break;

        case OPM_SLEEP:

          break;

        case OPM_SVO_TEST:

          break;

      }
    }
  }

//-----------------------------------------------------------------------------
void taskDetectLostLink() {

  if (iCount % 5000) {
    if (rtsThrottle.rtsWidth < MIN_PULSE_WIDTH ||
      rtsThrottle.rtsWidth > MAX_PULSE_WIDTH ||
      rtsAil.rtsWidth < MIN_PULSE_WIDTH ||
      rtsAil.rtsWidth > MAX_PULSE_WIDTH)
      rxLostLink = true;
    else {
      rxLostLink = false;

    }
  }
}


//-----------------------------------------------------------------------------
void taskRxToSvoOut() {
  static bool llShadow = false;
  static bool illShadow = false;
  static float _svoCmdWhLF = 0;
  static float _svoCmdWhRF = 0;


  //---calculate throttle cmd delta
  strD = 1500.0 - (float)rtsAil.rtsWidth;
  thrD = 1500.0 - (float)rtsThrottle.rtsWidth;

  _svoCmdWhLF = -(strD - thrD) / 2;
  _svoCmdWhLF += 1500;

  _svoCmdWhRF = (strD + thrD) / 2;
  _svoCmdWhRF += 1500;

  spState = 2;
  panState = 2;
  tiltState = 2;

  svoCmdWhLF_pwm = map(_svoCmdWhLF, 1000.0, 2000.0, 170, 420);
  svoCmdWhRF_pwm = map(_svoCmdWhRF, 1000.0, 2000.0, 170, 420);

  pwm.setPWM(0, 0, svoCmdWhLF_pwm);
  pwm.setPWM(4, 0, svoCmdWhRF_pwm);

  illShadow = rxInitLostLink;
  llShadow = rxLostLink;
}

//-----------------------------------------------------------------------------
void taskLED() {
  static int gearSwPosShadow = 0;
  static int auxSwPosShadow = 0;


  if (iCount % 25000 == 0) {
    gearSwPos = (rtsGear.rtsWidth > MID_PULSE_WIDTH ? 1 : 0);

    if (rtsAux.rtsWidth < MIN_THIRD_PULSE)
      auxSwPos = 0;
    else if (rtsAux.rtsWidth < MAX_THIRD_PULSE)
      auxSwPos = 1;
    else
      auxSwPos = 2;


    if (auxSwPos != auxSwPosShadow) {
      if (auxSwPos > 0)
        camLEDs = true;
      pin14 = !camLEDs;
      pin15 = !camLEDs;
      }


    if (gearSwPos != gearSwPosShadow) {
      floodLEDs = !gearSwPos;
      pin0 = !floodLEDs;
      pin1 = !floodLEDs;
      }


    gearSwPosShadow = gearSwPos;
    auxSwPosShadow = auxSwPos;
    }
  }
//-----------------------------------------------------------------------------
void taskDOUT() {
  //---V > 5v: enable normal dig out functionality
  if (rxLostLink || mainBattV < VOLT_SUB_BATT) {
    if (iCount % 1000 == 0) {
      pin13 = !pin13;
      }
    }
  else
    pin13 = HIGH;

  if (mainBattV > VOLT_USB) {
    digitalWrite(0, !pin0);
    digitalWrite(1, !pin1);
    digitalWrite(14, !pin14);
    digitalWrite(15, !pin15);

    }
  //---force all dig outs off when on USB power: prevent brown-outs that mess w/ the RX
  //---blink
  else {

    digitalWrite(0, HIGH);
    digitalWrite(1, HIGH);
    digitalWrite(14, HIGH);
    digitalWrite(15, HIGH);
    }

  digitalWrite(13, !pin13);
  }


//-----------------------------------------------------------------------------
// Board layout:
//       +----------+
//       |         *| RST   PITCH  ROLL  HEADING
//   ADR |*        *| SCL
//   INT |*        *| SDA     ^            /->
//   PS1 |*        *| GND     |            |
//   PS0 |*        *| 3VO     Y    Z-->    \-X
//       |         *| VIN
//       +----------+
//
//-----------------------------------------------------------------------------
void task9DOF() {

  // if (iCount % 25000 == 0) {
  //   /* Get a new sensor event */
  //   // sensors_event_t event;
  //   bno.getEvent(&event);


  //   }


  }

//-----------------------------------------------------------------------------
void taskAnalyzeInput() {
  if (iCount % 5000 == 0) {
    if (rtsAil.rtsWidth < MID_PULSE_WIDTH) {
      strInLeft = true;
      strInRight = false;
      }
    else {
      strInLeft = false;
      strInRight = true;
      }
    }

  }


//-----------------------------------------------------------------------------
// VOLT_SUB_USB
// VOLT_LOW_USB
// VOLT_USB
// VOLT_SUB_BATT

// PWR_STATE_BOOT,
// PWR_STATE_SUB_USB,
// PWR_STATE_LOW_USB,
// PWR_STATE_USB,
// PWR_STATE_SUB_BATT,
// PWR_STATE_NOM_BATT,
//-----------------------------------------------------------------------------
void taskGetPowerData() {
  // if (pwrSensorOnline) {
  //   if (iCount % 25000 == 0) {
  //     i2w.getData();
  //     if(i2w.i2wError)
  //       pwrSensorOnline = false;

  //     mainBattV = i2w.i2wVolt;
  //     mainBattCurrent = i2w.i2wCurrent;
  //     sysPower = i2w.i2wPower;

  //   }

  //   if ((mainBattV < VOLT_SUB_BATT) & (mainBattV >= VOLT_USB))
  //     powerState = PWR_STATE_SUB_BATT;
  //   else if (mainBattV < VOLT_USB && mainBattV >= VOLT_LOW_USB)
  //     powerState = PWR_STATE_USB;
  //   else if (mainBattV < VOLT_USB && mainBattV >= VOLT_SUB_USB)
  //     powerState = PWR_STATE_LOW_USB;
  //   else if (mainBattV < VOLT_SUB_USB)
  //     powerState = PWR_STATE_SUB_USB;
  //   else
  //     powerState = PWR_STATE_NOM_BATT;

  // }
  // else {
  //   powerState = PWR_STATE_OFLINE;
  //   mainBattV = 5000;
  //   mainBattCurrent = 0;
  //   sysPower = 0;
  // }



  }

//-----------------------------------------------------------------------------
void taskAcquireInternalData() {
  // if (iCount % 100000 == 0)
  //   intTempC = InternalTemperature.readTemperatureC();
  }

//-----------------------------------------------------------------------------
void loop() {
  taskHandleSerIn();
  // task9DOF();
  // taskGetPowerData();
  // taskAcquireInternalData();
  // taskAnalyzeInput();
  taskOpMode();
  taskLED();
  taskDetectLostLink();
  taskDOUT();
  taskRxToSvoOut();
  taskSerialOut();

  iCount++;
  }


//-----------------------------------------------------------------------------
//   calculates the off time to determine pwm dwell
//-----------------------------------------------------------------------------
void isrThr(void) { rtsThrottle.rtsIsr(); }
void isrAil(void) { rtsAil.rtsIsr(); }
void isrEle(void) { rtsEle.rtsIsr(); }
void isrRudder(void) { rtsRudder.rtsIsr(); }
void isrGear(void) { rtsGear.rtsIsr(); }
void isrAux(void) { rtsAux.rtsIsr(); }
