#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"

///////////////////////////////////

#if __has_include("build_version.h")
#include "build_version.h"
#endif

#if __has_include("reeltwo_build_version.h")
#include "reeltwo_build_version.h"
#endif

///////////////////////////////////

#ifdef ESP32
#if !defined(ROAM_A_DOME_FULLSIZE_PCB) && \
    !defined(ROAM_A_DOME_COMPACT_PCB) && \
    !defined(LILYGO_MINI32) && \
    !defined(ROAM_A_DOME_DISPLAY)
#error Make sure you select the correct PCB and remove this error
//#define ROAM_A_DOME_FULLSIZE_PCB
#define ROAM_A_DOME_COMPACT_PCB
//#define LILYGO_MINI32
#endif
#elif defined(__AVR_ATmega2560__)
#define ROAM_A_DOME_MEGA_PCB
#elif defined(ARDUINO_ARCH_LINUX)
// Simulator no pins
#else
#error Unsupported platform
#endif

///////////////////////////////////

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

#define USE_DEBUG                     // Define to enable debug diagnostic
#if defined(ROAM_A_DOME_FULLSIZE_PCB) || defined(ROAM_A_DOME_MEGA_PCB)
#define USE_LCD_SCREEN                // Define if using LCD and Rotary encoder
#endif
#define USE_SERVOS                    // Define is enabling servo output on digital out pins
// #define USE_SERVO_DEBUG
// #define USE_VERBOSE_SERVO_DEBUG
#define USE_DOME_DEBUG                // Define for dome drive mode debug
#ifdef ESP32
#define USE_VERBOSE_DOME_DEBUG        // Define for dome motor specific debug
#endif
#undef  USE_DOME_SENSOR_SERIAL_DEBUG  // Define for dome sensor ring specific debug
#undef  USE_SCREEN_DEBUG
#ifdef ESP32
#define USE_DROID_REMOTE              // Define for droid remote support
#define USE_WIFI
#define USE_SPIFFS
#define USE_PREFERENCES
#endif

#if defined(USE_LCD_SCREEN) || defined(USE_DROID_REMOTE)
#define USE_MENUS                     // Define if using menu system
#endif

#ifdef USE_WIFI
#define USE_MDNS
#define USE_OTA
#define USE_WIFI_WEB
#define USE_WIFI_MARCDUINO
#endif

///////////////////////////////////

#ifdef USE_DROID_REMOTE
#define REMOTE_ENABLED                  false
#define SMQ_HOSTNAME                    "RoamADome"
#define SMQ_SECRET                      "Astromech"
#endif
#ifdef USE_WIFI
#define WIFI_ENABLED                    true
// Set these to your desired credentials.
#define WIFI_AP_NAME                    "RoamADome"
#define WIFI_AP_PASSPHRASE              "Astromech"
#define WIFI_ACCESS_POINT               true  /* true if access point: false if joining existing wifi */

#define MARC_SERIAL_BAUD_RATE           9600
#define MARC_WIFI_ENABLED               true
#define MARC_WIFI_SERIAL_PASS           true
#endif

#define SETUP_MAX_ANGULAR_VELOCITY      45 /* cm/s */
#define SETUP_VELOCITY_START            40
#define SETUP_VELOCITY_INCREMENT        10
#define DEFAULT_HOME_POSITION           0
#define DEFAULT_SABER_BAUD              9600
#define DEFAULT_SERIAL_BAUD             9600
#define DEFAULT_PACKET_SERIAL_INPUT     true
#define DEFAULT_PACKET_SERIAL_OUTPUT    true
#define DEFAULT_PWM_INPUT               false
#define DEFAULT_PWM_OUTPUT              false
#define DEFAULT_MAX_SPEED               50
#define DEFAULT_RANDOM_MODE             false
#define DEFAULT_HOME_MODE               false
#define DEFAULT_SPEED_SCALING           false
// If true no automatic movement will happen until the dome has been moved by joystick first.
// If false automatic movement can happen on startup.
#define DEFAULT_AUTO_SAFETY             true
#define DEFAULT_INVERTED                false
#define DEFAULT_PWM_MIN_PULSE           1000
#define DEFAULT_PWM_MAX_PULSE           2000
#define DEFAULT_PWM_NEUTRAL_PULSE       1500
#define DEFAULT_PWM_DEADBAND            5
#define DEFAULT_ACCELERATION_SCALE      20
#define DEFAULT_DECELERATION_SCALE      50
#define DEFAULT_DOME_HOME_MIN_DELAY     6
#define DEFAULT_DOME_HOME_MAX_DELAY     8
#define DEFAULT_DOME_AUTO_MIN_DELAY     6
#define DEFAULT_DOME_AUTO_MAX_DELAY     8
#define DEFAULT_DOME_TARGET_MIN_DELAY   0
#define DEFAULT_DOME_TARGET_MAX_DELAY   1
#define DEFAULT_DOME_AUTO_LEFT          80
#define DEFAULT_DOME_AUTO_RIGHT         80
#define DEFAULT_DOME_FUDGE              5
#define DEFAULT_DOME_SPEED_HOME         40
#define DEFAULT_DOME_SPEED_AUTO         30
#define DEFAULT_DOME_SPEED_TARGET       100
#define DEFAULT_DOME_SPEED_MIN          15
#define DEFAULT_DIGITAL_PINS            0
#define DEFAULT_TIMEOUT                 5

#define MAX_SPEED               100
#define MAX_SPEED_F             float(MAX_SPEED)
#define MAX_FUDGE_FACTOR        20
#define MAX_AUTO_LEFT           180
#define MAX_AUTO_RIGHT          180
#define MAX_AUTO_DELAY          255
#define MAX_ACC_SCALE           255
#define MAX_DEC_SCALE           255
#define MAX_TIMEOUT             30
#define MOVEMODE_MAX_INTERVAL   5       // default interval between random commands
#define MAX_COMMANDS            100

#define KEY_REPEAT_RATE_MS      500     // Key repeat rate in milliseconds

#define SYREN_ADDRESS_INPUT     129
#define SYREN_ADDRESS_OUTPUT    129

#define DOME_SENSOR_SERIAL          Serial1
#define CONSOLE_BUFFER_SIZE     300
#define COMMAND_BUFFER_SIZE     256

#ifdef ESP32
// Default dome sensor baud rate is 115200 for ESP32
#define DOME_SENSOR_SERIAL_BAUD 115200
#else
// Default dome sensor baud rate is 57600 for Mega
#define DOME_SENSOR_SERIAL_BAUD 57600
#endif

#define PACKET_SERIAL_TIMEOUT   1500

///////////////////////////////////

#include "pin-map.h"

#if defined(USE_SERVOS) && !defined(PWM_OUTPUT_PIN)
// Disable servo output if PWM_OUTPUT_PIN is not supported on PCB
#undef USE_SERVOS
#endif

///////////////////////////////////

#ifndef DOME_DRIVE_SERIAL_WRITE
 #define DOME_DRIVE_SERIAL          Serial2
 #define DOME_DRIVE_SERIAL_WRITE    Serial2
 #define DOME_DRIVE_SERIAL_READ     Serial2
#endif

///////////////////////////////////

#ifdef USE_DROID_REMOTE
#include "ReelTwoSMQ32.h"
#else
#include "ReelTwo.h"
#endif

#ifdef ARDUINO_ARCH_LINUX
 #define USE_SIMULATOR
 #undef DOME_SENSOR_SERIAL
 #undef DOME_DRIVE_SERIAL
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_LINUX)
 #define EEPROM_SIZE             4096
#endif

///////////////////////////////////

#include "core/AnimatedEvent.h"
#include "core/StringUtils.h"
#include "core/EEPROMSettings.h"
#include "core/PinManager.h"
#include "drive/DomePosition.h"
#include "drive/SerialConsoleController.h"
#include "drive/DomeDrive.h"
#ifndef USE_SIMULATOR
#include "drive/DomeSensorRingSerialListener.h"
#include "drive/DomeDriveSabertooth.h"
#endif
#ifdef PWM_INPUT_PIN
#ifdef ESP32
 #include "encoder/PWMDecoder.h"
#else
 #include "encoder/ServoDecoder.h"
#endif
#endif
#ifdef USE_SERVOS
#include "ServoDispatchDirect.h"
#include "ServoEasing.h"
#endif
#ifdef USE_PREFERENCES
#include <Preferences.h>
#define PREFERENCE_REMOTE_ENABLED           "remote"
#define PREFERENCE_REMOTE_HOSTNAME          "rhost"
#define PREFERENCE_REMOTE_SECRET            "rsecret"
#define PREFERENCE_WIFI_ENABLED             "wifi"
#define PREFERENCE_WIFI_SSID                "ssid"
#define PREFERENCE_WIFI_PASS                "pass"
#define PREFERENCE_WIFI_AP                  "ap"

#define PREFERENCE_MARCWIFI_ENABLED         "mwifi"
#define PREFERENCE_MARCWIFI_SERIAL_PASS     "mwifipass"
#endif
#ifdef USE_OTA
#include <ArduinoOTA.h>
#endif
#ifdef USE_SPIFFS
#include "SPIFFS.h"
#define USE_FS SPIFFS
#elif defined(USE_FATFS)
#include "FFat.h"
#define USE_FS FFat
#elif defined(USE_LITTLEFS)
#include "LITTLEFS.h"
#define USE_FS LITTLEFS
#endif
#ifdef ESP32
#include "FS.h"
#endif

///////////////////////////////////

#ifdef USE_LVGL_DISPLAY
#include "core/PushButton.h"
#include "TFT_eSPI.h"
#include "lvgl.h"

#define SCREEN_WIDTH            320     // OLED display width, in pixels
#define SCREEN_HEIGHT           170     // OLED display height, in pixels
#define SCREEN_BUFFER_SIZE      (SCREEN_WIDTH * SCREEN_HEIGHT)

#define LV_DELAY(x) {                                               \
  uint32_t start = millis();                                        \
  do {                                                              \
    lv_timer_handler();                                             \
    delay(1);                                                       \
  } while (millis() < start + (x));                                 \
}

TFT_eSPI tft = TFT_eSPI();
static lv_disp_drv_t disp_drv;      // contains callback functions
static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_color_t *lv_disp_buf;
PushButton button1(PIN_BUTTON_1, true);
PushButton button2(PIN_BUTTON_2, true);

////////////////////////////////

LV_FONT_DECLARE(lv_font_Astromech);
LV_FONT_DECLARE(lv_font_montserrat_32);
LV_IMG_DECLARE(r2d2_gif);

////////////////////////////////

static const lv_font_t* font_large = &lv_font_Astromech;
static const lv_font_t* font_normal = &lv_font_montserrat_32;

#endif

///////////////////////////////////

// STATUSLED cannot support PWM INPUT until Adafruit_NeoPixel is fixed or replaced
#undef STATUSLED_PIN
#ifdef STATUSLED_PIN
#include "core/SingleStatusLED.h"

enum {
    kNormalMode = 0,
    kWifiMode = 1,
    kRemoteMode = 2,
    kNormalMovingMode = 3,
    kWifiMovingMode = 4,
    kRemoteMovingMode = 5
};
unsigned sCurrentMode = kNormalMode;
static constexpr uint8_t kStatusColors[][4][3] = {
      { {  0,   2,    0} , {   0,    2,    0} , {  0,   2,    0} , {   0,    2,    0}  },  // normal (all green)
      { {  0,   0,    2} , {   0,    0,    2} , {  0,   0,    2} , {   0,    0,    2}  },  // wifi enabled (all blue)
      { {  0,   2,    2} , {   0,    2,    2} , {  0,   2,    2} , {   0,    2,    2}  },  // remote enabled (all yellow)
      { {  0,  10,    0} , {  10,    0,    0} , {  0,  10,    0} , {  10,    0,    0}  },  // green,red,green,red
      { {  0,   0,   10} , {  10,    0,    0} , {  0,   0,   10} , {  10,    0,    0}  },  // blue,red,blue,red
      { {  0,  10,   10} , {  10,    0,    0} , {  0,  10,   10} , {  10,    0,    0}  }   // yellow,red,yellow,red
};
typedef SingleStatusLED<STATUSLED_PIN> StatusLED;
StatusLED statusLED(kStatusColors, SizeOfArray(kStatusColors));
#endif

///////////////////////////////////

#ifdef USE_WIFI
#include "wifi/WifiAccess.h"
#endif
#ifdef USE_MDNS
#include <ESPmDNS.h>
#endif
#ifdef USE_WIFI_WEB
#include "wifi/WifiWebServer.h"
#include "web-images.h"
#endif
#ifdef USE_WIFI_MARCDUINO
#include "wifi/WifiMarcduinoReceiver.h"
#endif

///////////////////////////////////

#ifdef USE_I2C_GPIO_EXPANDER
#include "PCF8574.h"
#ifndef GPIO_EXPANDER_ADDRESS
#define GPIO_EXPANDER_ADDRESS 0x20
#endif

class CustomPinManager : public PinManager
{
public:
    CustomPinManager(byte i2cAddress = GPIO_EXPANDER_ADDRESS) :
        fGPIOExpander(i2cAddress)
    {}

    virtual void begin() override
    {
        fGPIOExpander.begin();
    }

    virtual bool digitalRead(uint8_t pin) override
    {
        if (pin >= GPIO_PIN_BASE)
        {
            return fGPIOExpander.digitalRead(pin-GPIO_PIN_BASE, true);
        }
        return PinManager::digitalRead(pin);
    }
    virtual void digitalWrite(uint8_t pin, uint8_t val) override
    {
        if (pin >= GPIO_PIN_BASE)
        {
            fGPIOExpander.digitalWrite(pin-GPIO_PIN_BASE, val);
        }
        else
        {
            PinManager::digitalWrite(pin, val);
        }
    }
    virtual void pinMode(uint8_t pin, uint8_t mode) override
    {
        if (pin >= GPIO_PIN_BASE)
        {
            fGPIOExpander.pinMode(pin-GPIO_PIN_BASE, mode);
        }
        else
        {
            PinManager::pinMode(pin, mode);
        }
    }

protected:
    PCF8574 fGPIOExpander;
};
CustomPinManager sPinManager;
#else
PinManager sPinManager;
#endif

///////////////////////////////////

#ifdef USE_WIFI
WifiAccess wifiAccess;
bool wifiEnabled;
bool wifiActive;
#endif

#ifdef USE_DROID_REMOTE
bool remoteEnabled;
bool remoteActive;
#endif

#ifdef USE_WIFI_MARCDUINO
WifiMarcduinoReceiver wifiMarcduinoReceiver(wifiAccess);
#endif

#ifdef USE_PREFERENCES
Preferences preferences;
#endif

#ifdef USE_WIFI
TaskHandle_t eventTask;
#endif

#ifdef USE_OTA
bool otaInProgress;
#endif

bool sDomeHasMovedManually = false;

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_WIFI
String getHostName()
{
    String mac = wifiAccess.getMacAddress();
    String hostName = mac.substring(mac.length()-5, mac.length());
    hostName.remove(2, 1);
    hostName = WIFI_AP_NAME+String("-")+hostName;
    return hostName;
}
#endif

///////////////////////////////////

bool mountReadOnlyFileSystem()
{
#ifdef USE_SPIFFS
    return (SPIFFS.begin(true));
#endif
    return false;
}

void unmountFileSystems()
{
#ifdef USE_SPIFFS
    SPIFFS.end();
#endif
}

///////////////////////////////////

#ifdef USE_MENUS

#include "Screens.h"
#include "menus/CommandScreen.h"

#ifdef USE_LCD_SCREEN

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define SCREEN_ADDRESS 0x3C

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "menus/CommandScreenDisplay.h"
Adafruit_SSD1306 sLCD(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
CommandScreenDisplay<Adafruit_SSD1306> sDisplay(sLCD, sPinManager, []() {
    return sLCD.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
});

#else

#include "menus/CommandScreenHandlerSMQ.h"
CommandScreenHandlerSMQ sDisplay;

#endif

#endif

byte sDigitalPin[] = {
#ifdef DOUT1_PIN
    DOUT1_PIN
#endif
#ifdef DOUT2_PIN
    ,DOUT2_PIN
#endif
#ifdef DOUT3_PIN
    ,DOUT3_PIN
#endif
#ifdef DOUT4_PIN
    ,DOUT4_PIN
#endif
#ifdef DOUT5_PIN
    ,DOUT5_PIN
#endif
#ifdef DOUT6_PIN
    ,DOUT6_PIN
#endif
#ifdef DOUT7_PIN
    ,DOUT7_PIN
#endif
#ifdef DOUT8_PIN
    ,DOUT8_PIN
#endif
};

////////////////////////////////

static unsigned sSerialBaudRates[] = {
    2400,
    9600,
    19200,
    38400
};

#ifdef USE_WIFI_WEB
static String sSerialBaudRatesStr[] = {
    "2400",
    "9600",
    "19200",
    "38400"
};
#endif

////////////////////////////////

#ifdef USE_SERVOS
// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.
//
//   Pin  Group ID,      Min,  Max
const ServoSettings sServoSettings[] PROGMEM = {
    { PWM_OUTPUT_PIN,    1000, 2000, 0 }
#ifdef DOUT1_PIN
    ,{ DOUT1_PIN,        1000, 2000, 0 }
#endif
#ifdef DOUT2_PIN
    ,{ DOUT2_PIN,        1000, 2000, 0 }
#endif
#ifdef DOUT3_PIN
    ,{ DOUT3_PIN,        1000, 2000, 0 }
#endif
#ifdef DOUT4_PIN
    ,{ DOUT4_PIN,        1000, 2000, 0 }
#endif
#ifdef DOUT5_PIN
    ,{ DOUT5_PIN,        1000, 2000, 0 }
#endif
#ifdef DOUT6_PIN
    ,{ DOUT6_PIN,        1000, 2000, 0 }
#endif
#ifdef DOUT7_PIN
    ,{ DOUT7_PIN,        1000, 2000, 0 }
#endif
#ifdef DOUT8_PIN
    ,{ DOUT8_PIN,        1000, 2000, 0 }
#endif
};

ServoDispatchDirect<SizeOfArray(sServoSettings)> sServoDispatch(sServoSettings);
#endif

////////////////////////////////

#ifdef USE_SIMULATOR
class DomeSensorRingEmulator: public DomePositionProvider
{
public:
    DomeSensorRingEmulator()
    {
    }

    inline unsigned getErrorCount()
    {
        return 0;
    }

    virtual bool ready() override
    {
        return true;
    }

    int normalize(int degrees)
    {
        degrees = fmod(degrees, 360);
        if (degrees < 0)
            degrees += 360;
        return degrees;
    }

    virtual int getAngle() override
    {
        return normalize(fDomePositionDegrees);
    }

    float fDomePositionDegrees = 0;

private:
    bool fReady = false;
};

DomeSensorRingEmulator sDomeRing;
#else
DomeSensorRingSerialListener sDomeRing(DOME_SENSOR_SERIAL);
#endif
DomePosition sDomePosition(sDomeRing);

static void restoreDomeSettings();

class SerialDomeController : public SerialConsoleController, public AnimatedEvent
{
public:
    SerialDomeController(Stream& serial) :
        SerialConsoleController(serial)
    {}

    virtual void notify() override
    {
        if (isEmulationActive())
        {
            sDomeHasMovedManually = true;
            restoreDomeSettings();
        }
    }

    virtual void animate() override
    {
        switch (read())
        {
            case kHome:
                DEBUG_PRINTLN(F("HOME"));
                sDomePosition.setDomeDefaultMode(DomePosition::kHome);
                break;
            case kEnd:
                DEBUG_PRINTLN(F("OFF"));
                sDomePosition.setDomeDefaultMode(DomePosition::kOff);
                disconnect();
                break;
            case kPageUp:
                increaseSpeed();
                DEBUG_PRINT(F("SPEED: ")); DEBUG_PRINTLN(floor(getSpeed()*MAX_SPEED));
                break;
            case kPageDown:
                decreaseSpeed();
                DEBUG_PRINT(F("SPEED: ")); DEBUG_PRINTLN(floor(getSpeed()*MAX_SPEED));
                break;
            case 'R':
            case 'r':
                DEBUG_PRINTLN(F("RANDOM"));
                sDomePosition.setDomeDefaultMode(DomePosition::kRandom);
                break;
        }
    }
};
SerialDomeController sDomeStick(Serial);

#if defined(DOME_DRIVE_SOFT_SERIAL) || defined(COMMAND_SOFT_SERIAL)
#include "SoftwareSerial.h"
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_SERVOS
class DomeDriveSabertoothPWM : public DomeDriveSabertooth
{
public:
    DomeDriveSabertoothPWM(
            int id,
            Stream& serial,
            ServoDispatch& dispatch,
            uint8_t pwmNum,
            JoystickController& domeStick) :
        DomeDriveSabertooth(id, serial, domeStick),
        fDispatch(dispatch),
        fPWM(pwmNum)
    {
    }

    inline void setOutput(bool pulseOutput, bool packetOutput)
    {
        if ((fPulseOutput && !pulseOutput) ||
            (fPacketOutput && !packetOutput))
        {
            // Disabling either pulse or packet output. Motor might be
            // moving so lets make sure it is stopped.
            stop();
        }
        fPulseOutput = pulseOutput;
        fPacketOutput = packetOutput;
    }

    virtual void stop() override
    {
        if (fPacketOutput)
        {
            DomeDriveSabertooth::stop();
        }
        if (fPulseOutput)
        {
            fDispatch.moveTo(fPWM, 0.5);
        }
    }

protected:
    ServoDispatch& fDispatch;
    uint8_t fPWM;
    bool fPacketOutput = false;
    bool fPulseOutput = false;

    static float map(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    virtual void motor(float m) override
    {
        if (fPacketOutput)
        {
            DomeDriveSabertooth::motor(m);
        }
        if (fPulseOutput)
        {
            fDispatch.moveTo(fPWM, map(m, -1.0f, 1.0f, 0.0f, 1.0f));
        }
    }
};
#endif

///////////////////////////////////////////////////////////////////////////////


#ifdef USE_SIMULATOR
class DomeDriveEmulator : public DomeDrive
{
public:
    /** \brief Constructor
      *
      * Only a single instance of WifiSerialBridge should be created per sketch.
      *
      * \param port the port number of this service
      */
    DomeDriveEmulator(DomeSensorRingEmulator& domeRing, JoystickController& domeStick) :
        DomeDrive(domeStick),
        fDomeRing(domeRing)
    {
    }

    virtual void setup() override
    {
    }

    virtual void stop() override
    {
        DomeDrive::stop();
    }

    void setBaudRate(unsigned)
    {
    }

    void setAddress(uint8_t)
    {
    }

protected:
    DomeSensorRingEmulator& fDomeRing;
    virtual void motor(float m) override
    {
        static float sLastMotor;
        static int sLastPos;
        float pos = fDomeRing.fDomePositionDegrees;
        if (sLastMotor != m || sLastPos != int(pos))
        {
            if (m == 0)
            {
                printf("                                   \r");
            }
            else
            {
                printf("MOTOR %0.2f POS=%d [%d]            \r", m, normalize(pos), int(pos));
            }
            fflush(stdout);
            // printf("MOTOR %0.2f POS=%d [%d]\n", m, normalize(pos), int(pos));
            sLastMotor = m;
            sLastPos = int(pos);
        }
        fDomeRing.fDomePositionDegrees -= m;
    }
};
DomeDriveEmulator sDomeDrive(sDomeRing, sDomeStick);
#else
 #ifdef DOME_DRIVE_SOFT_SERIAL
  SoftwareSerial DOME_DRIVE_SERIAL;
 #endif
 #ifdef USE_SERVOS
  DomeDriveSabertoothPWM sDomeDrive(SYREN_ADDRESS_OUTPUT, DOME_DRIVE_SERIAL, sServoDispatch, 0, sDomeStick);
 #else
  DomeDriveSabertooth sDomeDrive(SYREN_ADDRESS_OUTPUT, DOME_DRIVE_SERIAL, sDomeStick);
 #endif
#endif

#ifdef COMMAND_SOFT_SERIAL
SoftwareSerial COMMAND_SERIAL;
#endif

#ifdef USE_SERVOS
struct Channel
{
    uint8_t fEasing;
    uint16_t fStartPulse;
    uint16_t fEndPulse;
    uint32_t fGroup;
};
#endif

struct DomeControllerSettings
{
    uint8_t fSaberAddressInput = SYREN_ADDRESS_INPUT;
    uint8_t fSaberAddressOutput = SYREN_ADDRESS_OUTPUT;
    uint16_t fHomePosition = DEFAULT_HOME_POSITION;
    uint32_t fSensorBaudRate = DOME_SENSOR_SERIAL_BAUD;
    uint32_t fSaberBaudRate = DEFAULT_SABER_BAUD;
    uint32_t fSerialBaudRate = DEFAULT_SERIAL_BAUD;
    bool fPacketSerialInput = DEFAULT_PACKET_SERIAL_INPUT;
    bool fPacketSerialOutput = DEFAULT_PACKET_SERIAL_OUTPUT;
    bool fPWMInput = DEFAULT_PWM_INPUT;
    bool fPWMOutput = DEFAULT_PWM_OUTPUT;
    uint8_t fMaxSpeed = DEFAULT_MAX_SPEED;
    uint8_t fTimeout = DEFAULT_TIMEOUT;
    bool fRandomMode = DEFAULT_RANDOM_MODE;
    bool fHomeMode = DEFAULT_HOME_MODE;
    bool fSpeedScaling = DEFAULT_SPEED_SCALING;
    bool fInverted = DEFAULT_INVERTED;
    bool fAutoSafety = DEFAULT_AUTO_SAFETY;
    uint16_t fSetupAngularVelocity = SETUP_MAX_ANGULAR_VELOCITY;
    uint16_t fPWMMinPulse = DEFAULT_PWM_MIN_PULSE;
    uint16_t fPWMMaxPulse = DEFAULT_PWM_MAX_PULSE;
    uint16_t fPWMNeutralPulse = DEFAULT_PWM_NEUTRAL_PULSE;
    uint8_t fPWMDeadbandPercent = DEFAULT_PWM_DEADBAND;
    uint8_t fAccScale = DEFAULT_ACCELERATION_SCALE;
    uint8_t fDecScale = DEFAULT_DECELERATION_SCALE; 
    uint8_t fDomeAutoMinDelay = DEFAULT_DOME_AUTO_MIN_DELAY;
    uint8_t fDomeAutoMaxDelay = DEFAULT_DOME_AUTO_MAX_DELAY;
    uint8_t fDomeHomeMinDelay = DEFAULT_DOME_HOME_MIN_DELAY;
    uint8_t fDomeHomeMaxDelay = DEFAULT_DOME_HOME_MAX_DELAY;
    uint8_t fDomeTargetMinDelay = DEFAULT_DOME_TARGET_MIN_DELAY;
    uint8_t fDomeTargetMaxDelay = DEFAULT_DOME_TARGET_MAX_DELAY;
    uint8_t fDomeAutoLeft = DEFAULT_DOME_AUTO_LEFT;
    uint8_t fDomeAutoRight = DEFAULT_DOME_AUTO_RIGHT;
    uint8_t fDomeFudge = DEFAULT_DOME_FUDGE;
    uint8_t fDomeSpeedHome = DEFAULT_DOME_SPEED_HOME;
    uint8_t fDomeSpeedAuto = DEFAULT_DOME_SPEED_AUTO;
    uint8_t fDomeSpeedTarget = DEFAULT_DOME_SPEED_TARGET;
    uint8_t fDomeSpeedMin = DEFAULT_DOME_SPEED_MIN;
    uint8_t fDigitalPins = DEFAULT_DIGITAL_PINS;

#ifdef USE_SERVOS
    Channel fServos[9] = {
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 },
        { 0, 1000, 2000, 0 }
    };
#endif
};
EEPROMSettings<DomeControllerSettings> sSettings;

///////////////////////////////////////////////////////////////////////////////

#ifdef PWM_INPUT_PIN
#define PWM_MIN_PULSE 800
#define PWM_MAX_PULSE 2200
ServoDecoder pulseInput([](int pin, uint16_t pulse) {
    float drive = 0;
    long min_pulse = sSettings.fPWMMinPulse;
    long neutral_pulse = sSettings.fPWMNeutralPulse;
    long max_pulse = sSettings.fPWMMaxPulse;
    uint8_t deadband = sSettings.fPWMDeadbandPercent;
    // printf("pulse: %d [%d:%d:%d]\n", pulse, int(min_pulse), int(max_pulse), int(neutral_pulse));
    if (pulse > PWM_MIN_PULSE && pulse < PWM_MAX_PULSE)
    {
        if (pulse < min_pulse)
            pulse = min_pulse;
        else if (pulse > max_pulse)
            pulse = max_pulse;
        if (pulse < neutral_pulse)
        {
            drive = -float(neutral_pulse - pulse) / (neutral_pulse - min_pulse);
        }
        else
        {
            drive = float(pulse - neutral_pulse) / (max_pulse - neutral_pulse);
        }
        if (float(deadband)/100 >= abs(drive))
        {
            drive = 0;
        }
        else if (!sDomeHasMovedManually)
        {
            sDomeHasMovedManually = true;
            restoreDomeSettings();
        }
        DEBUG_PRINT("PWM: "); DEBUG_PRINTLN(int(drive * 100));
        if (sSettings.fPWMInput)
            sDomeDrive.driveDome(drive);
    }
    else
    {
        DEBUG_PRINTLN("BAD PULSE");
    }
}, PWM_INPUT_PIN);
#endif

///////////////////////////////////////////////////////////////////////////////

static uint8_t getByteBit(uint8_t &byte, uint8_t bit)
{
    bit &= 7;
    return (byte >> bit) & 1;
}

static void setByteBit(uint8_t &byte, uint8_t bit, uint8_t val)
{
    bit &= 7;
    byte = (byte & ~(1<<bit)) | ((val&1)<<bit);
}

static uint8_t sPinState;
static void setDigitalPin(unsigned pin, bool state)
{
    if (pin-- <= SizeOfArray(sDigitalPin))
    {
        setByteBit(sPinState, pin, state);
        sPinManager.digitalWrite(sDigitalPin[pin], (state) ? HIGH : LOW);
    }
}

static void toggleDigitalPin(unsigned pin)
{
    if (pin <= SizeOfArray(sDigitalPin))
    {
        setDigitalPin(pin, !getByteBit(sPinState, pin-1));
    }
}

///////////////////////////////////////////////////////////////////////////////

static void restoreDomeSettings()
{
    sDomeDrive.setMaxSpeed(float(sSettings.fMaxSpeed) / MAX_SPEED);
    sDomeDrive.setScaling(sSettings.fSpeedScaling);
    sDomeDrive.setInverted(sSettings.fInverted);
    sDomeDrive.setThrottleAccelerationScale(sSettings.fAccScale);
    sDomeDrive.setThrottleDecelerationScale(sSettings.fDecScale);
    sDomePosition.setTimeout(sSettings.fTimeout);
    sDomePosition.setDomeHomePosition(sSettings.fHomePosition);
    sDomePosition.setDomeAutoMinDelay(sSettings.fDomeAutoMinDelay);
    sDomePosition.setDomeAutoMaxDelay(sSettings.fDomeAutoMaxDelay);
    sDomePosition.setDomeAutoLeftDegrees(sSettings.fDomeAutoLeft);
    sDomePosition.setDomeAutoRightDegrees(sSettings.fDomeAutoRight);
    sDomePosition.setDomeAutoSpeed(sSettings.fDomeSpeedAuto);

    sDomePosition.setDomeHomeSpeed(sSettings.fDomeSpeedHome);
    sDomePosition.setDomeHomeMinDelay(sSettings.fDomeHomeMinDelay);
    sDomePosition.setDomeHomeMaxDelay(sSettings.fDomeHomeMaxDelay);

    sDomePosition.setDomeTargetSpeed(sSettings.fDomeSpeedTarget);
    sDomePosition.setDomeTargetMinDelay(sSettings.fDomeTargetMinDelay);
    sDomePosition.setDomeTargetMaxDelay(sSettings.fDomeTargetMaxDelay);

    sDomePosition.setDomeMinSpeed(sSettings.fDomeSpeedMin);
    sDomePosition.setDomeFudgeFactor(sSettings.fDomeFudge);
    if (sSettings.fAutoSafety && !sDomeHasMovedManually)
    {
        if (sSettings.fRandomMode)
            DEBUG_PRINTLN(F("AUTO SAFETY PREVENTED AUTO SEEK MODE"));
        else if (sSettings.fHomeMode)
            DEBUG_PRINTLN(F("AUTO SAFETY PREVENTED HOME MODE"));
    }
    else if (sSettings.fRandomMode)
        sDomePosition.setDomeDefaultMode(DomePosition::kRandom);
    else if (sSettings.fHomeMode)
        sDomePosition.setDomeDefaultMode(DomePosition::kHome);
    else
        sDomePosition.setDomeDefaultMode(DomePosition::kOff);
#ifdef USE_SERVOS
    for (unsigned i = 1; i < SizeOfArray(sSettings.fServos); i++)
    {
        sServoDispatch.setServoEasingMethod(i, Easing::getEasingMethod(sSettings.fServos[i].fEasing));
        sServoDispatch.setServo(i, sServoDispatch.getPin(i), sSettings.fServos[i].fStartPulse,
            sSettings.fServos[i].fEndPulse, sSettings.fServos[i].fStartPulse, sSettings.fServos[i].fGroup);
    }
    sServoDispatch.setServo(0, sServoDispatch.getPin(0), sSettings.fPWMMinPulse,
        sSettings.fPWMMaxPulse, sSettings.fPWMNeutralPulse, 0);
#endif

    uint8_t pins = sSettings.fDigitalPins;
    for (uint8_t i = 0; i < 8; i++)
    {
        setDigitalPin(i+1, pins & 1);
        pins >>= 1;
    }
#if defined(ESP32) && defined(PWM_INPUT_PIN)
    if (sSettings.fPWMInput)
    {
        pulseInput.begin();
    }
    else
    {
        // pulseInput.end();
    }
#endif

#ifdef PWM_OUTPUT_PIN
    sDomeDrive.setOutput(sSettings.fPWMOutput, sSettings.fPacketSerialOutput);
#endif
}

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_WIFI_WEB
static bool sUpdateSettings;
#include "WebPages.h"
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_DROID_REMOTE
static bool sRemoteActive;
static SMQAddress sRemoteAddress;
#endif

///////////////////////////////////////////////////////////////////////////////

void configureDomeDrive()
{
#ifdef DOME_DRIVE_SOFT_SERIAL
    // We only use software serial to send Syren commands
    DOME_DRIVE_SERIAL.begin(sSettings.fSaberBaudRate, SWSERIAL_8N1, -1, TXD2_PIN, false);
#elif defined(DOME_DRIVE_SERIAL)
 #ifdef ESP32
    DOME_DRIVE_SERIAL.begin(sSettings.fSaberBaudRate, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
 #else
    DOME_DRIVE_SERIAL.begin(sSettings.fSaberBaudRate);
 #endif
#endif
}

void configureCommandSerial()
{
#ifdef COMMAND_SOFT_SERIAL
    COMMAND_SERIAL.begin(sSettings.fSerialBaudRate, SWSERIAL_8N1, RXD3_PIN, TXD3_PIN, false);
#elif defined(COMMAND_SERIAL)
 #ifdef ESP32
    COMMAND_SERIAL.begin(sSettings.fSerialBaudRate, SERIAL_8N1, RXD3_PIN, TXD3_PIN);
 #else
    COMMAND_SERIAL.begin(sSettings.fSerialBaudRate);
 #endif
#endif
}

////////////////////////////////

#ifdef USE_LVGL_DISPLAY

static void astro_lvgl_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p)
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, false );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

////////////////////////////////

void setupLVGLDisplay()
{
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    lv_init();
    lv_disp_buf = (lv_color_t *)heap_caps_malloc(SCREEN_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    tft.begin();          /* TFT init */
    tft.setRotation( 3 ); /* Landscape orientation, flipped */

    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, NULL, SCREEN_BUFFER_SIZE);

    /*Initialize the display*/
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = astro_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    // draw r2d2
    lv_obj_t *logo_img = lv_gif_create(lv_scr_act());
    lv_obj_center(logo_img);
    lv_gif_set_src(logo_img, &r2d2_gif);
    LV_DELAY(3440);
    lv_obj_del(logo_img);

    button1.attachClick([]() {
        printf("BUTTON 1\n");
        // pinMode(PIN_POWER_ON, OUTPUT);
        // pinMode(PIN_LCD_BL, OUTPUT);
        // digitalWrite(PIN_POWER_ON, LOW);
        // digitalWrite(PIN_LCD_BL, LOW);
        // esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON_2, 0); // 1 = High, 0 = Low
        // esp_deep_sleep_start();
    });
    button2.attachClick([]() {
        printf("BUTTON 2\n");
    });
}

#include "StatusDisplayLVGL.h"
StatusDisplayLVGL statusDisplay;
#endif

//////////////////////////////////////////////////////////////////////////

void setup()
{
    REELTWO_READY();

#ifdef USE_SPIFFS
    if (!mountReadOnlyFileSystem())
    {
        DEBUG_PRINTLN("Failed to mount filesystem");
    }
#endif

#ifdef USE_PREFERENCES
    if (!preferences.begin("roamadome", false))
    {
        DEBUG_PRINTLN("Failed to init prefs");
    }
#ifdef USE_WIFI
    wifiEnabled = wifiActive = preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED);
#endif
#ifdef USE_DROID_REMOTE
    remoteEnabled = remoteActive = preferences.getBool(PREFERENCE_REMOTE_ENABLED, REMOTE_ENABLED);
#endif
#endif

    PrintReelTwoInfo(Serial, "Droid Dome Controller");

#ifdef EEPROM_FLASH_PARTITION_NAME
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println("Failed to initialize EEPROM");
    }
    else
#endif
    if (sSettings.read())
    {
        Serial.println(F("Settings Restored"));
    }
    else
    {
        Serial.println(F("First Time Settings"));
        sSettings.write();
        if (sSettings.read())
        {
            Serial.println(F("Readback Success"));
        }
    }
#ifdef DOME_SENSOR_SERIAL
 #ifdef ESP32
    DOME_SENSOR_SERIAL.setRxBufferSize(1024);
    DOME_SENSOR_SERIAL.begin(sSettings.fSensorBaudRate, SERIAL_8N1, RXD1_PIN, TXD1_PIN);
 #else
    DOME_SENSOR_SERIAL.begin(sSettings.fSensorBaudRate);
 #endif
#endif
#ifdef ESP32
    if ((void*)&DOME_DRIVE_SERIAL_READ != (void*)&DOME_DRIVE_SERIAL_WRITE)
    {
        // We only use hardware serial to read Syren commands
        DOME_DRIVE_SERIAL_READ.begin(sSettings.fSaberBaudRate, SERIAL_8N1, RXD2_PIN, 0 /*not used*/);
    }
#endif
    configureCommandSerial();
    configureDomeDrive();

    sDomeDrive.setBaudRate(sSettings.fSaberBaudRate);
    sDomeDrive.setAddress(sSettings.fSaberAddressOutput);

    SetupEvent::ready();

    // N.B.: Must call pinMode() before begin()
    for (unsigned i = 0; i < SizeOfArray(sDigitalPin); i++)
    {
        sPinManager.pinMode(sDigitalPin[i], OUTPUT);
    }
    sPinManager.begin();

#ifdef USE_LCD_SCREEN
    Wire.begin();
#ifndef ESP32
    Wire.setWireTimeout();
#endif
    Wire.beginTransmission(SCREEN_ADDRESS);
    Wire.endTransmission();
#ifndef ESP32
    if (!Wire.getWireTimeoutFlag())
#endif
    {
        sDisplay.setEnabled(sDisplay.begin());
        if (sDisplay.isEnabled())
        {
            sDisplay.invertDisplay(false);
            sDisplay.clearDisplay();
            sDisplay.setRotation(2);
            sDisplay.setTextColor(WHITE);
        }
    }
#endif

#ifdef USE_LVGL_DISPLAY
    setupLVGLDisplay();
#endif

#ifdef USE_PREFERENCES
#ifdef USE_DROID_REMOTE
    if (remoteEnabled)
    {
    #ifdef USE_SMQ
        WiFi.mode(WIFI_MODE_APSTA);
        if (SMQ::init(preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME),
                        preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_SECRET)))
        {
            printf("Droid Remote Enabled %s:%s\n",
                preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME).c_str(),
                    preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_SECRET).c_str());
            SMQ::setHostDiscoveryCallback([](SMQHost* host) {
                if (host->hasTopic("LCD"))
                {
                    printf("Remote Discovered: %s\n", host->getHostName().c_str());
                }
            });

            SMQ::setHostLostCallback([](SMQHost* host) {
                printf("Lost: %s [%s] [%s]\n", host->getHostName().c_str(), host->getHostAddress().c_str(),
                    sRemoteAddress.toString().c_str());
                if (sRemoteAddress.equals(host->fAddr))
                {
                    printf("DISABLING REMOTE\n");
                    sDisplay.setEnabled(false);
                }
            });
        }
        else
        {
            printf("Failed to activate Droid Remote\n");
        }
    #endif
    }
#endif
#ifdef USE_WIFI
    if (wifiEnabled)
    {
    #ifdef USE_WIFI_WEB
        // In preparation for adding WiFi settings web page
        wifiAccess.setNetworkCredentials(
            preferences.getString(PREFERENCE_WIFI_SSID, getHostName()),
            preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE),
            preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT),
            preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED));
    #ifdef USE_WIFI_MARCDUINO
        wifiMarcduinoReceiver.setEnabled(preferences.getBool(PREFERENCE_MARCWIFI_ENABLED, MARC_WIFI_ENABLED));
        if (wifiMarcduinoReceiver.enabled())
        {
            wifiMarcduinoReceiver.setCommandHandler([](const char* cmd) {
                // Marcduino::processCommand(player, cmd);
                printf("PROCESS: \"%s\"\n", cmd);
                if (preferences.getBool(PREFERENCE_MARCWIFI_SERIAL_PASS, MARC_WIFI_SERIAL_PASS))
                {
                    COMMAND_SERIAL.print(cmd); COMMAND_SERIAL.print('\r');
                }
            });
        }
    #endif
        wifiAccess.notifyWifiConnected([](WifiAccess &wifi) {
        #ifdef STATUSLED_PIN
            statusLED.setMode(sCurrentMode = kWifiMode);
        #endif
            Serial.print("Connect to http://"); Serial.println(wifi.getIPAddress());
        #ifdef USE_MDNS
            // No point in setting up mDNS if R2 is the access point
            if (!wifi.isSoftAP())
            {
                String mac = wifi.getMacAddress();
                String hostName = mac.substring(mac.length()-5, mac.length());
                hostName.remove(2, 1);
                hostName = "RoamADome-"+hostName;
                if (webServer.enabled())
                {
                    Serial.print("Host name: "); Serial.println(hostName);
                    if (!MDNS.begin(hostName.c_str()))
                    {
                        DEBUG_PRINTLN("Error setting up MDNS responder!");
                    }
                }
            }
        #endif
        });
    #endif
    #ifdef USE_OTA
        ArduinoOTA.onStart([]()
        {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
            {
                type = "sketch";
            }
            else // U_SPIFFS
            {
                type = "filesystem";
            }
            DEBUG_PRINTLN("OTA START");
        })
        .onEnd([]()
        {
            DEBUG_PRINTLN("OTA END");
        })
        .onProgress([](unsigned int progress, unsigned int total)
        {
            // float range = (float)progress / (float)total;
        })
        .onError([](ota_error_t error)
        {
            String desc;
            if (error == OTA_AUTH_ERROR) desc = "Auth Failed";
            else if (error == OTA_BEGIN_ERROR) desc = "Begin Failed";
            else if (error == OTA_CONNECT_ERROR) desc = "Connect Failed";
            else if (error == OTA_RECEIVE_ERROR) desc = "Receive Failed";
            else if (error == OTA_END_ERROR) desc = "End Failed";
            else desc = "Error: "+String(error);
            DEBUG_PRINTLN(desc);
        });
    #endif
    }
#endif
#endif

    restoreDomeSettings();
    sDomeDrive.setDomePosition(&sDomePosition);
    sDomeDrive.setEnable(true);
    Serial.println(F("READY"));

#ifdef USE_WIFI_WEB
    // For safety we will stop the motors if the web client is connected
    webServer.setConnect([]() {
        // Callback for each connected web client
        // DEBUG_PRINTLN("Hello");
    });
#endif

#if defined(USE_WIFI) || defined(USE_DROID_REMOTE) || defined(USE_LVGL_DISPLAY)
    xTaskCreatePinnedToCore(
          eventLoopTask,
          "Events",
          5000,    // shrink stack size?
          NULL,
          1,
          &eventTask,
          0);
#endif
}

static byte sReadBuffer[4];
static uint8_t sReadPos;
static bool sSerialMotorActivity;
static uint32_t sPacketSerialErrors;
static uint32_t sLastSerialMotorEvent;

///////////////////////////////////////////////////////////////////////////////

static unsigned sBaudRates[] = {
    2400,
    9600,
    19200,
    38400,
    57600
#ifdef ESP32
    ,115200
#endif
};

#ifdef USE_MENUS

static unsigned sBooleanValues[] = {
    false,
    true,
};

static const char* sYesNoStrings[] = {
    "NO",
    "YES"
};

static const char* sOnOffStrings[] = {
    "OFF",
    "ON"
};

#include "menus/utility/ChoiceIntArrayScreen.h"
#include "menus/utility/ChoiceStrArrayScreen.h"
#include "menus/utility/UnsignedValueScreen.h"
#include "menus/utility/MenuScreen.h"

#include "menus/DomeFudgeScreen.h"
#include "menus/DomeHomeMaxDelayScreen.h"
#include "menus/DomeHomeMinDelayScreen.h"
#include "menus/DomeHomeSpeedScreen.h"
#include "menus/DomeMinSpeedScreen.h"
#include "menus/DomeAutoLeftScreen.h"
#include "menus/DomeAutoMaxDelayScreen.h"
#include "menus/DomeAutoMinDelayScreen.h"
#include "menus/DomeAutoRightScreen.h"
#include "menus/DomeAutoSpeedScreen.h"
#include "menus/DomeSpeedScreen.h"
#include "menus/EraseSettingsScreen.h"
#include "menus/HomeModeScreen.h"
#include "menus/MainScreen.h"
#include "menus/SerialBaudRateScreen.h"
#include "menus/PacketSerialInputScreen.h"
#include "menus/PacketSerialOutputScreen.h"
#include "menus/RandomModeScreen.h"
#include "menus/RotateDomeScreen.h"
#include "menus/SaberBaudRateScreen.h"
#include "menus/SelectScreen.h"
#include "menus/SetDeadbandPercentScreen.h"
#include "menus/SetHomeScreen.h"
#include "menus/SetMaxPulseScreen.h"
#include "menus/SetMinPulseScreen.h"
#include "menus/SetNeutralPulseScreen.h"
#include "menus/SetPWMInputScreen.h"
#include "menus/SetPWMOutputScreen.h"
#include "menus/SettingsScreen.h"
#include "menus/SettingsUpdatedScreen.h"
#include "menus/SplashScreen.h"
#ifdef USE_WIFI
#include "menus/WiFiModeScreen.h"
#endif

#endif

///////////////////////////////////////////////////////////////////////////////

#define CONSOLE_BUFFER_SIZE                 300
#define COMMAND_BUFFER_SIZE                 256

static bool sNextCommand;
static bool sProcessing;
static unsigned sPos;
static bool sWaitTarget;
static uint32_t sWaitNextSerialCommand;
static char sBuffer[CONSOLE_BUFFER_SIZE];
static bool sCmdNextCommand;
static char sCmdBuffer[COMMAND_BUFFER_SIZE];

static void runSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sProcessing = true;
}

static void resetSerialCommand()
{
    sWaitTarget = false;
    sWaitNextSerialCommand = 0;
    sNextCommand = false;
    sProcessing = (sCmdBuffer[0] == ':');
    sPos = 0;
}

static void abortSerialCommand()
{
    sBuffer[0] = '\0';
    sCmdBuffer[0] = '\0';
    sCmdNextCommand = false;
    resetSerialCommand();
}

bool processDomePositionCommand(const char* cmd)
{
    // if (!sDomeDrive.idle())
    // {
    //     Serial.println(F("MANUAL OVERRIDE. DOME NOT IDLE."));
    //     return false;
    // }
    // move mode ends on the next serial command
    switch (*cmd++)
    {
        case 'S':
        {
            if (sSettings.fAutoSafety && !sDomeHasMovedManually)
            {
                Serial.println(F("AUTO SAFETY PREVENTED PLAY SEQUENCE"));
                break;
            }
            sDomeDrive.autonomousDriveDome(0);

            // play sequence
            uint32_t seq = strtolu(cmd, &cmd);
            if (*cmd == '\0')
            {
                seq = min(max(int(seq), 0), MAX_COMMANDS);
                if (!sSettings.readCommand(seq, sCmdBuffer, sizeof(sCmdBuffer), ":DP"))
                {
                    sCmdBuffer[0] = '\0';
                }
                else
                {
                    Serial.print(F("Play Sequence: ")); Serial.println(seq);
                    Serial.println(sCmdBuffer);
                }
            }
            else
            {
                // Invalid
                return false;
            }
            break;
        }
    #ifdef USE_SERVOS
        case 'Q':
        {
            uint16_t num = strtolu(cmd, &cmd);
            if (num > 0 && num < SizeOfArray(sServoSettings))
            {
                unsigned i = 0;
                uint32_t val[4];
                while (*cmd == ',' && i < SizeOfArray(val))
                {
                    val[i++] = strtolu(cmd+1, &cmd);
                }
                switch (i)
                {
                    // :DPQ0,2000,100,30
                    // :DPQ0,2000,0,30
                    case 4:
                        // moveTime, startPos, endPos, easing
                        val[1] = min(val[1], 100u);
                        val[2] = min(val[1], 100u);
                        sServoDispatch.setServoEasingMethod(num, Easing::getEasingMethod(val[3]));
                        sServoDispatch.moveTo(num, 0, val[0], float(val[1]) / 100, float(val[2]) / 100);
                        break;

                    case 3:
                        // moveTime, endPos, easing
                        val[1] = min(val[1], 100u);
                        sServoDispatch.setServoEasingMethod(num, Easing::getEasingMethod(val[2]));
                        DEBUG_PRINTLN_HEX(uint32_t(Easing::getEasingMethod(val[2])));
                        sServoDispatch.moveTo(num, val[0], float(val[1]) / 100);
                        break;

                    case 2:
                        // moveTime, endPos
                        val[1] = min(val[1], 100u);
                        sServoDispatch.moveTo(num, val[0], float(val[1]) / 100);
                        break;

                    case 1:
                        // endPos
                        val[0] = min(val[0], 100u);
                        DEBUG_PRINT(F("moveTo num:")); DEBUG_PRINT(num); DEBUG_PRINT(F(" pos: ")); DEBUG_PRINTLN(val[0]);
                        sServoDispatch.moveTo(num, float(val[0]) / 100);
                        break;
                    case 0:
                        Serial.println(F("Invalid"));
                        break;
                }
            }
            break;
        }
    #endif
        case 'P':
        {
            if (cmd[0] >= '1' && cmd[0] <= '8' && (cmd[1] == '0' || cmd[1] == '1'))
            {
                setDigitalPin(cmd[0] - '0', cmd[1] - '0');
            }
            break;
        }
        case 'T':
        {
            if (cmd[0] >= '1' && cmd[0] <= '8')
            {
                toggleDigitalPin(cmd[0] - '0');
            }
            break;
        }
        case 'A':
        case 'D':
        {
            if (sSettings.fAutoSafety && !sDomeHasMovedManually)
            {
                Serial.println(F("AUTO SAFETY PREVENTED TARGET MODE"));
                break;
            }
            bool relative = (cmd[-1] == 'D');

            // position absolute degree
            sWaitTarget = true;
            float speed = sDomePosition.getDomeSpeedTarget();
            float minspeed = sDomePosition.getDomeMinSpeed();
            int32_t degrees;
            if (*cmd == 'R' && (cmd[1] == ',' || cmd[1] == '\0'))
            {
                degrees = random(360);
                cmd++;
            }
            else
            {
                degrees = strtol(cmd, &cmd);
            }
            if (*cmd == ',')
            {
                uint32_t speedpercentage;
                if (cmd[1] == 'R' && (cmd[2] == ',' || cmd[2] == '\0'))
                {
                    speedpercentage = random(MAX_SPEED);
                    cmd += 2;
                }
                else
                {
                    speedpercentage = strtolu(cmd+1, &cmd);
                }
                if (*cmd == '+' || *cmd == '\0')
                {
                    speedpercentage = min(max(int(speedpercentage), int(minspeed*100)), MAX_SPEED);
                    speed = speedpercentage / MAX_SPEED_F;
                }
            }
            if (*cmd == '+')
            {
                sWaitTarget = false;
            }
            sDomeDrive.autonomousDriveDome(0);
            if (*cmd == '\0')
            {
                Serial.print(F("ROTARY DEGREE: ")); Serial.print(degrees); Serial.print(F(" speed: ")); Serial.println(speed);
                if (relative)
                {
                    sDomePosition.setDomeRelativeTargetPosition(degrees);
                }
                else
                {
                    sDomePosition.setDomeHomeRelativeTargetPosition(degrees);
                }
                if (sDomePosition.getDomeSpeedTarget() != speed)
                {
                    sDomePosition.setDomeTargetSpeed(speed * 100);
                }
                sDomePosition.setTargetReached([]() {
                    Serial.print(F("REACHED TARGET: ")); Serial.println(sDomePosition.getHomeRelativeDomePosition());
                    sDomePosition.setDomeMode(sDomePosition.getDomeDefaultMode());
                    sDomePosition.setDomeTargetSpeed(sSettings.fDomeSpeedTarget);
                    sWaitTarget = false;
                });
                sDomePosition.setDomeMode(DomePosition::kTarget);
            }
            else
            {
                // Invalid
                return false;
            }
            break;
        }
        case 'R':
        {
            if (sSettings.fAutoSafety && !sDomeHasMovedManually)
            {
                Serial.println(F("AUTO SAFETY PREVENTED ROTATE MODE"));
                break;
            }
            // spin rotary speed
            int32_t speed = 0;
            if (*cmd == 'R')
            {
                speed = strtolu(cmd+1, &cmd);
                if (speed == 0)
                    speed = int(sSettings.fDomeSpeedTarget);
                speed = max(speed, int(sSettings.fDomeSpeedMin));
                speed = -speed + random(speed*2);
                if (abs(speed) < sSettings.fDomeSpeedMin)
                    speed = (speed < 0) ? -sSettings.fDomeSpeedMin : sSettings.fDomeSpeedMin;
            }
            else
            {
                speed = strtol(cmd, &cmd);
                if (abs(speed) < sSettings.fDomeSpeedMin)
                    speed = 0;
            }
            int32_t targetSpeed = sDomePosition.getDomeSpeedTarget() * 100;
            speed = min(max(speed, -targetSpeed), targetSpeed);
            sDomeDrive.autonomousDriveDome(0);
            if (*cmd == '\0')
            {
                Serial.print(F("ROTATE SPEED: ")); Serial.println(speed);
                sDomeDrive.autonomousDriveDome(float(speed) / MAX_SPEED_F);
            }
            else
            {
                // Invalid
                return false;
            }
            break;
        }
        case 'H':
        {
            if (sSettings.fAutoSafety && !sDomeHasMovedManually)
            {
                Serial.println(F("AUTO SAFETY PREVENTED SEEK HOME"));
                break;
            }
            sWaitTarget = true;
            int32_t speed = 0;
            if (*cmd == 'R')
            {
                speed = strtolu(cmd+1, &cmd);
                if (speed == 0)
                    speed = int(sSettings.fDomeSpeedTarget);
            }
            else if (isdigit(*cmd))
            {
                speed = strtol(cmd, &cmd);
            }
            else
            {
                speed = sDomePosition.getDomeSpeedHome() * 100;
            }
            speed = max(speed, int(sSettings.fDomeSpeedMin));
            int32_t targetSpeed = sDomePosition.getDomeSpeedTarget() * 100;
            speed = min(max(speed, -targetSpeed), targetSpeed);
            if (sDomePosition.getDomeSpeedTarget() != speed)
            {
                sDomePosition.setDomeTargetSpeed(speed);
            }
            sDomeDrive.autonomousDriveDome(0);
            sDomePosition.setDomeHomeRelativeTargetPosition(0);
                sDomePosition.setTargetReached([]() {
                    Serial.print(F("REACHED HOME: ")); Serial.println(sDomePosition.getHomeRelativeDomePosition());
                    sDomePosition.setDomeMode(sDomePosition.getDomeDefaultMode());
                    sDomePosition.setDomeTargetSpeed(sSettings.fDomeSpeedTarget);
                    sWaitTarget = false;
                });
            sDomePosition.setDomeMode(DomePosition::kTarget);
            break;
        }
        case 'Z':
        {
            sDomeDrive.autonomousDriveDome(0);
            restoreDomeSettings();
            break;
        }
        case 'W':
        {
            // wait seconds
            bool rand = false;
            bool waitms = false;
            int randlower = 1;
            int32_t duration;
            if ((waitms = (*cmd == 'M')))
                cmd++;
            else if ((rand = (*cmd == 'R')))
                cmd++;
            duration = strtolu(cmd, &cmd);
            if (rand)
            {
                if (duration == 0)
                    duration = 6;
                if (*cmd == ',')
                {
                    randlower = strtolu(cmd+1, &cmd);
                    randlower = min(max(randlower, 0), 600);
                }
                if (randlower > duration)
                {
                    uint32_t t = duration;
                    duration = randlower;
                    randlower = t;
                }
                duration = random(randlower, duration);
            }
            if (!waitms)
            {
                duration = min(max(duration, 1), 600);
                Serial.print(F("WAIT SECONDS: ")); Serial.println(duration);
                duration = duration * 1000L;
            }
            else
            {
                duration = min(max(long(duration), 1L), 600*1000L);
                Serial.print(F("WAIT MILLIS: ")); Serial.println(duration);
            }
            if (*cmd == '\0')
            {
                sWaitNextSerialCommand = millis() + duration;
            }
            else
            {
                // Invalid
                return false;
            }
            break;
        }
        default:
            // INVALID
            sDomeDrive.autonomousDriveDome(0);
            return false;
    }
    return true;
}

static void updateSettings()
{
    printf("restoreDomeSettings\n");
    restoreDomeSettings();
    printf("sSettings.write\n");
    sSettings.write();
    Serial.println(F("Updated"));
    printf("Updated\n");
#ifdef USE_WIFI_WEB
    sUpdateSettings = false;
#endif
}

// Given motor power percentage returns speed in cm/s
float calculateSpeed(unsigned speedPercentage)
{
    if (sSettings.fAutoSafety && !sDomeHasMovedManually)
    {
        Serial.println(F("AUTO SAFETY PREVENTED TARGET MODE"));
        return 0;
    }
    sWaitTarget = true;
    float speed = sDomePosition.getDomeSpeedTarget();
    int32_t degrees = 360;
    sDomeDrive.autonomousDriveDome(0);
    sDomePosition.setDomeRelativeTargetPosition(degrees);
    sDomePosition.setDomeTargetSpeed(speed * 100);
    sDomePosition.setTargetReached([]() {
        Serial.print(F("REACHED TARGET: ")); Serial.println(sDomePosition.getHomeRelativeDomePosition());
        sDomePosition.setDomeMode(sDomePosition.getDomeDefaultMode());
        sDomePosition.setDomeTargetSpeed(sSettings.fDomeSpeedTarget);
        sWaitTarget = false;
    });
    long startTime = millis();
    sDomePosition.setDomeMode(DomePosition::kTarget);

    sDomePosition.resetWatchdog();
    bool checkDomeDirection = true;
    unsigned sLastPositionReading = 0;
    while (sWaitTarget)
    {
        AnimatedEvent::process();
    #ifdef USE_MENUS
        sDisplay.process();
    #endif
    #ifdef ESP32
        vTaskDelay(1);
    #endif
        // Check if the motor is spinning in the opposite direction from what we expect
        if (checkDomeDirection)
        {
            unsigned pos = sDomePosition.getDomePosition();
            if (pos < 300)
            {
                if (!sLastPositionReading)
                {
                    sLastPositionReading = pos;
                }
                else if (sLastPositionReading < pos)
                {
                    // Previous reading is less than current reading - expected direction
                    sSettings.fInverted = false;
                    checkDomeDirection = false;
                }
                else if (sLastPositionReading > pos)
                {
                    // Previous reading greater than current reading - inverted direction
                    sSettings.fInverted = true;
                    checkDomeDirection = false;
                }
            }
        }
        if (sDomePosition.isTimeout())
        {
            return 0;
        }
    }
    long stopTime = millis();
    const float domeDiameterInches = 18.25;
    float linearVelocity = (2 * M_PI) / (float(stopTime - startTime) / 1000);
    float angularVelocityCentimetersPerSecond = (linearVelocity * (domeDiameterInches * 0.0256) / 2) * 100;
    return angularVelocityCentimetersPerSecond;
}

bool setupDomeControl()
{
    if (!sDomePosition.ready())
    {
        DEBUG_PRINTLN(F("Setup Failed. Dome Sensor Not Ready."));
        return false;
    }
    // Manual command so allow dome to move
    sDomeHasMovedManually = true;
    // Loop starts with 50% and ends after 100%
    unsigned speed = SETUP_VELOCITY_START;
    do
    {
        speed += SETUP_VELOCITY_INCREMENT;
        Serial.print("SPEED: "); Serial.println(speed);
        sDomeDrive.setMaxSpeed(speed / 100.0);
        sDomeDrive.setThrottleDecelerationScale(1);
        float duration = calculateSpeed(speed);
        if (!duration)
        {
            Serial.println("AUTO SETUP FAILED");
            return false;
        }
        Serial.print("Angular velocity: "); Serial.print(duration); Serial.println(" cm/s");
        if (duration >= sSettings.fSetupAngularVelocity)
        {
            Serial.print("GOOD MAX SPEED: "); Serial.println(speed);
            break;
        }
    }
    while (speed < 100);
    sSettings.fMaxSpeed = speed;
    updateSettings();
    return true;
}

void reboot()
{
    Serial.println(F("Restarting..."));
#ifdef ESP32
 #ifdef USE_DROID_REMOTE
    DisconnectRemote();
#endif
#ifdef PWM_INPUT_PIN
    pulseInput.end();
#endif
    unmountFileSystems();
#ifdef USE_PREFERENCES
    preferences.end();
#endif
    ESP.restart();
#elif defined(REELTWO_AVR)
    void (*resetArduino)() = NULL;
    resetArduino();
#else
    Serial.println(F("Restart not supported."));
#endif
}

#define UPDATE_SETTING(a,b) { \
    if (a != b) { a = b; needsUpdate = true; } else { unchanged = true; } }
void processConfigureCommand(const char* cmd)
{
    bool needsUpdate = false;
    bool unchanged = false;
    if (startswith_P(cmd, F("#DPZERO")))
    {
        DomeControllerSettings defaultSettings;
        *sSettings.data() = defaultSettings;
        updateSettings();
    }
    else if (startswith_P(cmd, F("#DPRESTART")))
    {
        reboot();
    }
#if defined(USE_VERBOSE_DOME_DEBUG) && defined(ESP32)
    else if (startswith_P(cmd, F("#DPDEBUG")) && isdigit(*cmd))
    {
        bool debugSetting = (strtolu(cmd, &cmd) == 1);
        if (sVerboseDomeDebug != debugSetting)
        {
            if (debugSetting)
            {
                Serial.println(F("Debug Enabled"));
            }
            else
            {
                Serial.println(F("Debug Disabled"));
            }
            sVerboseDomeDebug = debugSetting;
        }
    }
#endif
#ifdef USE_WIFI
    else if (startswith_P(cmd, F("#DPWIFI")) && isdigit(*cmd))
    {
        bool wifiSetting = (strtolu(cmd, &cmd) == 1);
        if (wifiEnabled != wifiSetting)
        {
            if (wifiSetting)
            {
                preferences.putBool(PREFERENCE_WIFI_ENABLED, true);
                Serial.println(F("WiFi Enabled"));
            }
            else
            {
                preferences.putBool(PREFERENCE_WIFI_ENABLED, false);
                Serial.println(F("WiFi Disabled"));
            }
            reboot();
        }
    }
#endif
#ifdef USE_DROID_REMOTE
    else if (startswith_P(cmd, F("#DPREMOTE")) && isdigit(*cmd))
    {
        bool remoteSetting = (strtolu(cmd, &cmd) == 1);
        if (remoteEnabled != remoteSetting)
        {
            if (remoteSetting)
            {
                preferences.putBool(PREFERENCE_REMOTE_ENABLED, true);
                Serial.println(F("Remote Enabled"));
            }
            else
            {
                preferences.putBool(PREFERENCE_REMOTE_ENABLED, false);
                Serial.println(F("Remote Disabled"));
            }
            reboot();
        }
    }
    else if (startswith_P(cmd, F("#DPRNAME")))
    {
        String newName = String(cmd);
        if (preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME) != newName)
        {
            preferences.putString(PREFERENCE_REMOTE_HOSTNAME, cmd);
            printf("Changed.\n");
            reboot();
        }
    }
    else if (startswith_P(cmd, F("#DPRSECRET")))
    {
        String newSecret = String(cmd);
        if (preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_HOSTNAME) != newSecret)
        {
            preferences.putString(PREFERENCE_REMOTE_SECRET, newSecret);
            printf("Changed.\n");
            reboot();
        }
    }
#endif
    else if (startswith_P(cmd, F("#DPSTATUS")))
    {
        unsigned errorCount = sDomeRing.getErrorCount();
        if (sSettings.fAutoSafety && !sDomeHasMovedManually)
        {
            Serial.println(F("Auto Safety Engaged"));
        }
        else
        {
            Serial.println(F("Auto Safety Disengaged"));
        }
    #ifdef PWM_INPUT_PIN
        if (pulseInput.isActive())
        {
            Serial.println(F("Pulse Input Active"));
        }
    #endif
        if (sSerialMotorActivity)
        {
            Serial.println(F("Receiving Packet Serial"));
        }
        if (sPacketSerialErrors != 0)
        {
            Serial.print(F("Packet Serial Errors: ")); Serial.println(sPacketSerialErrors);
        }
        if (errorCount != 0)
        {
            Serial.print(F("Dome Sensor Errors: ")); Serial.println(errorCount);
        }
        else
        {
            Serial.println(F("No Dome Sensor Errors"));
        }
    #ifdef USE_WIFI
        if (wifiEnabled)
        {
            Serial.println(F("WiFi Enabled"));
        }
        else
        {
            Serial.println(F("WiFi Disabled"));
        }
    #endif
    #ifdef USE_DROID_REMOTE
        if (remoteEnabled)
        {
            Serial.println(F("Remote Enabled"));
        }
        else
        {
            Serial.println(F("Remote Disabled"));
        }
     #endif
    }
    else if (startswith_P(cmd, F("#DPCONFIG")))
    {
        Serial.print(F("HomePos=")); Serial.println(sSettings.fHomePosition);
        Serial.print(F("MaxSpeed=")); Serial.println(sSettings.fMaxSpeed);
        Serial.print(F("MinSpeed=")); Serial.println(sSettings.fDomeSpeedMin);
        Serial.print(F("AutoMode=")); Serial.println(sSettings.fRandomMode);
        Serial.print(F("HomeMode=")); Serial.println(sSettings.fHomeMode);
        Serial.print(F("Scaling=")); Serial.println(sSettings.fSpeedScaling);
        Serial.print(F("Inverted=")); Serial.println(sSettings.fInverted);
        Serial.print(F("Timeout=")); Serial.println(sSettings.fTimeout);
        Serial.print(F("AutoSafety=")); Serial.println(sSettings.fAutoSafety);
        Serial.print(F("AccelerationScale=")); Serial.println(sSettings.fAccScale);
        Serial.print(F("DecelerationScale=")); Serial.println(sSettings.fDecScale);
        Serial.print(F("HomeMinDelay=")); Serial.println(sSettings.fDomeHomeMinDelay);
        Serial.print(F("HomeMaxDelay=")); Serial.println(sSettings.fDomeHomeMaxDelay);
        Serial.print(F("AutoMinDelay=")); Serial.println(sSettings.fDomeAutoMinDelay);
        Serial.print(F("AutoMaxDelay=")); Serial.println(sSettings.fDomeAutoMaxDelay);
        Serial.print(F("TargetMinDelay=")); Serial.println(sSettings.fDomeTargetMinDelay);
        Serial.print(F("TargetMaxDelay=")); Serial.println(sSettings.fDomeTargetMaxDelay);
        Serial.print(F("SetupAngularVelocity=")); Serial.print(sSettings.fSetupAngularVelocity); Serial.println(F(" cm/s"));
        Serial.print(F("AutoLeft=")); Serial.println(sSettings.fDomeAutoLeft);
        Serial.print(F("AutoRight=")); Serial.println(sSettings.fDomeAutoRight);
        Serial.print(F("Fudge=")); Serial.println(sSettings.fDomeFudge);
        Serial.print(F("SpeedHome=")); Serial.println(sSettings.fDomeSpeedHome);
        Serial.print(F("SpeedAuto=")); Serial.println(sSettings.fDomeSpeedAuto);
        Serial.print(F("SpeedTarget=")); Serial.println(sSettings.fDomeSpeedTarget);
        Serial.print(F("SaberAddressIn=")); Serial.println(sSettings.fSaberAddressInput);
        Serial.print(F("SaberAddressOut=")); Serial.println(sSettings.fSaberAddressOutput);
        Serial.print(F("SensorBaud=")); Serial.println(sSettings.fSensorBaudRate);
        Serial.print(F("SaberBaud=")); Serial.println(sSettings.fSaberBaudRate);
        Serial.print(F("SerialBaud=")); Serial.println(sSettings.fSerialBaudRate);
        Serial.print(F("SerialIn=")); Serial.println(sSettings.fPacketSerialInput);
        Serial.print(F("SerialOut=")); Serial.println(sSettings.fPacketSerialOutput);
        Serial.print(F("PWMIn=")); Serial.println(sSettings.fPWMInput);
        Serial.print(F("PWMOut=")); Serial.println(sSettings.fPWMOutput);
        Serial.print(F("PWMMinPulse=")); Serial.println(sSettings.fPWMMinPulse);
        Serial.print(F("PWMMaxPulse=")); Serial.println(sSettings.fPWMMaxPulse);
        Serial.print(F("PWMNeutralPulse=")); Serial.println(sSettings.fPWMNeutralPulse);
        Serial.print(F("PWMDeadband=")); Serial.println(sSettings.fPWMDeadbandPercent);
        Serial.print(F("DOut="));
        // Write out the pins backwards (pin1 first)
        uint8_t pins = sSettings.fDigitalPins;
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.print(pins & 1);
            pins >>= 1;
        }
        Serial.println();
    }
    else if (startswith_P(cmd, F("#DPSETUPVELOCITY")) && isdigit(*cmd))
    {
        uint32_t velocity = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fSetupAngularVelocity, velocity);
    }
    else if (startswith_P(cmd, F("#DPSETUP")))
    {
        setupDomeControl();
        restoreDomeSettings();
    }
    else if (startswith_P(cmd, F("#DPL")))
    {
        sSettings.listSortedCommands(Serial);
        Serial.println(F("Done"));
    }
    else if (startswith_P(cmd, F("#DPDSCALE")) && isdigit(*cmd))
    {
        int scale = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDecScale, min(scale, MAX_DEC_SCALE));
    }
    else if (startswith_P(cmd, F("#DPD")) && isdigit(*cmd))
    {
        int seq = strtolu(cmd, &cmd);
        Serial.println(seq);
        if (sSettings.deleteCommand(seq))
            Serial.println(F("Deleted"));
    }
    else if (startswith_P(cmd, F("#DPMAXSPEED")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fMaxSpeed, min(speed, MAX_SPEED));
    }
    else if (startswith_P(cmd, F("#DPTIMEOUT")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fTimeout, min(speed, MAX_TIMEOUT));
    }
    else if (startswith_P(cmd, F("#DPHOMESPEED")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeSpeedHome, min(speed, MAX_SPEED));
    }
    else if (startswith_P(cmd, F("#DPAUTOSPEED")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeSpeedAuto, min(speed, MAX_SPEED));
    }
    else if (startswith_P(cmd, F("#DPTARGETSPEED")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeSpeedTarget, min(speed, MAX_SPEED));
    }
    else if (startswith_P(cmd, F("#DPMINSPEED")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeSpeedMin, min(speed, MAX_SPEED));
    }
    else if (startswith_P(cmd, F("#DPAUTOLEFT")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeAutoLeft, min(speed, MAX_AUTO_LEFT));
    }
    else if (startswith_P(cmd, F("#DPAUTORIGHT")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeAutoRight, min(speed, MAX_AUTO_RIGHT));
    }
    else if (startswith_P(cmd, F("#DPAUTOMIN")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeAutoMinDelay, min(speed, MAX_AUTO_DELAY));
    }
    else if (startswith_P(cmd, F("#DPAUTOMAX")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeAutoMaxDelay, min(speed, MAX_AUTO_DELAY));
    }
    else if (startswith_P(cmd, F("#DPHOMEMIN")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeHomeMinDelay, min(speed, MAX_AUTO_DELAY));
    }
    else if (startswith_P(cmd, F("#DPHOMEMAX")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeHomeMaxDelay, min(speed, MAX_AUTO_DELAY));
    }
    else if (startswith_P(cmd, F("#DPTARGETMIN")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeTargetMinDelay, min(speed, MAX_AUTO_DELAY));
    }
    else if (startswith_P(cmd, F("#DPTARGETMAX")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeTargetMaxDelay, min(speed, MAX_AUTO_DELAY));
    }
    else if (startswith_P(cmd, F("#DPFUDGE")) && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fDomeFudge, min(speed, MAX_FUDGE_FACTOR));
    }
    else if (startswith_P(cmd, F("#DPHOMEPOS")))
    {
        if (isdigit(*cmd))
        {
            int pos = strtolu(cmd, &cmd);
            sDomePosition.setDomeHomeRelativeHomePosition(pos);
        }
        else
        {
            Serial.print(F("\nCURRENT POSITION: ")); Serial.println(sDomePosition.getDomePosition());
            sDomePosition.setDomeHomePosition(sDomePosition.getDomePosition());
        }
        UPDATE_SETTING(sSettings.fHomePosition, sDomePosition.getDomeHome());
    }
    else if (startswith_P(cmd, F("#DPHOME")))
    {
        if (isdigit(*cmd))
        {
            int mode = strtolu(cmd, &cmd);
            UPDATE_SETTING(sSettings.fHomeMode, (mode != 0));
        }
        else
        {
            // Toggle home mode
            UPDATE_SETTING(sSettings.fHomeMode, !sSettings.fHomeMode);
        }
    }
    else if (startswith_P(cmd, F("#DPSCALE")) && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fSpeedScaling, (mode != 0));
    }
    else if (startswith_P(cmd, F("#DPINVERT")) && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fInverted, (mode != 0));
    }
    else if (startswith_P(cmd, F("#DPAUTOSAFETY")) && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fAutoSafety, (mode != 0));
    }
    else if (startswith_P(cmd, F("#DPAUTO")))
    {
        if (isdigit(*cmd))
        {
            int mode = strtolu(cmd, &cmd);
            UPDATE_SETTING(sSettings.fRandomMode, (mode != 0));
        }
        else
        {
            // Toggle random mode
            UPDATE_SETTING(sSettings.fRandomMode, !sSettings.fRandomMode);
        }
    }
    else if (startswith_P(cmd, F("#DPASCALE")) && isdigit(*cmd))
    {
        int scale = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fAccScale, min(scale, MAX_ACC_SCALE));
    }
    else if (startswith_P(cmd, F("#DPSERIALIN")) && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fPacketSerialInput, (mode != 0));
    }
    else if (startswith_P(cmd, F("#DPPWMIN")) && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fPWMInput, (mode != 0));
    }
    else if (startswith_P(cmd, F("#DPPWMOUT")) && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fPWMOutput, (mode != 0));
    }
    else if (startswith_P(cmd, F("#DPPWMMIN")) && isdigit(*cmd))
    {
        uint32_t pulse = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fPWMMinPulse, pulse);
    }
    else if (startswith_P(cmd, F("#DPPWMMAX")) && isdigit(*cmd))
    {
        uint32_t pulse = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fPWMMaxPulse, pulse);
    }
    else if (startswith_P(cmd, F("#DPPWMNEUTRAL")) && isdigit(*cmd))
    {
        uint32_t pulse = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fPWMNeutralPulse, pulse);
    }
    else if (startswith_P(cmd, F("#DPSERIALOUT")) && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fPacketSerialOutput, (mode != 0));
    }
    else if (startswith_P(cmd, F("#DPSERIALBAUD")) && isdigit(*cmd))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        for (unsigned i = 0; i < SizeOfArray(sSerialBaudRates); i++)
        {
            if (baudrate == sSerialBaudRates[i])
            {
                UPDATE_SETTING(sSettings.fSerialBaudRate, baudrate);
                break;
            }
        }
    }
    else if (startswith_P(cmd, F("#DPSENSORBAUD")) && isdigit(*cmd))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        for (unsigned i = 0; i < SizeOfArray(sBaudRates); i++)
        {
            if (baudrate == sBaudRates[i])
            {
                UPDATE_SETTING(sSettings.fSensorBaudRate, baudrate);
                break;
            }
        }
    }
    else if (startswith_P(cmd, F("#DPSABERBAUD")) && isdigit(*cmd))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        for (unsigned i = 0; i < SizeOfArray(sBaudRates); i++)
        {
            if (baudrate == sBaudRates[i])
            {
                UPDATE_SETTING(sSettings.fSaberBaudRate, baudrate);
                break;
            }
        }
    }
    else if (startswith_P(cmd, F("#DPSABERADDRIN")) && isdigit(*cmd))
    {
        uint32_t addrIn = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fSaberAddressInput, addrIn);
    }
    else if (startswith_P(cmd, F("#DPSABERADDROUT")) && isdigit(*cmd))
    {
        uint32_t addrOut = strtolu(cmd, &cmd);
        UPDATE_SETTING(sSettings.fSaberAddressOutput, addrOut);
    }
    else if (startswith_P(cmd, F("#DPPIN")) && cmd[0] >= '1' && cmd[0] <= '8' && (cmd[1] == '0' || cmd[1] == '1'))
    {
        setByteBit(sSettings.fDigitalPins, cmd[0]-1, cmd[1]);
        updateSettings();
    }
    else if (startswith_P(cmd, F("#DPJOY")))
    {
        // Enable serial joystick
        Serial.println(F("Serial Console Joystick Emulation Connected."));
        Serial.println(F("Press END to stop."));
        sDomeStick.connect();
    }
    else if (startswith_P(cmd, F("#DPS")) && isdigit(*cmd))
    {
        uint32_t storeseq = strtolu(cmd, &cmd);
        if (*cmd == ':')
        {
            storeseq = min(max(int(storeseq), 0), 100);
            const char* startcmd = ++cmd;
            while (*cmd != '\0')
            {
                switch (*cmd)
                {
                    case 'Z':
                    {
                        Serial.println(F("RESET"));
                        break;
                    }
                    case 'R':
                    {
                        // speed
                        int32_t speed = strtol(cmd+1, &cmd);
                        speed = min(max(speed, -100), 100);
                        Serial.print(F("Rotate Speed: "));
                        Serial.println(speed);
                        break;
                    }
                    case 'A':
                    case 'D':
                    {
                        bool relative = (cmd[0] == 'D');
                        bool randdegrees = false;
                        bool randspeed = false;
                        bool randmax = false;
                        uint32_t speed = 0;
                        uint32_t maxspeed = 0;
                        int32_t degrees = 0;
                        cmd += 1;
                        if (*cmd == 'R')
                        {
                            randdegrees = true;
                            cmd++;
                        }
                        else
                        {
                            degrees = strtol(cmd, &cmd);
                        }
                        Serial.println(cmd);
                        // optional speed
                        if (*cmd == ',')
                        {
                            if (cmd[1] == 'R')
                            {
                                randspeed = true;
                                cmd += 2;
                            }
                            else
                            {
                                speed = strtolu(cmd+1, &cmd);
                                speed = max(min(max(int(speed), 0), 100), int(sSettings.fDomeSpeedAuto));
                            }
                        }
                        // optional maxspeed
                        if (*cmd == ',')
                        {
                            if (cmd[1] == 'R')
                            {
                                randmax = true;
                                cmd += 2;
                            }
                            else
                            {
                                maxspeed = strtolu(cmd+1, &cmd);
                                maxspeed = max(min(max(int(maxspeed), 0), 100), int(speed));
                            }
                        }
                        Serial.print(F("Rotate "));
                        Serial.print((relative) ? F("Relative") : F("Absolute"));
                        Serial.print(F(" Degrees: "));
                        if (randdegrees)
                            Serial.print(F("Random"));
                        else
                            Serial.print(degrees);
                        if (speed != 0 || randspeed)
                        {
                            Serial.print(F(" Speed: "));
                            if (randspeed)
                                Serial.print(F("Random"));
                            else
                                Serial.print(speed);
                        }
                        if (maxspeed != 0 || randmax)
                        {
                            Serial.print(F(" Max Speed: "));
                            if (randmax)
                                Serial.print(F("Random"));
                            else
                                Serial.print(maxspeed);
                        }
                        Serial.println();
                        break;
                    }
                    case 'W':
                    {
                        // seconds
                        bool rand = false;
                        bool millis = false;
                        int randlower = 1;
                        int seconds = 0;
                        cmd++;
                        if (*cmd == 'R')
                        {
                            rand = true;
                            cmd++;
                        }
                        else if (*cmd == 'M')
                        {
                            millis = true;
                            cmd++;
                        }
                        seconds = strtolu(cmd, &cmd);
                        seconds = min(max(seconds, 0), 600);
                        if (rand)
                        {
                            if (seconds == 0)
                                seconds = 6;
                            if (*cmd == ',')
                            {
                                randlower = strtolu(cmd+1, &cmd);
                                randlower = min(max(randlower, 0), 600);
                            }
                            if (randlower > seconds)
                            {
                                uint32_t t = seconds;
                                seconds = randlower;
                                randlower = t;
                            }
                        }
                        if (millis)
                        {
                            Serial.print(F("Wait Millis: "));
                        }
                        else
                        {
                            Serial.print(F("Wait Seconds: "));
                        }
                        if (rand)
                        {
                            Serial.print(F("Random "));
                            Serial.print(randlower);
                            Serial.print(F(" - "));
                            Serial.println(seconds);
                        }
                        else
                        {
                            Serial.println(seconds);
                        }
                        break;
                    }
                    case 'H':
                    {
                        // return home
                        int speed = sSettings.fDomeSpeedHome; cmd++;
                        if (*cmd == 'R')
                        {
                            speed = strtolu(cmd+1, &cmd);
                            speed = max(speed, int(sSettings.fDomeSpeedMin));
                            Serial.print(F("Return Home: Speed: Random "));
                            Serial.print(sSettings.fDomeSpeedMin);
                            Serial.print(F(" - "));
                            Serial.println(speed);
                        }
                        else if (isdigit(*cmd))
                        {
                            speed = strtolu(cmd+1, &cmd);
                            speed = max(speed, int(sSettings.fDomeSpeedMin));
                            Serial.print(F("Return Home: Speed: "));
                            Serial.println(speed);
                        }
                        else
                        {
                            speed = sDomePosition.getDomeSpeedHome() * 100;
                            Serial.print(F("Return Home: Speed: "));
                            Serial.println(speed);
                        }
                        break;
                    }
                    default:
                        cmd = nullptr;
                        break;
                }
                if (cmd != nullptr && *cmd == ':')
                {
                    cmd++;
                }
                else if (cmd == nullptr || *cmd != '\0')
                {
                    Serial.println(F("Invalid"));
                    Serial.println(cmd);
                    cmd = nullptr;
                    break;
                }
            }
            if (cmd != nullptr && *cmd == '\0')
            {
                Serial.print(F("Sequence ["));
                Serial.print(storeseq);
                Serial.print(F("]: "));
                Serial.println(startcmd);
                sCmdBuffer[0] = '\0';
                if (sSettings.writeCommand(storeseq, startcmd))
                    Serial.println(F("Stored"));
                else
                    Serial.println(F("Failed"));
            }
        }
        else
        {
            Serial.println(F("Invalid"));
        }
    }
    else
    {
        Serial.println(F("Invalid"));
    }
    if (needsUpdate)
    {
        updateSettings();
    }
    if (unchanged)
    {
        Serial.println(F("Unchanged"));
    }
}

bool processCommand(const char* cmd, bool firstCommand)
{
    sWaitNextSerialCommand = 0;
    if (*cmd == '\0')
        return true;
    if (!firstCommand)
    {
        if (cmd[0] != ':')
        {
            Serial.println(F("Invalid"));
            return false;
        }
        return processDomePositionCommand(cmd+1);
    }
    switch (cmd[0])
    {
        case ':':
            if (cmd[1] == 'D' && cmd[2] == 'P')
                return processDomePositionCommand(cmd+3);
            break;
        case '#':
            processConfigureCommand(cmd);
            return true;
        default:
            Serial.println(F("Invalid"));
            break;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////

#ifdef USE_SMQ
SMQMESSAGE(DIAL, {
    long newValue = msg.get_int32("new");
    long oldValue = msg.get_int32("old");
    sDisplay.remoteDialEvent(newValue, oldValue);
})

///////////////////////////////////////////////////////////////////////////////

SMQMESSAGE(BUTTON, {
    uint8_t id = msg.get_uint8("id");
    bool pressed = msg.get_uint8("pressed");
    bool repeat = msg.get_uint8("repeat");
    sDisplay.remoteButtonEvent(id, pressed, repeat);
})

///////////////////////////////////////////////////////////////////////////////

SMQMESSAGE(SELECT, {
// #ifdef STATUSLED_PIN
//     statusLED.setMode(sCurrentMode = kRemoteMovingMode);
// #endif
    printf("REMOTE ACTIVE\n");
    sRemoteActive = true;
    sRemoteAddress = SMQ::messageSender();
    sMainScreen.init();
    sDisplay.remoteActive();
    // sDisplay.acquireLock();
    // sMainScreen.init();
    // sDisplay.setEnabled(true);
    // sDisplay.switchToScreen(kMainScreen);
    // delay(1000);
    // sDisplay.releaseLock();
})
#endif

#ifdef USE_DROID_REMOTE
static void DisconnectRemote()
{
#ifdef USE_SMQ
    if (sRemoteActive)
    {
        sRemoteActive = false;
        if (SMQ::sendTopic("EXIT", "Remote"))
        {
            SMQ::sendString("addr", SMQ::getAddress());
            SMQ::sendEnd();
            sDisplay.setEnabled(false);
        #ifdef STATUSLED_PIN
            statusLED.setMode(sCurrentMode = kNormalMode);
        #endif
        }
    }
#endif
}
#endif

///////////////////////////////////////////////////////////////////////////////

void mainLoop()
{
#ifdef USE_WIFI_WEB
    if (sUpdateSettings)
    {
        updateSettings();
    }
#endif
    if (!sDomePosition.ready())
    {
        static uint32_t sLastDomeSensorCheck;
        if (sLastDomeSensorCheck + 5000 < millis())
        {
            DEBUG_PRINTLN(F("Dome Sensor Not Ready. Check connection and height."));
            sLastDomeSensorCheck = millis();
        }
    }
    AnimatedEvent::process();
#ifdef USE_MENUS
    sDisplay.process();
#endif
#ifdef USE_VERBOSE_DOME_DEBUG
  #ifdef ESP32
    if (sVerboseDomeDebug)
    {
        const char* modeStr = " OFF";
        DomePosition::Mode mode = sDomePosition.getDomeMode();
        int pos = sDomePosition.getDomePosition();
        static int sLastPos = -1;
        static DomePosition::Mode sLastMode = DomePosition::kOff;
        if (sLastMode != mode || pos != sLastPos)
        {
            switch (mode)
            {
                case DomePosition::kOff:
                    break;
                case DomePosition::kHome:
                    modeStr = "HOME";
                    break;
                case DomePosition::kRandom:
                    modeStr = "RAN";
                    break;
                case DomePosition::kTarget:
                    modeStr = "TRGT";
                    break;
            }
            printf("[%s] pos=%d\n", modeStr, pos);
            sLastPos = pos;
            sLastMode = mode;
        }
    }
  #endif
#endif
    // append commands to command buffer
    if (!sDomeStick.isEmulationActive() && Serial.available())
    {
        int ch = Serial.read();
        if (ch == 0x0A || ch == 0x0D)
        {
            runSerialCommand();
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
#ifdef COMMAND_SERIAL
    // Serial commands are processed in the same buffer as the console serial
    if (COMMAND_SERIAL_READ.available())
    {
        int ch = COMMAND_SERIAL_READ.read();
        if (ch >= 32 && ch <= 127)
        {
            Serial.print("READ: ");
            Serial.print(char(ch));
            Serial.print(" [");
            Serial.print(ch);
            Serial.println("]");
        }
        else
        {
            Serial.print("READ: . [");
            Serial.print(ch);
            Serial.println("]");
        }
        if (ch == 0x0A || ch == 0x0D)
        {
            runSerialCommand();
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
#endif
    if (sProcessing && !sWaitTarget && millis() > sWaitNextSerialCommand)
    {
        if (sCmdBuffer[0] == ':')
        {
            char* end = strchr(sCmdBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sCmdBuffer, !sCmdNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT(F("Unrecognized: ")); DEBUG_PRINTLN(sCmdBuffer);
                abortSerialCommand();
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sCmdBuffer, end);
                DEBUG_PRINT(F("REMAINS: "));
                DEBUG_PRINTLN(sCmdBuffer);
                sCmdNextCommand = true;
            }
            else
            {
                sCmdBuffer[0] = '\0';
                sCmdNextCommand = false;
            }
        }
        else if (sBuffer[0] == ':')
        {
            char* end = strchr(sBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sBuffer, !sNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT(F("Unrecognized: ")); DEBUG_PRINTLN(sBuffer);
                abortSerialCommand();
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sBuffer, end);
                sPos = strlen(sBuffer);
                DEBUG_PRINT(F("REMAINS: "));
                DEBUG_PRINTLN(sBuffer);
                sNextCommand = true;
            }
            else
            {
                resetSerialCommand();
                sBuffer[0] = '\0';
            }
        }
        else
        {
            processCommand(sBuffer, true);
            resetSerialCommand();
        }
    }
#ifdef DOME_DRIVE_SERIAL
    static float sLastMotorValue;
    if (sSettings.fPacketSerialInput)
    {
        do
        {
            if (sReadPos == sizeof(sReadBuffer))
            {
                byte address = sReadBuffer[0];
                byte command = sReadBuffer[1];
                byte value = sReadBuffer[2];
                byte crc = sReadBuffer[3];
                byte calcCRC = ((unsigned(address) + command + value) & B01111111);
            #ifdef ESP32
                // printf("SYREN{%d}:%d:%d:%d [%d]\n", address, command, value, crc, calcCRC);
            #endif
                if (address == sSettings.fSaberAddressInput && crc == calcCRC)
                {
                    // DEBUG_PRINT(address);
                    switch (command)
                    {
                        case 0: /* motor #1 */
                        case 1: /* motor #1 - negative */
                        {
                            float mval = float(value) / 127.0;
                            if (command == 1)
                                mval = -mval;
                            sLastMotorValue = mval;
                            if (value == 0)
                                break;
                           FALL_THROUGH
                        }
                        case 4: /* motor #2 */
                        case 5: /* motor #2 - negative */
                            if (value == 0)
                                break;
                           FALL_THROUGH

                        case 8: /* drive */
                        case 9: /* drive - negative */
                        case 10: /* turn */
                        case 11: /* turn - negative */
                            if (!sSerialMotorActivity)
                            {
                            #ifdef STATUSLED_PIN
                                statusLED.setMode(sCurrentMode + 3);
                            #endif
                                DEBUG_PRINTLN(F("Syren Active"));
                                sSerialMotorActivity = true;
                            }
                            // DEBUG_PRINT("["); DEBUG_PRINT(address); DEBUG_PRINT("] ");
                            // DEBUG_PRINT(command); DEBUG_PRINT(":"); DEBUG_PRINTLN(value);
                            sLastSerialMotorEvent = millis();
                            break;

                        case 2: /* setMinVoltage */
                        case 3: /* setMaxVoltage */
                        case 14: /* setTimeout */
                        case 15: /* setBaudRate */
                        case 16: /* setRamping */
                        case 17: /* setDeadband */
                        default:
                            // Ignore non-drive related Syren commands
                            // if (!sSerialMotorActivity)
                            // {
                            // #ifdef STATUSLED_PIN
                            //     statusLED.setMode(sCurrentMode + 3);
                            // #endif
                            //     DEBUG_PRINTLN(F("Syren Active"));
                            //     sSerialMotorActivity = true;
                            // }
                            // sLastSerialMotorEvent = millis();
                            break;
                    }
                    sReadPos = 0;
                }
                else
                {
                    if (command != 0 || value != 0 || crc != 0 || calcCRC != 0)
                    {
                        DEBUG_PRINT(F("{BAD}"));
                        DEBUG_PRINT('['); DEBUG_PRINT(address); DEBUG_PRINT(F("] "));
                        DEBUG_PRINT(command); DEBUG_PRINT(':');
                        DEBUG_PRINT(value); DEBUG_PRINT(F(":CRC:"));
                        DEBUG_PRINT_HEX(crc); DEBUG_PRINT(F(":EXPECTED:"));
                        DEBUG_PRINT_HEX(calcCRC);
                        DEBUG_PRINTLN();
                        sPacketSerialErrors++;
                    }
                    sReadBuffer[0] = sReadBuffer[1];
                    sReadBuffer[1] = sReadBuffer[2];
                    sReadBuffer[2] = sReadBuffer[3];
                    sReadPos = 3;
                }
            }
            while (DOME_DRIVE_SERIAL_READ.available() && sReadPos < sizeof(sReadBuffer))
            {
                sReadBuffer[sReadPos++] = DOME_DRIVE_SERIAL_READ.read();
            }
        }
        while (DOME_DRIVE_SERIAL_READ.available());
    }
    if (sSerialMotorActivity && sLastSerialMotorEvent + PACKET_SERIAL_TIMEOUT < millis())
    {
    #ifdef STATUSLED_PIN
        statusLED.setMode(sCurrentMode);
    #endif
        Serial.println(F("Syren Idle"));
        sSerialMotorActivity = false;
        sLastMotorValue = 0;
        sDomeDrive.driveDome(0);
        sReadPos = 0;
    }
    if (sSerialMotorActivity)
    {
        if (!sDomeHasMovedManually)
        {
            sDomeHasMovedManually = true; 
            restoreDomeSettings();
        }
        sDomeDrive.driveDome(sLastMotorValue);
    }
#endif
#ifdef PWM_INPUT_PIN
    if (sSettings.fPWMInput)
    {
        if (pulseInput.becameInactive())
        {
        #ifdef STATUSLED_PIN
            statusLED.setMode(sCurrentMode);
        #endif
            Serial.println(F("No PWM Input"));
            sDomeDrive.driveDome(0);
        }
        else if (pulseInput.becameActive())
        {
            Serial.println(F("PWM Input Active"));
        #ifdef STATUSLED_PIN
            statusLED.setMode(sCurrentMode + 3);
        #endif
        }
    }
#endif
}

#if defined(USE_WIFI) || defined(USE_DROID_REMOTE) || defined(USE_LVGL_DISPLAY)
void eventLoopTask(void* )
{
    for (;;)
    {
        if (wifiActive)
        {
        #ifdef USE_OTA
            ArduinoOTA.handle();
        #endif
        #ifdef USE_WIFI_WEB
            webServer.handle();
        #endif
        }
        if (remoteActive)
        {
        #ifdef USE_SMQ
            SMQ::process();
        #endif
        }
    #ifdef USE_LVGL_DISPLAY
        statusDisplay.refresh();
    #endif
        vTaskDelay(1);
    }
}
#endif

void loop()
{
    mainLoop();
 #ifdef ESP32
    vTaskDelay(1);
 #endif
}

#pragma GCC diagnostic pop
