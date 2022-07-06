#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

#define USE_DEBUG               // Define to enable debug diagnostic
#define USE_SCREEN              // Define if using LCD and Rotary encoder
#undef  USE_SERVOS              // Define is enabling servo output on digital out pins
#undef  USE_DOME_DEBUG          // Define for dome motor specific debug
#undef  USE_DOME_SENSOR_DEBUG   // Define for dome sensor ring specific debug

#define DEFAULT_HOME_POSITION           0
#define DEFAULT_SABER_BAUD              9600
#define DEFAULT_MARC_BAUD               9600
#define DEFAULT_PACKET_SERIAL_INPUT     true
#define DEFAULT_PACKET_SERIAL_OUTPUT    true
#define DEFAULT_PWM_INPUT               false
#define DEFAULT_PWM_OUTPUT              false
#define DEFAULT_MAX_SPEED               50
#define DEFAULT_RANDOM_MODE             false
#define DEFAULT_HOME_MODE               false
#define DEFAULT_SPEED_SCALING           true
// If true no automatic movement will happen until the dome has been moved by joystick first.
// If false automatic movement can happen on startup.
#define DEFAULT_AUTO_SAFETY             true
#define DEFAULT_INVERTED                false
#define DEFAULT_PWM_MIN_PULSE           1000
#define DEFAULT_PWM_MAX_PULSE           2000
#define DEFAULT_PWM_NEUTRAL_PULSE       1500
#define DEFAULT_PWM_DEADBAND            5
#define DEFAULT_ACCELERATION_SCALE      20
#define DEFAULT_DECELERATION_SCALE      5
#define DEFAULT_DOME_HOME_MIN_DELAY     6
#define DEFAULT_DOME_HOME_MAX_DELAY     8
#define DEFAULT_DOME_SEEK_MIN_DELAY     6
#define DEFAULT_DOME_SEEK_MAX_DELAY     8
#define DEFAULT_DOME_TARGET_MIN_DELAY   0
#define DEFAULT_DOME_TARGET_MAX_DELAY   1
#define DEFAULT_DOME_SEEK_LEFT          80
#define DEFAULT_DOME_SEEK_RIGHT         80
#define DEFAULT_DOME_FUDGE              5
#define DEFAULT_DOME_SPEED_HOME         40
#define DEFAULT_DOME_SPEED_SEEK         30
#define DEFAULT_DOME_SPEED_TARGET       100
#define DEFAULT_DOME_SPEED_MIN          15
#define DEFAULT_DIGITAL_PINS            0
#define DEFAULT_TIMEOUT                 5

#define MAX_SPEED               100
#define MAX_SPEED_F             float(MAX_SPEED)
#define MAX_FUDGE_FACTOR        20
#define MAX_SEEK_LEFT           180
#define MAX_SEEK_RIGHT          180
#define MAX_SEEK_DELAY          255
#define MAX_ACC_SCALE           255
#define MAX_DEC_SCALE           255
#define MAX_TIMEOUT             30
#define MOVEMODE_MAX_INTERVAL   5       // default interval between random commands
#define MAX_COMMANDS            100

#define KEY_REPEAT_RATE_MS      500     // Key repeat rate in milliseconds

#define SYREN_ADDRESS           128
#define SYREN_ADDRESS_READ      129

#define DOME_SENSOR_SERIAL      Serial1
#define DOME_DRIVE_SERIAL       Serial2
#define MARC_SERIAL             Serial3
#define CONSOLE_BUFFER_SIZE     300
#define COMMAND_BUFFER_SIZE     256

#define DOME_SENSOR_SERIAL_BAUD 57600

#define PACKET_SERIAL_TIMEOUT   1500

///////////////////////////////////
// CONTROL BOARD PIN OUT
///////////////////////////////////
// Only change if you using a
// different PCB
///////////////////////////////////

#define PWM_INPUT_PIN           2
#define PWM_OUTPUT_PIN          3

#define PIN_ENCODER_A           4
#define PIN_ENCODER_B           5

#define BUTTON_IN               8
#define BUTTON_LEFT             9
#define BUTTON_UP               12
#define BUTTON_RIGHT            11
#define BUTTON_DOWN             10

#define DOUT1_PIN               22
#define DOUT2_PIN               23
#define DOUT3_PIN               24
#define DOUT4_PIN               25
#define DOUT5_PIN               26
#define DOUT6_PIN               27
#define DOUT7_PIN               28
#define DOUT8_PIN               29

#define SENSOR1_PIN             A0
#define SENSOR2_PIN             A1
#define SENSOR3_PIN             A2
#define SENSOR4_PIN             A3
#define SENSOR5_PIN             A4
#define SENSOR6_PIN             A5
#define SENSOR7_PIN             A6
#define SENSOR8_PIN             A7
#define SENSOR9_PIN             A8
#define SENSOROUT_PIN           A9

///////////////////////////////////

#include "ReelTwo.h"
#include "core/AnimatedEvent.h"
#include "core/AnalogMonitor.h"
#include "core/StringUtils.h"
#include "core/EEPROMSettings.h"
#include "core/PinManager.h"
#include "drive/DomeSensorRingSerialListener.h"
#include "drive/SerialConsoleController.h"
#include "drive/DomeDriveSabertooth.h"
#include "encoder/ServoDecoder.h"
#ifdef USE_SERVOS
#include "ServoDispatchDirect.h"
#include "ServoEasing.h"
#endif
#ifdef USE_SCREEN
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "encoder/AnoRotaryEncoder.h"
#include <Wire.h>
#endif

///////////////////////////////////

#ifdef USE_SCREEN

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define SCREEN_ADDRESS 0x3C

#include "Screens.h"
#include "CommandScreen.h"
#include "CommandScreenHandlerSSD1306.h"

PinManager sPinManager;
CommandScreenHandlerSSD1306 sDisplay(sPinManager);

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

#ifdef USE_SERVOS
// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.
//
//   Pin  Group ID,      Min,  Max
const ServoSettings sServoSettings[] PROGMEM = {
    { PWM_OUTPUT_PIN,    1000, 2000, 0 },
    { DOUT1_PIN,         1000, 2000, 0 },
    { DOUT2_PIN,         1000, 2000, 0 },
    { DOUT3_PIN,         1000, 2000, 0 },
    { DOUT4_PIN,         1000, 2000, 0 },
    { DOUT5_PIN,         1000, 2000, 0 },
    { DOUT6_PIN,         1000, 2000, 0 },
    { DOUT7_PIN,         1000, 2000, 0 },
    { DOUT8_PIN,         1000, 2000, 0 }
};

ServoDispatchDirect<SizeOfArray(sServoSettings)> sServoDispatch(sServoSettings);
#endif

////////////////////////////////

DomeSensorRingSerialListener sDomeRing(DOME_SENSOR_SERIAL);
DomePosition sDomePosition(sDomeRing);

class SerialDomeController : public SerialConsoleController, public AnimatedEvent
{
public:
    SerialDomeController(Stream& serial) :
        SerialConsoleController(serial)
    {}

    virtual void notify() override
    {
    }

    virtual void animate() override
    {
        switch (read())
        {
            case kHome:
                DEBUG_PRINTLN("HOME");
                sDomePosition.setDomeDefaultMode(DomePosition::kHome);
                break;
            case kEnd:
                DEBUG_PRINTLN("OFF");
                sDomePosition.setDomeDefaultMode(DomePosition::kOff);
                disconnect();
                break;
            case kPageUp:
                increaseSpeed();
                DEBUG_PRINT("SPEED: "); DEBUG_PRINTLN(floor(getSpeed()*MAX_SPEED));
                break;
            case kPageDown:
                decreaseSpeed();
                DEBUG_PRINT("SPEED: "); DEBUG_PRINTLN(floor(getSpeed()*MAX_SPEED));
                break;
            case 'R':
            case 'r':
                DEBUG_PRINTLN("RANDOM");
                sDomePosition.setDomeDefaultMode(DomePosition::kRandom);
                break;
        }
    }
};

SerialDomeController sDomeStick(Serial);
DomeDriveSabertooth sDomeDrive(SYREN_ADDRESS, DOME_DRIVE_SERIAL, sDomeStick);

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
    uint16_t fHomePosition = DEFAULT_HOME_POSITION;
    uint32_t fSaberBaudRate = DEFAULT_SABER_BAUD;
    uint32_t fMarcBaudRate = DEFAULT_MARC_BAUD;
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
    uint16_t fPWMMinPulse = DEFAULT_PWM_MIN_PULSE;
    uint16_t fPWMMaxPulse = DEFAULT_PWM_MAX_PULSE;
    uint16_t fPWMNeutralPulse = DEFAULT_PWM_NEUTRAL_PULSE;
    uint8_t fPWMDeadbandPercent = DEFAULT_PWM_DEADBAND;
    uint8_t fAccScale = DEFAULT_ACCELERATION_SCALE;
    uint8_t fDecScale = DEFAULT_DECELERATION_SCALE; 
    uint8_t fDomeSeekMinDelay = DEFAULT_DOME_SEEK_MIN_DELAY;
    uint8_t fDomeSeekMaxDelay = DEFAULT_DOME_SEEK_MAX_DELAY;
    uint8_t fDomeHomeMinDelay = DEFAULT_DOME_HOME_MIN_DELAY;
    uint8_t fDomeHomeMaxDelay = DEFAULT_DOME_HOME_MAX_DELAY;
    uint8_t fDomeTargetMinDelay = DEFAULT_DOME_TARGET_MIN_DELAY;
    uint8_t fDomeTargetMaxDelay = DEFAULT_DOME_TARGET_MAX_DELAY;
    uint8_t fDomeSeekLeft = DEFAULT_DOME_SEEK_LEFT;
    uint8_t fDomeSeekRight = DEFAULT_DOME_SEEK_RIGHT;
    uint8_t fDomeFudge = DEFAULT_DOME_FUDGE;
    uint8_t fDomeSpeedHome = DEFAULT_DOME_SPEED_HOME;
    uint8_t fDomeSpeedSeek = DEFAULT_DOME_SPEED_SEEK;
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
bool sDomeHasMovedManually = false;

///////////////////////////////////////////////////////////////////////////////

#ifdef PWM_INPUT_PIN
static void pulseInputChanged(uint16_t pulse)
{
    float drive = 0;
    long min_pulse = sSettings.fPWMMinPulse;
    long neutral_pulse = sSettings.fPWMNeutralPulse;
    long max_pulse = sSettings.fPWMMaxPulse;
    uint8_t deadband = sSettings.fPWMDeadbandPercent;
    if (pulse < neutral_pulse)
    {
        drive = -float(pulse - min_pulse) / (neutral_pulse - min_pulse);
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
        restoreDomeSettings();
        sDomeHasMovedManually = true; 
    }
    if (sSettings.fPWMInput)
        sDomeDrive.driveDome(drive);
}

ServoDecoder pulseInput(PWM_INPUT_PIN, pulseInputChanged);
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
    sDomeDrive.setThrottleAccelerationScale(sSettings.fDecScale);
    sDomePosition.setTimeout(sSettings.fTimeout);
    sDomePosition.setDomeHomePosition(sSettings.fHomePosition);
    sDomePosition.setDomeSeekMinDelay(sSettings.fDomeSeekMinDelay);
    sDomePosition.setDomeSeekMaxDelay(sSettings.fDomeSeekMaxDelay);
    sDomePosition.setDomeSeekLeftDegrees(sSettings.fDomeSeekLeft);
    sDomePosition.setDomeSeekRightDegrees(sSettings.fDomeSeekRight);
    sDomePosition.setDomeSeekSpeed(sSettings.fDomeSpeedSeek);

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
            DEBUG_PRINTLN("AUTO SAFETY PREVENTED RANDOM SEEK MODE");
        else if (sSettings.fHomeMode)
            DEBUG_PRINTLN("AUTO SAFETY PREVENTED HOME MODE");
    }
    else if (sSettings.fRandomMode)
        sDomePosition.setDomeDefaultMode(DomePosition::kRandom);
    else if (sSettings.fHomeMode)
        sDomePosition.setDomeDefaultMode(DomePosition::kHome);
    else
        sDomePosition.setDomeDefaultMode(DomePosition::kOff);
#ifdef USE_SERVOS
    for (unsigned i = 0; i < SizeOfArray(sSettings.fServos); i++)
    {
        unsigned pin = (i == 0) ? PWM_OUTPUT : DOUT1_PIN+i-1;
        if (i > 0)
            sServoDispatch.setServoEasingMethod(i, Easing::getEasingMethod(i));
        sServoDispatch.setServo(i, pin, sSettings.fServos[i].fStartPulse,
            sSettings.fServos[i].fEndPulse, sSettings.fServos[i].fStartPulse, sSettings.fServos[i].fGroup);
    }
#endif
    uint8_t pins = sSettings.fDigitalPins;
    for (uint8_t i = 0; i < 8; i++)
    {
        setDigitalPin(i+1, pins & 1);
        pins >>= 1;
    }
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
    REELTWO_READY();
    DOME_SENSOR_SERIAL.begin(DOME_SENSOR_SERIAL_BAUD);

    Serial.print(F("Droid Dome Controller - "));
    Serial.println(F(__DATE__));
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
    DOME_DRIVE_SERIAL.begin(sSettings.fSaberBaudRate);
#ifdef MARC_SERIAL
    MARC_SERIAL.begin(sSettings.fMarcBaudRate);
#endif

    SetupEvent::ready();

#ifdef USE_SCREEN
    Wire.begin();
    Wire.setWireTimeout();
    Wire.beginTransmission(SCREEN_ADDRESS);
    Wire.endTransmission();
    if (!Wire.getWireTimeoutFlag())
    {
        sDisplay.setEnabled(sDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS));
        if (sDisplay.isEnabled())
        {
            sDisplay.invertDisplay(false);
            sDisplay.clearDisplay();
            sDisplay.setRotation(2);
        }
    }
#endif
    restoreDomeSettings();
    sDomeDrive.setDomePosition(&sDomePosition);
    sDomeDrive.setEnable(true);
    Serial.println(F("READY"));
}

static byte sReadBuffer[4];
static uint8_t sReadPos;
static bool sSerialMotorActivity;
static uint32_t sLastSerialData;
static uint32_t sLastSerialMotorEvent;

///////////////////////////////////////////////////////////////////////////////

static unsigned sMarcBaudRates[] = {
    2400,
    9600,
    19200,
    38400
};

static unsigned sSaberBaudRates[] = {
    2400,
    9600,
    19200,
    38400
};

#ifdef USE_SCREEN

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
#include "menus/DomeSeekLeftScreen.h"
#include "menus/DomeSeekMaxDelayScreen.h"
#include "menus/DomeSeekMinDelayScreen.h"
#include "menus/DomeSeekRightScreen.h"
#include "menus/DomeSeekSpeedScreen.h"
#include "menus/DomeSpeedScreen.h"
#include "menus/DomeTargetMaxDelayScreen.h"
#include "menus/DomeTargetMinDelayScreen.h"
#include "menus/EraseSettingsScreen.h"
#include "menus/HomeModeScreen.h"
#include "menus/MainScreen.h"
#include "menus/MarcBaudRateScreen.h"
#include "menus/PacketSerialInputScreen.h"
#include "menus/PacketSerialOutputScreen.h"
#include "menus/RandomModeScreen.h"
#include "menus/RotateDomeScreen.h"
#include "menus/SaberBaudRateScreen.h"
#include "menus/SelectScreen.h"
#include "menus/SetAccScaleScreen.h"
#include "menus/SetAutoSafetyScreen.h"
#include "menus/SetDeadbandPercentScreen.h"
#include "menus/SetDecScaleScreen.h"
#include "menus/SetHomeScreen.h"
#include "menus/SetInvertedScreen.h"
#include "menus/SetMaxPulseScreen.h"
#include "menus/SetMinPulseScreen.h"
#include "menus/SetNeutralPulseScreen.h"
#include "menus/SetPWMInputScreen.h"
#include "menus/SetPWMOutputScreen.h"
#include "menus/SetSpeedScalingScreen.h"
#include "menus/SetTimeoutScreen.h"
#include "menus/SettingsScreen.h"
#include "menus/SettingsUpdatedScreen.h"
#include "menus/SplashScreen.h"

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
    sWaitNextSerialCommand = 0;
    sNextCommand = false;
    sProcessing = (sCmdBuffer[0] == ':');
    sPos = 0;
}

bool processDomePositionCommand(const char* cmd)
{
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
            if (num >= 0 && num <= 8)
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
                        val[1] = min(val[1], 100);
                        val[2] = min(val[1], 100);
                        sServoDispatch.setServoEasingMethod(num, Easing::getEasingMethod(val[3]));
                        sServoDispatch.moveTo(num, 0, val[0], float(val[1]) / 100, float(val[2]) / 100);
                        break;

                    case 3:
                        // moveTime, endPos, easing
                        val[1] = min(val[1], 100);
                        sServoDispatch.setServoEasingMethod(num, Easing::getEasingMethod(val[2]));
                        DEBUG_PRINTLN_HEX(uint32_t(Easing::getEasingMethod(val[2])));
                        sServoDispatch.moveTo(num, val[0], float(val[1]) / 100);
                        break;

                    case 2:
                        // moveTime, endPos
                        val[1] = min(val[1], 100);
                        sServoDispatch.moveTo(num, val[0], float(val[1]) / 100);
                        break;

                    case 1:
                        // endPos
                        val[0] = min(val[0], 100);
                        DEBUG_PRINTLN("moveTo num:"+String(num)+" pos: "+val[0]);
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
                Serial.print(F("ROTARY DEGREE: ")); Serial.print(degrees); Serial.print(" speed: "); Serial.println(speed);
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
                    Serial.print("REACHED TARGET: "); Serial.println(sDomePosition.getHomeRelativeDomePosition());
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
                    Serial.print("REACHED HOME: "); Serial.println(sDomePosition.getHomeRelativeDomePosition());
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
            int randlower = 1;
            int seconds;
            if ((rand = (*cmd == 'R')))
                cmd++;
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
                seconds = random(randlower, seconds);
            }
            Serial.print(F("WAIT SECONDS: ")); Serial.println(seconds);
            if (*cmd == '\0')
            {
                sWaitNextSerialCommand = millis() + uint32_t(min(max(int(seconds), 1), 600)) * 1000L;
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
    restoreDomeSettings();
    sSettings.write();
    Serial.println(F("Updated"));
}

void processConfigureCommand(const char* cmd)
{
    if (startswith(cmd, "#DPZERO"))
    {
        DomeControllerSettings defaultSettings;
        *sSettings.data() = defaultSettings;
        updateSettings();
    }
    else if (startswith(cmd, "#DPCONFIG"))
    {
        Serial.print("HomePos="); Serial.println(sSettings.fHomePosition);
        Serial.print("MaxSpeed="); Serial.println(sSettings.fMaxSpeed);
        Serial.print("MinSpeed="); Serial.println(sSettings.fDomeSpeedMin);
        Serial.print("SeekMode="); Serial.println(sSettings.fRandomMode);
        Serial.print("HomeMode="); Serial.println(sSettings.fHomeMode);
        Serial.print("Scaling="); Serial.println(sSettings.fSpeedScaling);
        Serial.print("Inverted="); Serial.println(sSettings.fInverted);
        Serial.print("Timeout="); Serial.println(sSettings.fTimeout);
        Serial.print("AutoSafety="); Serial.println(sSettings.fAutoSafety);
        Serial.print("AccelerationScale="); Serial.println(sSettings.fAccScale);
        Serial.print("DecelerationScale="); Serial.println(sSettings.fDecScale);
        Serial.print("HomeMinDelay="); Serial.println(sSettings.fDomeHomeMinDelay);
        Serial.print("HomeMaxDelay="); Serial.println(sSettings.fDomeHomeMaxDelay);
        Serial.print("SeekMinDelay="); Serial.println(sSettings.fDomeSeekMinDelay);
        Serial.print("SeekMaxDelay="); Serial.println(sSettings.fDomeSeekMaxDelay);
        Serial.print("TargetMinDelay="); Serial.println(sSettings.fDomeTargetMinDelay);
        Serial.print("TargetMaxDelay="); Serial.println(sSettings.fDomeTargetMaxDelay);
        Serial.print("SeekLeft="); Serial.println(sSettings.fDomeSeekLeft);
        Serial.print("SeekRight="); Serial.println(sSettings.fDomeSeekRight);
        Serial.print("Fudge="); Serial.println(sSettings.fDomeFudge);
        Serial.print("SpeedHome="); Serial.println(sSettings.fDomeSpeedHome);
        Serial.print("SpeedSeek="); Serial.println(sSettings.fDomeSpeedSeek);
        Serial.print("SpeedTarget="); Serial.println(sSettings.fDomeSpeedTarget);
        Serial.print("SaberBaud="); Serial.println(sSettings.fSaberBaudRate);
        Serial.print("MarcBaud="); Serial.println(sSettings.fMarcBaudRate);
        Serial.print("SerialIn="); Serial.println(sSettings.fPacketSerialInput);
        Serial.print("SerialOut="); Serial.println(sSettings.fPacketSerialOutput);
        Serial.print("PWMIn="); Serial.println(sSettings.fPWMInput);
        Serial.print("PWMOut="); Serial.println(sSettings.fPWMOutput);
        Serial.print("PWMMinPulse="); Serial.println(sSettings.fPWMMinPulse);
        Serial.print("PWMMaxPulse="); Serial.println(sSettings.fPWMMaxPulse);
        Serial.print("PWMNeutralPulse="); Serial.println(sSettings.fPWMNeutralPulse);
        Serial.print("PWMDeadband="); Serial.println(sSettings.fPWMDeadbandPercent);

        Serial.print("DOut=");
        // Write out the pins backwards (pin1 first)
        uint8_t pins = sSettings.fDigitalPins;
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.print(pins & 1);
            pins >>= 1;
        }
        Serial.println();
    }
    else if (startswith(cmd, "#DPSETUP"))
    {
    }
    else if (startswith(cmd, "#DPL"))
    {
        sSettings.listSortedCommands(Serial);
        Serial.println(F("Done"));
    }
    else if (startswith(cmd, "#DPD") && isdigit(*cmd))
    {
        int seq = strtolu(cmd, &cmd);
        Serial.println(seq);
        if (sSettings.deleteCommand(seq))
            Serial.println(F("Deleted"));
    }
    else if (startswith(cmd, "#DPSPEED") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fMaxSpeed = min(speed, MAX_SPEED);
        updateSettings();
    }
    else if (startswith(cmd, "#DPTIMEOUT") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fTimeout = min(speed, MAX_TIMEOUT);
        updateSettings();
    }
    else if (startswith(cmd, "#DPHOMESPEED") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeSpeedHome = min(speed, MAX_SPEED);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSEEKSPEED") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeSpeedSeek = min(speed, MAX_SPEED);
        updateSettings();
    }
    else if (startswith(cmd, "#DPMINSPEED") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeSpeedMin = min(speed, MAX_SPEED);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSEEKLEFT") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeSeekLeft = min(speed, MAX_SEEK_LEFT);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSEEKRIGHT") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeSeekRight = min(speed, MAX_SEEK_RIGHT);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSEEKMIN") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeSeekMinDelay = min(speed, MAX_SEEK_DELAY);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSEEKMAX") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeSeekMaxDelay = min(speed, MAX_SEEK_DELAY);
        updateSettings();
    }
    else if (startswith(cmd, "#DPHOMEMIN") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeHomeMinDelay = min(speed, MAX_SEEK_DELAY);
        updateSettings();
    }
    else if (startswith(cmd, "#DPHOMEMAX") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeHomeMaxDelay = min(speed, MAX_SEEK_DELAY);
        updateSettings();
    }
    else if (startswith(cmd, "#DPTARGETMIN") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeTargetMinDelay = min(speed, MAX_SEEK_DELAY);
        updateSettings();
    }
    else if (startswith(cmd, "#DPTARGETMAX") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeTargetMaxDelay = min(speed, MAX_SEEK_DELAY);
        updateSettings();
    }
    else if (startswith(cmd, "#DPFUDGE") && isdigit(*cmd))
    {
        int speed = strtolu(cmd, &cmd);
        sSettings.fDomeFudge = min(speed, MAX_FUDGE_FACTOR);
        updateSettings();
    }
    else if (startswith(cmd, "#DPHOMEPOS"))
    {
        if (isdigit(*cmd))
        {
            int pos = strtolu(cmd, &cmd);
            sDomePosition.setDomeHomeRelativeHomePosition(pos);
        }
        else
        {
            Serial.print("\nCURRENT POSITION: "); Serial.println(sDomePosition.getDomePosition());
            sDomePosition.setDomeHomePosition(sDomePosition.getDomePosition());
        }
        sSettings.fHomePosition = sDomePosition.getDomeHome();
        updateSettings();
    }
    else if (startswith(cmd, "#DPHOME") && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        sSettings.fHomeMode = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSEEK") && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        sSettings.fRandomMode = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSCALE") && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        sSettings.fSpeedScaling = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPINVERT") && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        sSettings.fInverted = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPAUTOSAFETY") && isdigit(*cmd))
    {
        int mode = strtolu(cmd, &cmd);
        sSettings.fAutoSafety = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPASCALE") && isdigit(*cmd))
    {
        int scale = strtolu(cmd, &cmd);
        sSettings.fAccScale = min(scale, MAX_ACC_SCALE);
        updateSettings();
    }
    else if (startswith(cmd, "#DPDSCALE") && isdigit(*cmd))
    {
        int scale = strtolu(cmd, &cmd);
        sSettings.fDecScale = min(scale, MAX_DEC_SCALE);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSERIALIN") && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        sSettings.fPacketSerialInput = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPPWMIN") && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        sSettings.fPWMInput = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPPWMOUT") && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        sSettings.fPWMOutput = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPSERIALOUT") && isdigit(*cmd))
    {
        uint32_t mode = strtolu(cmd, &cmd);
        sSettings.fPacketSerialOutput = (mode != 0);
        updateSettings();
    }
    else if (startswith(cmd, "#DPMARCBAUD") && isdigit(*cmd))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        for (unsigned i = 0; i < SizeOfArray(sMarcBaudRates); i++)
        {
            if (baudrate == sMarcBaudRates[i])
            {
                sSettings.fMarcBaudRate = baudrate;
                updateSettings();
                return;
            }
        }
    }
    else if (startswith(cmd, "#DPSABERBAUD") && isdigit(*cmd))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        for (unsigned i = 0; i < SizeOfArray(sSaberBaudRates); i++)
        {
            if (baudrate == sSaberBaudRates[i])
            {
                sSettings.fSaberBaudRate = baudrate;
                updateSettings();
                return;
            }
        }
    }
    else if (startswith(cmd, "#DPPIN") && cmd[0] >= '1' && cmd[0] <= '8' && (cmd[1] == '0' || cmd[1] == '1'))
    {
        setByteBit(sSettings.fDigitalPins, cmd[0]-1, cmd[1]);
        updateSettings();
    }
    else if (startswith(cmd, "#DPJOY"))
    {
        // Enable serial joystick
        Serial.println("Serial Console Joystick Emulation Connected.");
        Serial.println("Press END to stop.");
        sDomeStick.connect();
    }
    else if (startswith(cmd, "#DPS") && isdigit(*cmd))
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
                        Serial.print("Rotate Speed: ");
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
                                speed = max(min(max(int(speed), 0), 100), int(sSettings.fDomeSpeedSeek));
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
                        Serial.print("Rotate ");
                        Serial.print((relative) ? "Relative" : "Absolute");
                        Serial.print(" Degrees: ");
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
                            Serial.print(" Max Speed: ");
                            if (randmax)
                                Serial.print("Random");
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
                        int randlower = 1;
                        int seconds = 0;
                        cmd++;
                        if (*cmd == 'R')
                        {
                            rand = true;
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
                        Serial.print("Wait Seconds: ");
                        if (rand)
                        {
                            Serial.print("Random ");
                            Serial.print(randlower);
                            Serial.print(" - ");
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
                            Serial.print("Return Home: Speed: Random ");
                            Serial.print(sSettings.fDomeSpeedMin);
                            Serial.print(" - ");
                            Serial.println(speed);
                        }
                        else if (isdigit(*cmd))
                        {
                            speed = strtolu(cmd+1, &cmd);
                            speed = max(speed, int(sSettings.fDomeSpeedMin));
                            Serial.print("Return Home: Speed: ");
                            Serial.println(speed);
                        }
                        else
                        {
                            speed = sDomePosition.getDomeSpeedHome() * 100;
                            Serial.print("Return Home: Speed: ");
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

void loop()
{
    AnimatedEvent::process();
#ifdef USE_SCREEN
    sDisplay.process();
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
#ifdef MARC_SERIAL
    // Marcduino commands are processed in the same buffer as the console serial
    if (MARC_SERIAL.available())
    {
        int ch = MARC_SERIAL.read();
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
    if (sProcessing && millis() > sWaitNextSerialCommand)
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
                sWaitNextSerialCommand = 0;
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
                sWaitNextSerialCommand = 0;
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

    static float sLastMotorValue;
    if (sReadPos == sizeof(sReadBuffer))
    {
        byte address = sReadBuffer[0];
        byte command = sReadBuffer[1];
        byte value = sReadBuffer[2];
        byte crc = sReadBuffer[3];
        byte calcCRC = ((address + command + value) & B01111111);
        if (address == SYREN_ADDRESS_READ && crc == calcCRC)
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
                    // FALL THROUGH
                    [[fallthrough]];
                }
                case 4: /* motor #2 */
                case 5: /* motor #2 - negative */
                    if (value == 0)
                        break;
                    // FALL THROUGH
                    [[fallthrough]];
                case 2: /* setMinVoltage */
                case 3: /* setMaxVoltage */
                case 8: /* drive */
                case 9: /* drive - negative */
                case 10: /* turn */
                case 11: /* turn - negative */
                case 14: /* setTimeout */
                case 15: /* setBaudRate */
                case 16: /* setRamping */
                case 17: /* setDeadband */
                default:
                    if (!sSerialMotorActivity)
                    {
                        DEBUG_PRINTLN("Syren Active");
                        sSerialMotorActivity = true;
                    }
                    // DEBUG_PRINT("["); DEBUG_PRINT(address); DEBUG_PRINT("] ");
                    // DEBUG_PRINT(command); DEBUG_PRINT(":"); DEBUG_PRINTLN(value);
                    sLastSerialMotorEvent = millis();
                    break;
            }
            sReadPos = 0;
        }
        else
        {
            DEBUG_PRINT("{BAD}");
            DEBUG_PRINT("["); DEBUG_PRINT(address); DEBUG_PRINT("] ");
            DEBUG_PRINT(command); DEBUG_PRINT(":");
            DEBUG_PRINT(value); DEBUG_PRINT(":CRC:");
            DEBUG_PRINT_HEX(calcCRC); DEBUG_PRINT(":EXPECTED:");
            DEBUG_PRINT_HEX(crc);
            DEBUG_PRINTLN(value);
            sReadBuffer[0] = sReadBuffer[1];
            sReadBuffer[1] = sReadBuffer[2];
            sReadBuffer[2] = sReadBuffer[3];
            sReadPos = 3;
        }
    }
    if (sSettings.fPacketSerialInput)
    {
        while (DOME_DRIVE_SERIAL.available() && sReadPos < sizeof(sReadBuffer))
        {
            sReadBuffer[sReadPos++] = DOME_DRIVE_SERIAL.read();
            sLastSerialData = millis();
        }
    }
    if (sSerialMotorActivity && sLastSerialMotorEvent + PACKET_SERIAL_TIMEOUT < millis())
    {
        DEBUG_PRINTLN("Syren Idle");
        sSerialMotorActivity = false;
        sLastMotorValue = 0;
    }
    if (sSerialMotorActivity)
    {
        if (!sDomeHasMovedManually)
        {
            restoreDomeSettings();
            sDomeHasMovedManually = true; 
        }
        sDomeDrive.driveDome(sLastMotorValue);
    }

#ifdef PWM_INPUT_PIN
    if (sSettings.fPWMInput && pulseInput.becameInactive())
    {
        DEBUG_PRINTLN("No PWM Input");
        sDomeDrive.driveDome(0);
    }
#endif
}
#pragma GCC diagnostic pop
