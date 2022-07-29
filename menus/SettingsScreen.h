///////////////////////////////////////////////////////////////////////////////
//
// Settings menu selection
// 
///////////////////////////////////////////////////////////////////////////////

static const char* sSettingsMenu[] = {
    "Set Home\nPosition",
    "Set Max\nSpeed",
    "Set Home\nSpeed",
    "Set Auto\nSpeed",
    "Set Target\nSpeed",
    "Set Min\nSpeed",
    "Set Random\nLeft Dgr",
    "Set Random\nRight Dgr",
    "Set Fudge\nDegrees",
    "Set Random\nMin Delay",
    "Set Random\nMax Delay",
    "Set Home\nMin Delay",
    "Set Home\nMax Delay",
    "Set Auto\nSafety",
    "Set Serial\nBaud Rate",
    "Saber Baud\nRate",
    "Set Packet\nSerial In",
    "Set Packet\nSerial Out",
    "Set PWM\nInput",
    "Set PWM\nOutput",
    "Set PWM\nMin Pulse",
    "Set PWM\nMax Pulse",
    "Set PWM\nNeutral",
    "Set PWM\nDeadband",
    "Erase All\nSettings"
};

class SettingsScreen : public MenuScreen
{
public:
    enum {
        kSetHomePosition,
        kSetDomeSpeed,
        kSetHomeSpeed,
        kSetAutoSpeed,
        kSetTargetSpeed,
        kSetMinSpeed,
        kSetAutoLeft,
        kSetAutoRight,
        kSetFudge,
        kSetAutoMinDelay,
        kSetAutoMaxDelay,
        kSetHomeMinDelay,
        kSetHomeMaxDelay,
        kSetAutoSafety,
        kSetSerialBaudRate,
        kSetSaberBaudRate,
        kSetPacketSerialInput,
        kSetPacketSerialOutput,
        kSetPWMInput,
        kSetPWMOutput,
        kSetMinPulse,
        kSetMaxPulse,
        kSetNeutralPulse,
        kSetDeadbandPercent,
        kEraseAllSettings
    };
    SettingsScreen() :
        MenuScreen(kSettingsScreen, sSettingsMenu, SizeOfArray(sSettingsMenu))
    {}

    virtual void buttonInReleased() override
    {
        switch (fCurrentItem)
        {
            case kSetHomePosition:
                pushScreen(kSetHomeScreen);
                break;
            case kSetDomeSpeed:
                pushScreen(kDomeSpeedScreen);
                break;
            case kSetHomeSpeed:
                pushScreen(kDomeHomeSpeedScreen);
                break;
            case kSetAutoSpeed:
                pushScreen(kDomeAutoSpeedScreen);
                break;
            case kSetTargetSpeed:
                pushScreen(kDomeTargetSpeedScreen);
                break;
            case kSetMinSpeed:
                pushScreen(kDomeMinSpeedScreen);
                break;
            case kSetAutoLeft:
                pushScreen(kDomeAutoLeftScreen);
                break;
            case kSetAutoRight:
                pushScreen(kDomeAutoRightScreen);
                break;
            case kSetFudge:
                pushScreen(kDomeFudgeScreen);
                break;
            case kSetAutoMinDelay:
                pushScreen(kDomeAutoMinDelayScreen);
                break;
            case kSetAutoMaxDelay:
                pushScreen(kDomeAutoMaxDelayScreen);
                break;
            case kSetHomeMinDelay:
                pushScreen(kDomeHomeMinDelayScreen);
                break;
            case kSetHomeMaxDelay:
                pushScreen(kDomeHomeMaxDelayScreen);
                break;
            case kSetAutoSafety:
                pushScreen(kSetAutoSafetyScreen);
                break;
            case kSetSerialBaudRate:
                pushScreen(kSerialBaudRateScreen);
                break;
            case kSetSaberBaudRate:
                pushScreen(kSaberBaudRateScreen);
                break;
            case kSetPacketSerialInput:
                pushScreen(kPacketSerialInputScreen);
                break;
            case kSetPacketSerialOutput:
                pushScreen(kPacketSerialOutputScreen);
                break;
            case kSetPWMInput:
                pushScreen(kSetPWMInputScreen);
                break;
            case kSetPWMOutput:
                pushScreen(kSetPWMOutputScreen);
                break;
            case kSetMinPulse:
                pushScreen(kSetMinPulseScreen);
                break;
            case kSetMaxPulse:
                pushScreen(kSetMaxPulseScreen);
                break;
            case kSetNeutralPulse:
                pushScreen(kSetNeutralPulseScreen);
                break;
            case kSetDeadbandPercent:
                pushScreen(kSetDeadbandPercentScreen);
                break;
            case kEraseAllSettings:
                pushScreen(kEraseSettingsScreen);
                break;
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SettingsScreen sSettingsScreen;
