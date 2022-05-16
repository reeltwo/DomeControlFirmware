///////////////////////////////////////////////////////////////////////////////
//
// Settings menu selection
// 
///////////////////////////////////////////////////////////////////////////////

static const char* sSettingsMenu[] = {
    "Set Home\nPosition",
    "Set Dome\nSpeed",
    "Set Home\nSpeed",
    "Set Seek\nSpeed",
    "Set Min\nSpeed",
    "Set Random\nLeft Dgr",
    "Set Random\nRight Dgr",
    "Set Fudge\nDegrees",
    "Set Random\nMin Delay",
    "Set Random\nMax Delay",
    "Set Invert\nMotor",
    "Set Scale\nSpeed",
    "Set Acc\nScale",
    "Set Dec\nScale",
    "Marc Baud\nRate",
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
        kSetSeekSpeed,
        kSetMinSpeed,
        kSetSeekLeft,
        kSetSeekRight,
        kSetFudge,
        kSetSeekMinDelay,
        kSetSeekMaxDelay,
        kSetInvertMotor,
        kSetScaleSpeed,
        kSetAccScale,
        kSetDecScale,
        kSetMarcBaudRate,
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
            case kSetSeekSpeed:
                pushScreen(kDomeSeekSpeedScreen);
                break;
            case kSetMinSpeed:
                pushScreen(kDomeMinSpeedScreen);
                break;
            case kSetSeekLeft:
                pushScreen(kDomeSeekLeftScreen);
                break;
            case kSetSeekRight:
                pushScreen(kDomeSeekRightScreen);
                break;
            case kSetFudge:
                pushScreen(kDomeFudgeScreen);
                break;
            case kSetSeekMinDelay:
                pushScreen(kDomeSeekMinDelayScreen);
                break;
            case kSetSeekMaxDelay:
                pushScreen(kDomeSeekMaxDelayScreen);
                break;
            case kSetInvertMotor:
                pushScreen(kSetInvertedScreen);
                break;
            case kSetScaleSpeed:
                pushScreen(kSetSpeedScalingScreen);
                break;
            case kSetAccScale:
                pushScreen(kSetAccScaleScreen);
                break;
            case kSetDecScale:
                pushScreen(kSetDecScaleScreen);
                break;
            case kSetMarcBaudRate:
                pushScreen(kMarcBaudRateScreen);
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
