///////////////////////////////////////////////////////////////////////////////
//
// Select PWM maximum pulse 1550 - 2400
// 
///////////////////////////////////////////////////////////////////////////////

class SetMaxPulseScreen : public UnsignedValueScreen
{
public:
    SetMaxPulseScreen(ScreenID id = kSetMaxPulseScreen) :
        UnsignedValueScreen(id, 2400, 1550)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPWMMaxPulse;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPWMMaxPulse = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPWMMaxPulse = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetMaxPulseScreen sSetMaxPulseScreen;
