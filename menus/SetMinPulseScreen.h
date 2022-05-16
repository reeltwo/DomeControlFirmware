///////////////////////////////////////////////////////////////////////////////
//
// Select PWM minimum pulse 600 - 1450
// 
///////////////////////////////////////////////////////////////////////////////

class SetMinPulseScreen : public UnsignedValueScreen
{
public:
    SetMinPulseScreen(ScreenID id = kSetMinPulseScreen) :
        UnsignedValueScreen(id, 1450, 600)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPWMMinPulse;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPWMMinPulse = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPWMMinPulse = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetMinPulseScreen sSetMinPulseScreen;
