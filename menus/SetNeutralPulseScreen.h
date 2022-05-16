///////////////////////////////////////////////////////////////////////////////
//
// Select PWM neutral pulse 1451 - 1549
// 
///////////////////////////////////////////////////////////////////////////////

class SetNeutralPulseScreen : public UnsignedValueScreen
{
public:
    SetNeutralPulseScreen(ScreenID id = kSetNeutralPulseScreen) :
        UnsignedValueScreen(id, 1549, 1451)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPWMNeutralPulse;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPWMNeutralPulse = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPWMNeutralPulse = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetNeutralPulseScreen sSetNeutralPulseScreen;
