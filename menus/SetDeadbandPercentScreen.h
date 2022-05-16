///////////////////////////////////////////////////////////////////////////////
//
// Select PWM deadband percentage 0 - 100
// 
///////////////////////////////////////////////////////////////////////////////

class SetDeadbandPercentScreen : public UnsignedValueScreen
{
public:
    SetDeadbandPercentScreen(ScreenID id = kSetDeadbandPercentScreen) :
        UnsignedValueScreen(id, 100)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPWMDeadbandPercent;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPWMDeadbandPercent = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPWMDeadbandPercent = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetDeadbandPercentScreen sSetDeadbandPercentScreen;
