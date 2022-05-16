///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable PWM pulse dome motor control output
// 
///////////////////////////////////////////////////////////////////////////////

class SetPWMOutputScreen : public ChoiceStrArrayScreen
{
public:
    SetPWMOutputScreen(ScreenID id = kSetPWMOutputScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPWMOutput;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPWMOutput = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPWMOutput = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetPWMOutputScreen sSetPWMOutputScreen;
