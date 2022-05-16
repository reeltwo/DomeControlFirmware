///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable PWM pulse input from droid control board (Shadow/Stealth/etc)
// 
///////////////////////////////////////////////////////////////////////////////

class SetPWMInputScreen : public ChoiceStrArrayScreen
{
public:
    SetPWMInputScreen(ScreenID id = kSetPWMInputScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPWMInput;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPWMInput = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPWMInput = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetPWMInputScreen sSetPWMInputScreen;
