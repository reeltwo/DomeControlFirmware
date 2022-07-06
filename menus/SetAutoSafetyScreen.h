///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable dome motor inversion
// 
///////////////////////////////////////////////////////////////////////////////

class SetAutoSafetyScreen : public ChoiceStrArrayScreen
{
public:
    SetAutoSafetyScreen(ScreenID id = kSetAutoSafetyScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fAutoSafety;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fAutoSafety = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fAutoSafety = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetAutoSafetyScreen sSetAutoSafetyScreen;
