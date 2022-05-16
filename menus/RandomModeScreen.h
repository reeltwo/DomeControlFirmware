///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable dome random seek left/right mode
// 
///////////////////////////////////////////////////////////////////////////////

class RandomModeScreen : public ChoiceStrArrayScreen
{
public:
    RandomModeScreen(ScreenID id = kRandomModeScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fRandomMode;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fRandomMode = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fRandomMode = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

RandomModeScreen sRandomModeScreen;
