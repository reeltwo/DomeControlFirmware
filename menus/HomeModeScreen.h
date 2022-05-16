///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable dome return home mode
// 
///////////////////////////////////////////////////////////////////////////////

class HomeModeScreen : public ChoiceStrArrayScreen
{
public:
    HomeModeScreen(ScreenID id = kHomeModeScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fHomeMode;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fHomeMode = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fHomeMode = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

HomeModeScreen sHomeModeScreen;
