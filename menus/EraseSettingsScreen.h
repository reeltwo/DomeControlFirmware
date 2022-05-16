///////////////////////////////////////////////////////////////////////////////
//
// Erase all settings
// 
///////////////////////////////////////////////////////////////////////////////

class EraseSettingsScreen : public ChoiceStrArrayScreen
{
public:
    EraseSettingsScreen(ScreenID id = kEraseSettingsScreen) :
        ChoiceStrArrayScreen(id, sYesNoStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

    virtual void init() override
    {
        ChoiceStrArrayScreen::init();
        fSelected = false;
    }

protected:
    unsigned fSelected = false;

    virtual unsigned getValue() override
    {
        return fSelected;
    }

    virtual void setValue(unsigned newValue) override
    {
        fSelected = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        DomeControllerSettings defaultSettings;
        *sSettings.data() = defaultSettings;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

EraseSettingsScreen sEraseSettingsScreen;
