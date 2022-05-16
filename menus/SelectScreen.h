///////////////////////////////////////////////////////////////////////////////
//
// Top level selection menu
// 
///////////////////////////////////////////////////////////////////////////////

static const char* sSelectMenu[] = {
    "Random\nMode",
    "Home\nMode",
    "Rotate\nDome",
    "Change\nSettings"
};

class SelectScreen : public MenuScreen
{
public:
    enum {
        kRandomMode,
        kHomeMode,
        kRotateDome,
        kChangeSettings
    };
    SelectScreen() :
        MenuScreen(kSelectScreen, sSelectMenu, SizeOfArray(sSelectMenu))
    {}

    virtual void buttonInReleased() override
    {
        switch (fCurrentItem)
        {
            case kRandomMode:
                pushScreen(kRandomModeScreen);
                break;
            case kHomeMode:
                pushScreen(kHomeModeScreen);
                break;
            case kRotateDome:
                pushScreen(kRotateDomeScreen);
                break;
            case kChangeSettings:
                pushScreen(kSettingsScreen);
                break;
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SelectScreen sSelectScreen;
