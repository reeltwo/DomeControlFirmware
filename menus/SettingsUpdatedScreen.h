///////////////////////////////////////////////////////////////////////////////
//
// Startup splash screen showing firmware compilation date
// 
///////////////////////////////////////////////////////////////////////////////

class SettingsUpdatedScreen : public CommandScreen
{
public:
    SettingsUpdatedScreen() :
        CommandScreen(sDisplay, kSettingsUpdatedScreen)
    {}

    virtual void render()
    {
        if (!fShown || sDisplay.needsRedisplay())
        {
            sDisplay.invertDisplay(false);
            sDisplay.clearDisplay();
            sDisplay.setTextSize(4);
            sDisplay.drawTextCentered("OK");
            sDisplay.display();
            fShown = true;
        }
        if (sDisplay.elapsed() >= 500)
        {
            popScreen();
        }
    }

    bool fShown = false;
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SettingsUpdatedScreen sSettingsUpdatedScreen;
