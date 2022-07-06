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

    virtual void init()
    {
        sDisplay.invertDisplay(false);
        sDisplay.clearDisplay();
        sDisplay.setTextSize(4);
        String textPos = "OK";
        int16_t x1, y1;
        uint16_t w, h;
        sDisplay.getTextBounds(textPos, 0, 0, &x1, &y1, &w, &h);
        sDisplay.setCursor(SCREEN_WIDTH / 2 - w / 2, 0);
        sDisplay.println(textPos);
        sDisplay.display();
    }

    virtual void render()
    {
        if (sDisplay.elapsed() >= 500)
        {
            popScreen();
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SettingsUpdatedScreen sSettingsUpdatedScreen;
