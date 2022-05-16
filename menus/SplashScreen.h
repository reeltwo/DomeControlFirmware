///////////////////////////////////////////////////////////////////////////////
//
// Startup splash screen showing firmware compilation date
// 
///////////////////////////////////////////////////////////////////////////////

class SplashScreen : public CommandScreen
{
public:
    SplashScreen() :
        CommandScreen(sDisplay, kSplashScreen)
    {}

    virtual void init()
    {
        sDisplay.invertDisplay(false);
        sDisplay.clearDisplay();
        sDisplay.setTextSize(1);
        sDisplay.setTextColor(SSD1306_WHITE);
        sDisplay.setCursor(0, 0);
        sDisplay.println(F("Droid Dome Controller"));
        sDisplay.println(F(__DATE__));
        sDisplay.println(F("READY"));
        sDisplay.display();
    }

    virtual void render()
    {
        if (sDisplay.elapsed() >= 5000)
        {
            switchToScreen(kMainScreen);
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SplashScreen sSplashScreen;
