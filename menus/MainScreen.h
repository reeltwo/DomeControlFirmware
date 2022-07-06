///////////////////////////////////////////////////////////////////////////////
//
// Main screen shows current dome position if dome position changes.
// Push button will activate SelectScreen
// 
///////////////////////////////////////////////////////////////////////////////

class MainScreen : public CommandScreen
{
public:
    MainScreen() :
        CommandScreen(sDisplay, kMainScreen)
    {}

    virtual void init() override
    {
        fLastDisplayPos = -1;
    }

    virtual void render() override
    {
        if (millis() > fLastScreenUpdate + 100)
        {
            int16_t pos = 148;//sDomePosition.getHomeRelativeDomePosition();
            if (/*sDomePosition.ready() &&*/ pos != fLastPos)
                fLastPos = pos;
            if (fLastPos != fLastDisplayPos)
            {
                sDisplay.invertDisplay(false);
                sDisplay.clearDisplay();
                sDisplay.setTextSize(4);
                String textPos = String(fLastPos);
                int16_t x1, y1;
                uint16_t w, h;
                sDisplay.getTextBounds(textPos, 0, 0, &x1, &y1, &w, &h);
                sDisplay.setCursor(SCREEN_WIDTH / 2 - w / 2, 0);
                sDisplay.print(textPos);
                sDisplay.display();
                fLastDisplayPos = fLastPos;
            }
            fLastScreenUpdate = millis();
        }
    }

    virtual void buttonInReleased() override
    {
        pushScreen(kSelectScreen);
    }

    virtual bool isActive() override
    {
        // int16_t pos = sDomePosition.getHomeRelativeDomePosition();
        // if (sDomePosition.ready() && pos != fLastPos)
        // {
        //     fLastPos = pos;
            return true;
        // }
        // return false;
    }

protected:
    uint32_t fLastScreenUpdate = 0;
    int16_t fLastPos = -1;
    int16_t fLastDisplayPos = -1;
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

MainScreen sMainScreen;
