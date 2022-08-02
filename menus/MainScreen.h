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
        if (sDomePosition.ready())
        {
            if (millis() > fLastScreenUpdate + 100)
            {
                int16_t pos = sDomePosition.getHomeRelativeDomePosition();
                if (sDomePosition.ready() && pos != fLastPos)
                    fLastPos = pos;
                if (fLastPos != fLastDisplayPos)
                {
                    sDisplay.invertDisplay(false);
                    sDisplay.clearDisplay();
                    sDisplay.setTextSize(4);
                    sDisplay.drawTextCentered(String(fLastPos));
                    sDisplay.display();
                    fLastDisplayPos = fLastPos;
                }
                fLastScreenUpdate = millis();
            }
        }
        else
        {
            if (fLastDisplayPos != -2)
            {
                sDisplay.invertDisplay(true);
                sDisplay.clearDisplay();
                sDisplay.setTextSize(4);
                sDisplay.drawTextCentered("N/A");
                sDisplay.display();
                fLastDisplayPos = -2;
            }
            if (sDisplay.elapsed() >= 2000)
            {
                pushScreen(kSelectScreen);
            }
        }
    }

    virtual void buttonInReleased() override
    {
        pushScreen(kSelectScreen);
    }

#ifdef USE_DROID_REMOTE
    virtual void buttonLeftPressed(bool repeat) override
    {
        if (remoteEnabled)
        {
        #ifdef USE_SMQ
            if (SMQ::sendTopic("EXIT", "Remote"))
            {
                SMQ::sendString("addr", SMQ::getAddress());
                SMQ::sendEnd();
                sDisplay.setEnabled(false);
            }
        #endif
        }
    }
#endif

    virtual bool isActive() override
    {
        if (sDomePosition.ready())
        {
            int16_t pos = sDomePosition.getHomeRelativeDomePosition();
            if (sDomePosition.ready() && pos != fLastPos)
            {
                fLastPos = pos;
                return true;
            }
        }
        return false;
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
