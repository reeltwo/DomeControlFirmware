///////////////////////////////////////////////////////////////////////////////
//
// Allow manual rotation of dome for testing
// 
///////////////////////////////////////////////////////////////////////////////

class RotateDomeScreen : public CommandScreen
{
public:
    RotateDomeScreen(ScreenID id = kRotateDomeScreen) :
        CommandScreen(sDisplay, id)
    {
        setKeyRepeatRate(50);
    }

    virtual void init() override
    {
        fLastDisplayPos = -1;
        // sDomePosition.setDomeDefaultMode(DomePosition::kTarget);
        // sDomePosition.setDomeTargetPosition(sDomePosition.getDomePosition());
    }

    virtual void render() override
    {
        if (millis() > fLastScreenUpdate + 100)
        {
            int16_t pos = 160;//sDomePosition.getHomeRelativeDomePosition();
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

    virtual void buttonUpPressed(bool repeat) override
    {
        // if (sDomePosition.isAtPosition(sDomePosition.getDomeTargetPosition()))
        // {
        //     sDomePosition.setDomeTargetPosition(int(sDomePosition.getDomePosition()) - 90);
        // }
    }

    virtual void buttonLeftPressed(bool repeat) override
    {
        // if (sDomePosition.isAtPosition(sDomePosition.getDomeTargetPosition()))
        // {
        //     sDomePosition.setDomeTargetPosition(sDomePosition.getDomePosition() - 10);
        // }
    }

    virtual void buttonDownPressed(bool repeat) override
    {
        // if (sDomePosition.isAtPosition(sDomePosition.getDomeTargetPosition()))
        // {
        //     sDomePosition.setDomeTargetPosition(sDomePosition.getDomePosition() - 90);
        // }
    }

    virtual void buttonRightPressed(bool repeat) override
    {
        // if (sDomePosition.isAtPosition(sDomePosition.getDomeTargetPosition()))
        // {
        //     sDomePosition.setDomeTargetPosition(sDomePosition.getDomePosition() + 10);
        // }
    }

    virtual void buttonDial(long newValue, long oldValue) override
    {
        long target = (long)fmod(newValue, 24) * 15;
        if (target < 0)
            target += 360;
        // sDomePosition.setDomeTargetPosition(target);
    }

    virtual void buttonInReleased() override
    {
        restoreDomeSettings();
        popScreen();
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

RotateDomeScreen sRotateDomeScreen;
