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
        sDomePosition.setDomeMode(DomePosition::kOff);
        sDomePosition.setDomeDefaultMode(DomePosition::kOff);
        sDomePosition.setDomeTargetPosition(sDomePosition.getDomePosition());
        fSavedOldValue = false;
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
                popScreen();
            }
        }
    }

    virtual void buttonUpPressed(bool repeat) override
    {
        if (sDomePosition.ready() && sDomePosition.getDomeMode() == DomePosition::kOff)
        {
            setDomeTarget(sDomePosition.getDomePosition() + 90);
        }
    }

    virtual void buttonLeftPressed(bool repeat) override
    {
        if (sDomePosition.ready() && sDomePosition.getDomeMode() == DomePosition::kOff)
        {
            setDomeTarget(sDomePosition.getDomePosition() + 10);
        }
    }

    virtual void buttonDownPressed(bool repeat) override
    {
        if (sDomePosition.ready() && sDomePosition.getDomeMode() == DomePosition::kOff)
        {
            setDomeTarget(int(sDomePosition.getDomePosition()) - 90);
        }
    }

    virtual void buttonRightPressed(bool repeat) override
    {
        if (sDomePosition.ready() && sDomePosition.getDomeMode() == DomePosition::kOff)
        {
            setDomeTarget(int(sDomePosition.getDomePosition()) - 10);
        }
    }

    virtual void buttonDial(long newValue, long oldValue) override
    {
        if (sDomePosition.ready())
        {
            if (!fSavedOldValue)
            {
                fOldValue = oldValue;
                fSavedOldValue = true;
            }
            long target = (long)fmod(newValue-fOldValue, 24) * 15;
            if (target < 0)
                target += 360;
            setDomeTarget(target);
        }
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
    bool fSavedOldValue = false;
    long fOldValue = 0;

    void setDomeTarget(long target)
    {
        sDomePosition.setDomeTargetPosition(target);
        sDomePosition.setDomeMode(DomePosition::kTarget);
        sDomePosition.setTargetReached([]() {
            sDomePosition.setDomeDefaultMode(DomePosition::kOff);
        });
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

RotateDomeScreen sRotateDomeScreen;
