///////////////////////////////////////////////////////////////////////////////
//
// Command screen handler for SSD 1306 using a rotary dial for events
// 
///////////////////////////////////////////////////////////////////////////////

class CommandScreenHandlerSSD1306: public Adafruit_SSD1306, public CommandScreenHandler
{
public:
    CommandScreenHandlerSSD1306() :
        Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1),
        fDial(PIN_ENCODER_A, PIN_ENCODER_B, BUTTON_UP, BUTTON_LEFT, BUTTON_DOWN, BUTTON_RIGHT, BUTTON_IN)
    {
    }

    virtual void sleepDevice() override
    {
        invertDisplay(false);
        clearDisplay();
        display();
        switchToScreen(kMainScreen);
    }

    virtual bool handleEvent()
    {
        bool ret = false;
        CommandScreen* currentScr = current();
        if (currentScr == nullptr)
            return ret;
        unsigned keyRepeatRate = currentScr->getKeyRepeatRate();
        if (keyRepeatRate == 0)
            keyRepeatRate = KEY_REPEAT_RATE_MS;
        if (fDial.hasChanged() || fDial.getButtonPressedMask() != 0)
        {
            // Reset the screen blanking timer if the encoder changed
            // or if any of the buttons are currently pressed
            ret = true;
        }
        if (fDial.hasChanged())
        {
            long dialValue = -fDial.getValue();
            currentScr->buttonDial(dialValue, fDialValue);
            fDialValue = dialValue;
        }
        if (fDial.hasButtonStateChanged())
        {
            if (fDial.isButtonPressed(BUTTON_UP))
                currentScr->buttonUpPressed();
            if (fDial.isButtonPressed(BUTTON_LEFT))
                currentScr->buttonLeftPressed();
            if (fDial.isButtonPressed(BUTTON_DOWN))
                currentScr->buttonDownPressed();
            if (fDial.isButtonPressed(BUTTON_RIGHT))
                currentScr->buttonRightPressed();
            if (fDial.isButtonPressed(BUTTON_IN))
                currentScr->buttonInPressed();
            if (fLastScreenID == currentScr->ID())
            {
                if (fDial.isButtonReleased(BUTTON_UP))
                    currentScr->buttonUpReleased();
                if (fDial.isButtonReleased(BUTTON_LEFT))
                    currentScr->buttonLeftReleased();
                if (fDial.isButtonReleased(BUTTON_DOWN))
                    currentScr->buttonDownReleased();
                if (fDial.isButtonReleased(BUTTON_RIGHT))
                    currentScr->buttonRightReleased();
                if (fDial.isButtonReleased(BUTTON_IN))
                    currentScr->buttonInReleased();
            }
            fLastKeyEvent = millis();
            fLastScreenID = currentScr->ID();
        }
        else if (fLastScreenID == currentScr->ID())
        {
            if (fDial.isButtonPressed(BUTTON_UP))
            {
                if (fLastKeyEvent + keyRepeatRate < millis())
                {
                    currentScr->buttonUpPressed(true);
                    fLastKeyEvent = millis();
                }
            }
            else if (fDial.isButtonPressed(BUTTON_LEFT))
            {
                if (fLastKeyEvent + keyRepeatRate < millis())
                {
                    currentScr->buttonLeftPressed(true);
                    fLastKeyEvent = millis();
                }
            }
            else if (fDial.isButtonPressed(BUTTON_DOWN))
            {
                if (fLastKeyEvent + keyRepeatRate < millis())
                {
                    currentScr->buttonDownPressed(true);
                    fLastKeyEvent = millis();
                }
            }
            else if (fDial.isButtonPressed(BUTTON_RIGHT))
            {
                if (fLastKeyEvent + keyRepeatRate < millis())
                {
                    currentScr->buttonRightPressed(true);
                    fLastKeyEvent = millis();
                }
            }
            else if (fDial.isButtonPressed(BUTTON_IN))
            {
                if (fLastKeyEvent + keyRepeatRate < millis())
                {
                    currentScr->buttonInPressed(true);
                    fLastKeyEvent = millis();
                }
            }
        }
        return ret;
    }

    AnoRotaryEncoder    fDial;
    long                fDialValue = 0;
    uint32_t            fLastKeyEvent = 0;
    ScreenID            fLastScreenID = kInvalid;
};
