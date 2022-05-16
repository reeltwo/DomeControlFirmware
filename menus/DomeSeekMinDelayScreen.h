///////////////////////////////////////////////////////////////////////////////
//
// Specify minimum delay in seconds for random seek left/right mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeSeekMinDelayScreen : public UnsignedValueScreen
{
public:
    DomeSeekMinDelayScreen(ScreenID id = kDomeSeekMinDelayScreen) :
        UnsignedValueScreen(id, MAX_SEEK_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sDomePosition.getDomeMinDelay();
    }

    virtual void setValue(unsigned newValue) override
    {
        sDomePosition.setDomeMinDelay(newValue);
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeMinDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeSeekMinDelayScreen sDomeSeekMinDelayScreen;
