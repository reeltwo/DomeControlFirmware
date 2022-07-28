///////////////////////////////////////////////////////////////////////////////
//
// Specify maximum delay in seconds for random seek left/right mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeAutoMaxDelayScreen : public UnsignedValueScreen
{
public:
    DomeAutoMaxDelayScreen(ScreenID id = kDomeAutoMaxDelayScreen) :
        UnsignedValueScreen(id, MAX_AUTO_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeAutoMaxDelay;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeAutoMaxDelay = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeAutoMaxDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeAutoMaxDelayScreen sDomeAutoMaxDelayScreen;
