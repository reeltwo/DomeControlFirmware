///////////////////////////////////////////////////////////////////////////////
//
// Specify minimum delay in seconds for random seek left/right mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeAutoMinDelayScreen : public UnsignedValueScreen
{
public:
    DomeAutoMinDelayScreen(ScreenID id = kDomeAutoMinDelayScreen) :
        UnsignedValueScreen(id, MAX_AUTO_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeAutoMinDelay;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeAutoMinDelay = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeAutoMinDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeAutoMinDelayScreen sDomeAutoMinDelayScreen;
