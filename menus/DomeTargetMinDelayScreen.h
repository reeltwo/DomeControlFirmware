///////////////////////////////////////////////////////////////////////////////
//
// Specify minimum delay in seconds for target mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeTargetMinDelayScreen : public UnsignedValueScreen
{
public:
    DomeTargetMinDelayScreen(ScreenID id = kDomeTargetMinDelayScreen) :
        UnsignedValueScreen(id, MAX_SEEK_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeTargetMinDelay;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeTargetMinDelay = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeTargetMinDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeTargetMinDelayScreen sDomeTargetMinDelayScreen;
