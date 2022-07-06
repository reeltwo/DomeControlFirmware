///////////////////////////////////////////////////////////////////////////////
//
// Specify minimum delay in seconds for home mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeHomeMinDelayScreen : public UnsignedValueScreen
{
public:
    DomeHomeMinDelayScreen(ScreenID id = kDomeHomeMinDelayScreen) :
        UnsignedValueScreen(id, MAX_SEEK_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeHomeMinDelay;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeHomeMinDelay = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeHomeMinDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeHomeMinDelayScreen sDomeHomeMinDelayScreen;
