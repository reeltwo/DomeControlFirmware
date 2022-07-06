///////////////////////////////////////////////////////////////////////////////
//
// Specify maximum delay in seconds for random seek left/right mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeSeekMaxDelayScreen : public UnsignedValueScreen
{
public:
    DomeSeekMaxDelayScreen(ScreenID id = kDomeSeekMaxDelayScreen) :
        UnsignedValueScreen(id, MAX_SEEK_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeSeekMaxDelay;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeSeekMaxDelay = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeSeekMaxDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeSeekMaxDelayScreen sDomeSeekMaxDelayScreen;
