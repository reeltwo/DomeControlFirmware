///////////////////////////////////////////////////////////////////////////////
//
// Specify maximum delay in seconds for target mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeTargetMaxDelayScreen : public UnsignedValueScreen
{
public:
    DomeTargetMaxDelayScreen(ScreenID id = kDomeTargetMaxDelayScreen) :
        UnsignedValueScreen(id, MAX_SEEK_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeTargetMaxDelay;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeTargetMaxDelay = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeTargetMaxDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeTargetMaxDelayScreen sDomeTargetMaxDelayScreen;
