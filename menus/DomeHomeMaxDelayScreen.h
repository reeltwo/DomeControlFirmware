///////////////////////////////////////////////////////////////////////////////
//
// Specify maximum delay in seconds for home mode
// Range is 0 - MAX_SEEK_DELAY (255)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeHomeMaxDelayScreen : public UnsignedValueScreen
{
public:
    DomeHomeMaxDelayScreen(ScreenID id = kDomeHomeMaxDelayScreen) :
        UnsignedValueScreen(id, MAX_SEEK_DELAY)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeHomeMaxDelay;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeHomeMaxDelay = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeHomeMaxDelay = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeHomeMaxDelayScreen sDomeHomeMaxDelayScreen;
