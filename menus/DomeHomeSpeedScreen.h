///////////////////////////////////////////////////////////////////////////////
//
// Specify the home mode speed. Range is 0 - 100 (default 40)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeHomeSpeedScreen : public UnsignedValueScreen
{
public:
    DomeHomeSpeedScreen(ScreenID id = kDomeHomeSpeedScreen) :
        UnsignedValueScreen(id, MAX_SPEED)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeSpeedHome;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeSpeedHome = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeSpeedHome = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeHomeSpeedScreen sDomeHomeSpeedScreen;
