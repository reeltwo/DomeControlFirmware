///////////////////////////////////////////////////////////////////////////////
//
// Specify the dome speed. Range is 0 - 100 (default 50) 
// 
///////////////////////////////////////////////////////////////////////////////

class DomeSpeedScreen : public UnsignedValueScreen
{
public:
    DomeSpeedScreen(ScreenID id = kDomeSpeedScreen) :
        UnsignedValueScreen(id, MAX_SPEED)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fMaxSpeed;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fMaxSpeed = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fMaxSpeed = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeSpeedScreen sDomeSpeedScreen;
