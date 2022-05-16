///////////////////////////////////////////////////////////////////////////////
//
// Specify the minimum power for dome motor. Range is 0 - 100 (default 15) 
// 
///////////////////////////////////////////////////////////////////////////////

class DomeMinSpeedScreen : public UnsignedValueScreen
{
public:
    DomeMinSpeedScreen(ScreenID id = kDomeMinSpeedScreen) :
        UnsignedValueScreen(id, MAX_SPEED)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeSpeedMin;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeSpeedMin = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeSpeedMin = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeMinSpeedScreen sDomeMinSpeedScreen;
