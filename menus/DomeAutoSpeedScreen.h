///////////////////////////////////////////////////////////////////////////////
//
// Specify the random seek speed. Range is 0 - 100 (default 30)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeAutoSpeedScreen : public UnsignedValueScreen
{
public:
    DomeAutoSpeedScreen(ScreenID id = kDomeAutoSpeedScreen) :
        UnsignedValueScreen(id, MAX_SPEED)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeSpeedAuto;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeSpeedAuto = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeSpeedAuto = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeAutoSpeedScreen sDomeAutoSpeedScreen;
