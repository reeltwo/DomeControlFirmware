///////////////////////////////////////////////////////////////////////////////
//
// Specify the maximum number of degrees to randomly seek left.
// Range is 0 - 180 degrees
// 
///////////////////////////////////////////////////////////////////////////////

class DomeAutoLeftScreen : public UnsignedValueScreen
{
public:
    DomeAutoLeftScreen(ScreenID id = kDomeAutoLeftScreen) :
        UnsignedValueScreen(id, MAX_AUTO_LEFT)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeAutoLeft;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeAutoLeft = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeAutoLeft = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeAutoLeftScreen sDomeAutoLeftScreen;
