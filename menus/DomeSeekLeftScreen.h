///////////////////////////////////////////////////////////////////////////////
//
// Specify the maximum number of degrees to randomly seek left.
// Range is 0 - 180 degrees
// 
///////////////////////////////////////////////////////////////////////////////

class DomeSeekLeftScreen : public UnsignedValueScreen
{
public:
    DomeSeekLeftScreen(ScreenID id = kDomeSeekLeftScreen) :
        UnsignedValueScreen(id, MAX_SEEK_LEFT)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeSeekLeft;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeSeekLeft = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeSeekLeft = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeSeekLeftScreen sDomeSeekLeftScreen;
