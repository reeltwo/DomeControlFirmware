///////////////////////////////////////////////////////////////////////////////
//
// Specify the maximum number of degrees to randomly seek right.
// Range is 0 - 180 degrees
// 
///////////////////////////////////////////////////////////////////////////////

class DomeSeekRightScreen : public UnsignedValueScreen
{
public:
    DomeSeekRightScreen(ScreenID id = kDomeSeekRightScreen) :
        UnsignedValueScreen(id, MAX_SEEK_RIGHT)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeSeekRight;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeSeekRight = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeSeekRight = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeSeekRightScreen sDomeSeekRightScreen;
