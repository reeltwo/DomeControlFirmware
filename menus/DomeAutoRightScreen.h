///////////////////////////////////////////////////////////////////////////////
//
// Specify the maximum number of degrees to randomly seek right.
// Range is 0 - 180 degrees
// 
///////////////////////////////////////////////////////////////////////////////

class DomeAutoRightScreen : public UnsignedValueScreen
{
public:
    DomeAutoRightScreen(ScreenID id = kDomeAutoRightScreen) :
        UnsignedValueScreen(id, MAX_AUTO_RIGHT)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeAutoRight;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeAutoRight = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeAutoRight = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeAutoRightScreen sDomeAutoRightScreen;
