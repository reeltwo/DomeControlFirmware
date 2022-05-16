///////////////////////////////////////////////////////////////////////////////
//
// Specify the random seek speed. Range is 0 - 100 (default 30)
// 
///////////////////////////////////////////////////////////////////////////////

class DomeSeekSpeedScreen : public UnsignedValueScreen
{
public:
    DomeSeekSpeedScreen(ScreenID id = kDomeSeekSpeedScreen) :
        UnsignedValueScreen(id, MAX_SPEED)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeSpeedSeek;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeSpeedSeek = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeSpeedSeek = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeSeekSpeedScreen sDomeSeekSpeedScreen;
