///////////////////////////////////////////////////////////////////////////////
//
// Specify the dome fudge factor in degrees. Range is 0 - MAX_FUDGE_FACTOR (20)
// Dome will stop if position is in range of target +/- fudge degrees
// 
///////////////////////////////////////////////////////////////////////////////

class DomeFudgeScreen : public UnsignedValueScreen
{
public:
    DomeFudgeScreen(ScreenID id = kDomeFudgeScreen) :
        UnsignedValueScreen(id, MAX_FUDGE_FACTOR)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDomeFudge;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDomeFudge = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDomeFudge = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

DomeFudgeScreen sDomeFudgeScreen;
