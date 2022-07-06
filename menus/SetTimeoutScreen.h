///////////////////////////////////////////////////////////////////////////////
//
// Set timeout value 0 - MAX_ACC_SCALE (255 default)
// 
///////////////////////////////////////////////////////////////////////////////

class SetTimeoutScreen : public UnsignedValueScreen
{
public:
    SetTimeoutScreen(ScreenID id = kSetTimeoutScreen) :
        UnsignedValueScreen(id, MAX_TIMEOUT)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fTimeout;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fTimeout = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fTimeout = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetTimeoutScreen sSetTimeoutScreen;
