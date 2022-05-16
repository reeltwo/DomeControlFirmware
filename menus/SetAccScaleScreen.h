///////////////////////////////////////////////////////////////////////////////
//
// Select deceleration scale value 0 - MAX_ACC_SCALE (255 default)
// 
///////////////////////////////////////////////////////////////////////////////

class SetAccScaleScreen : public UnsignedValueScreen
{
public:
    SetAccScaleScreen(ScreenID id = kSetAccScaleScreen) :
        UnsignedValueScreen(id, MAX_ACC_SCALE)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fAccScale;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fAccScale = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fAccScale = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetAccScaleScreen sSetAccScaleScreen;
