///////////////////////////////////////////////////////////////////////////////
//
// Select deceleration scale value 0 - MAX_DEC_SCALE (255 default)
// 
///////////////////////////////////////////////////////////////////////////////

class SetDecScaleScreen : public UnsignedValueScreen
{
public:
    SetDecScaleScreen(ScreenID id = kSetDecScaleScreen) :
        UnsignedValueScreen(id, MAX_DEC_SCALE)
    {}

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fDecScale;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fDecScale = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fDecScale = newValue;
        restoreDomeSettings();
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetDecScaleScreen sSetDecScaleScreen;
