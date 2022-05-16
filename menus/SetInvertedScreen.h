///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable dome motor inversion
// 
///////////////////////////////////////////////////////////////////////////////

class SetInvertedScreen : public ChoiceStrArrayScreen
{
public:
    SetInvertedScreen(ScreenID id = kSetInvertedScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fInverted;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fInverted = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fInverted = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetInvertedScreen sSetInvertedScreen;
