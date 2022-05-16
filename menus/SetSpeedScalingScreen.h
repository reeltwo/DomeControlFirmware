///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable speed scaling (acceleration/deceleration)
// 
///////////////////////////////////////////////////////////////////////////////

class SetSpeedScalingScreen : public ChoiceStrArrayScreen
{
public:
    SetSpeedScalingScreen(ScreenID id = kSetSpeedScalingScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fSpeedScaling;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fSpeedScaling = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fSpeedScaling = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetSpeedScalingScreen sSetSpeedScalingScreen;
