///////////////////////////////////////////////////////////////////////////////
//
// Select Sabertooth/Syren packet serial baud rates
// 
///////////////////////////////////////////////////////////////////////////////

class SaberBaudRateScreen : public ChoiceIntArrayScreen
{
public:
    SaberBaudRateScreen(ScreenID id = kSaberBaudRateScreen) :
        ChoiceIntArrayScreen(id, sSaberBaudRates, SizeOfArray(sSaberBaudRates))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fSaberBaudRate;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fSaberBaudRate = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fSaberBaudRate = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SaberBaudRateScreen sSaberBaudRateScreen;
