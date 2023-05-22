///////////////////////////////////////////////////////////////////////////////
//
// Select Sabertooth/Syren packet serial baud rates
// 
///////////////////////////////////////////////////////////////////////////////

class SaberBaudRateScreen : public ChoiceIntArrayScreen
{
public:
    SaberBaudRateScreen(ScreenID id = kSaberBaudRateScreen) :
        ChoiceIntArrayScreen(id, sBaudRates, SizeOfArray(sBaudRates))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fSyrenBaudRate;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fSyrenBaudRate = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fSyrenBaudRate = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SaberBaudRateScreen sSaberBaudRateScreen;
