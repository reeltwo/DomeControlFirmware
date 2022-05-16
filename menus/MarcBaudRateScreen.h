///////////////////////////////////////////////////////////////////////////////
//
// Select Marcduino baud rates
// 
///////////////////////////////////////////////////////////////////////////////

class MarcBaudRateScreen : public ChoiceIntArrayScreen
{
public:
    MarcBaudRateScreen(ScreenID id = kMarcBaudRateScreen) :
        ChoiceIntArrayScreen(id, sMarcBaudRates, SizeOfArray(sMarcBaudRates))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fMarcBaudRate;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fMarcBaudRate = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fMarcBaudRate = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

MarcBaudRateScreen sMarcBaudRateScreen;
