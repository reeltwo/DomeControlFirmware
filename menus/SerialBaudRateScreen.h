///////////////////////////////////////////////////////////////////////////////
//
// Select Command Serial baud rates
// 
///////////////////////////////////////////////////////////////////////////////

class SerialBaudRateScreen : public ChoiceIntArrayScreen
{
public:
    SerialBaudRateScreen(ScreenID id = kSerialBaudRateScreen) :
        ChoiceIntArrayScreen(id, sSerialBaudRates, SizeOfArray(sSerialBaudRates))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fSerialBaudRate;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fSerialBaudRate = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fSerialBaudRate = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SerialBaudRateScreen sSerialBaudRateScreen;
