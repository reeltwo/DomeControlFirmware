///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable packet serial dome motor control output
// 
///////////////////////////////////////////////////////////////////////////////

class PacketSerialOutputScreen : public ChoiceStrArrayScreen
{
public:
    PacketSerialOutputScreen(ScreenID id = kPacketSerialOutputScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPacketSerialOutput;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPacketSerialOutput = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPacketSerialOutput = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

PacketSerialOutputScreen sPacketSerialOutputScreen;
