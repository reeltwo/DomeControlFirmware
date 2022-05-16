///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable packet serial input from droid control board (Shadow/Stealth/etc)
// 
///////////////////////////////////////////////////////////////////////////////

class PacketSerialInputScreen : public ChoiceStrArrayScreen
{
public:
    PacketSerialInputScreen(ScreenID id = kPacketSerialInputScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return sSettings.fPacketSerialInput;
    }

    virtual void setValue(unsigned newValue) override
    {
        sSettings.fPacketSerialInput = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        sSettings.fPacketSerialInput = newValue;
        sSettings.write();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

PacketSerialInputScreen sPacketSerialInputScreen;
