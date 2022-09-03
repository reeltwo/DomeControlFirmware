///////////////////////////////////////////////////////////////////////////////
//
// Enable/disable dome return wifi mode
// 
///////////////////////////////////////////////////////////////////////////////

class WiFiModeScreen : public ChoiceStrArrayScreen
{
public:
    WiFiModeScreen(ScreenID id = kWiFiModeScreen) :
        ChoiceStrArrayScreen(id, sOnOffStrings, sBooleanValues, SizeOfArray(sBooleanValues))
    {
    }

protected:
    virtual unsigned getValue() override
    {
        return wifiEnabled;
    }

    virtual void setValue(unsigned newValue) override
    {
        wifiEnabled = newValue;
    }

    virtual void saveValue(unsigned newValue)
    {
        wifiEnabled = newValue;
        preferences.putBool(PREFERENCE_WIFI_ENABLED, wifiEnabled);
        reboot();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

WiFiModeScreen sWifiModeScreen;
