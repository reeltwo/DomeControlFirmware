///////////////////////////////////////////////////////////////////////////////
//
// Web Configuration Interface for RDH
// 
///////////////////////////////////////////////////////////////////////////////

#ifdef USE_WIFI_WEB

////////////////////////////////

int cmdSerialBaud;
bool cmdSerialPass;
bool cmdSerialEnabled;
bool cmdWifiEnabled;
bool cmdWifiSerialPass;

WElement serialContents[] = {
    WSelect("Serial Baud Rate", "serialbaud",
        sSerialBaudRatesStr, SizeOfArray(sSerialBaudRatesStr),
        []() {
            cmdSerialBaud = sSettings.fSerialBaudRate;
            for (int i = 0; i < SizeOfArray(sSerialBaudRates); i++)
            {
                if (sSerialBaudRates[i] == cmdSerialBaud)
                    return i;
            }
            return 0;
        },
        [](int val) {
            cmdSerialBaud = sSerialBaudRates[val];
        } ),
    WVerticalAlign(),
    WCheckbox("Commands on Wifi (port 2000)", "wificmds",
        []() { return (cmdWifiEnabled = (preferences.getBool(PREFERENCE_MARCWIFI_ENABLED, MARC_WIFI_ENABLED))); },
        [](bool val) { cmdWifiEnabled = val; } ),
    WVerticalAlign(),
    WCheckbox("Commands Wifi pass-through to Serial", "wifipass",
        []() { return (cmdWifiSerialPass = (preferences.getBool(PREFERENCE_MARCWIFI_SERIAL_PASS, MARC_WIFI_SERIAL_PASS))); },
        [](bool val) { cmdWifiSerialPass = val; } ),
    WVerticalAlign(),
    WButton("Save", "save", []() {
        if (cmdSerialBaud != sSettings.fSerialBaudRate)
        {
            sSettings.fSerialBaudRate = cmdSerialBaud;
            sUpdateSettings = true;
        }
        preferences.putBool(PREFERENCE_MARCWIFI_ENABLED, cmdWifiEnabled);
        preferences.putBool(PREFERENCE_MARCWIFI_SERIAL_PASS, cmdWifiSerialPass);
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

////////////////////////////////

static bool sDomePosSettingsChanged;

WElement domePositionContents[] = {
    WSlider("Home Position", "homepos", 0, 359,
        []()->int { return sSettings.fHomePosition; },
        [](int val) { sSettings.fHomePosition = val; sDomePosSettingsChanged = true; } ),
    WButton("Save", "save", []() {
        if (sDomePosSettingsChanged)
        {
            sUpdateSettings = true;
        }
        sUpdateSettings = true;
    }),
    WHorizontalAlign(),
    WButtonReload("Defaults", "default", []() {
        sSettings.fHomePosition = DEFAULT_HOME_POSITION;
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};
////////////////////////////////

static bool sDomeSettingsChanged;

WElement domeContents[] = {
    WCheckbox("Packet Serial Input", "saberin",
        []() { return sSettings.fPacketSerialInput; },
        [](bool val) { sSettings.fPacketSerialInput = val; sDomeSettingsChanged = true; } ),
    WCheckbox("Packet Serial Output", "saberout",
        []() { return sSettings.fPacketSerialOutput; },
        [](bool val) { sSettings.fPacketSerialOutput = val; sDomeSettingsChanged = true; } ),
    WCheckbox("Pulse Input", "pulsein",
        []() { return sSettings.fPWMInput; },
        [](bool val) { sSettings.fPWMInput = val; sDomeSettingsChanged = true; } ),
    WCheckbox("Pulse Output", "pulseout",
        []() { return sSettings.fPWMOutput; },
        [](bool val) { sSettings.fPWMOutput = val; sDomeSettingsChanged = true; } ),
    WCheckbox("Invert Motor Output", "invert",
        []() { return sSettings.fInverted; },
        [](bool val) { sSettings.fInverted = val; sDomeSettingsChanged = true; } ),
    WCheckbox("Auto Safety Check", "autosafety",
        []() { return sSettings.fAutoSafety; },
        [](bool val) { sSettings.fAutoSafety = val; sDomeSettingsChanged = true; } ),
    WSlider("Maximum Speed", "maxspeed", 0, MAX_SPEED,
        []()->int { return sSettings.fMaxSpeed; },
        [](int val) { sSettings.fMaxSpeed = val; sDomeSettingsChanged = true; } ),
    WSlider("Fudge Degrees", "domefudge", 0, MAX_FUDGE_FACTOR,
        []()->int { return sSettings.fDomeFudge; },
        [](int val) { sSettings.fDomeFudge = val; sDomeSettingsChanged = true; } ),
    WButton("Save", "save", []() {
        if (sDomeSettingsChanged)
        {
            sUpdateSettings = true;
        }
        sDomeSettingsChanged = false;
    }),
    WHorizontalAlign(),
    WButtonReload("Defaults", "default", []() {
        sSettings.fMaxSpeed = DEFAULT_MAX_SPEED;
        sSettings.fPacketSerialInput = DEFAULT_PACKET_SERIAL_INPUT;
        sSettings.fPacketSerialOutput = DEFAULT_PACKET_SERIAL_OUTPUT;
        sSettings.fPWMInput = DEFAULT_PWM_INPUT;
        sSettings.fPWMOutput = DEFAULT_PWM_OUTPUT;
        sSettings.fDomeAutoLeft = DEFAULT_DOME_FUDGE;
        sSettings.fDomeAutoRight = DEFAULT_DOME_AUTO_RIGHT;
        sSettings.fDomeAutoMinDelay = DEFAULT_DOME_AUTO_MIN_DELAY;
        sSettings.fDomeAutoMaxDelay = DEFAULT_DOME_AUTO_MAX_DELAY;
        sSettings.fDomeSpeedAuto = DEFAULT_DOME_SPEED_AUTO;
        sSettings.fInverted = DEFAULT_INVERTED;
        sSettings.fAutoSafety = DEFAULT_AUTO_SAFETY;
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

////////////////////////////////

static bool sAutoDomeSettingsChanged;

WElement autoDomeContents[] = {
    WCheckbox("Automatic Dome Movement", "seekmode",
        []() { return sSettings.fRandomMode; },
        [](bool val) { sSettings.fRandomMode = val; sAutoDomeSettingsChanged = true; } ),
    WSlider("Auto Left Distance Degrees", "seekleft", 0, MAX_AUTO_LEFT,
        []()->int { return sSettings.fDomeAutoLeft; },
        [](int val) { sSettings.fDomeAutoLeft = val; sAutoDomeSettingsChanged = true; } ),
    WSlider("Auto Right Distance Degrees", "seekright", 0, MAX_AUTO_RIGHT,
        []()->int { return sSettings.fDomeAutoRight; },
        [](int val) { sSettings.fDomeAutoRight = val; sAutoDomeSettingsChanged = true; } ),
    WSlider("Auto Minimum Delay Seconds", "mindelay", 0, MAX_AUTO_DELAY,
        []()->int { return sSettings.fDomeAutoMinDelay; },
        [](int val) { sSettings.fDomeAutoMinDelay = val; sAutoDomeSettingsChanged = true; } ),
    WSlider("Auto Maximum Delay Seconds", "maxdelay", 0, MAX_AUTO_DELAY,
        []()->int { return sSettings.fDomeAutoMaxDelay; },
        [](int val) { sSettings.fDomeAutoMaxDelay = val; sAutoDomeSettingsChanged = true; } ),
    WSlider("Auto Dome Speed", "autospeed", 0, MAX_SPEED,
        []()->int { return sSettings.fDomeSpeedAuto; },
        [](int val) { sSettings.fDomeSpeedAuto = val; sAutoDomeSettingsChanged = true; } ),
    WButton("Save", "save", []() {
        if (sAutoDomeSettingsChanged)
        {
            sUpdateSettings = true;
        }
        sAutoDomeSettingsChanged = false;
    }),
    WHorizontalAlign(),
    WButtonReload("Defaults", "default", []() {
        sSettings.fRandomMode = DEFAULT_RANDOM_MODE;
        sSettings.fDomeAutoLeft = DEFAULT_DOME_AUTO_LEFT;
        sSettings.fDomeAutoRight = DEFAULT_DOME_AUTO_RIGHT;
        sSettings.fDomeAutoMinDelay = DEFAULT_DOME_AUTO_MIN_DELAY;
        sSettings.fDomeAutoMaxDelay = DEFAULT_DOME_AUTO_MAX_DELAY;
        sSettings.fDomeSpeedAuto = DEFAULT_DOME_SPEED_AUTO;
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

////////////////////////////////

static bool sHomeDomeSettingsChanged;

WElement homeDomeContents[] = {
    WCheckbox("Automatically Roam Dome Home", "homemode",
        []() { return sSettings.fHomeMode; },
        [](bool val) { sSettings.fHomeMode = val; sHomeDomeSettingsChanged = true; } ),
    WSlider("Home Minimum Delay", "mindelay", 0, MAX_AUTO_DELAY,
        []()->int { return sSettings.fDomeHomeMinDelay; },
        [](int val) { sSettings.fDomeHomeMinDelay = val; sHomeDomeSettingsChanged = true; } ),
    WSlider("Home Maximum Delay", "maxdelay", 0, MAX_AUTO_DELAY,
        []()->int { return sSettings.fDomeHomeMaxDelay; },
        [](int val) { sSettings.fDomeHomeMaxDelay = val; sHomeDomeSettingsChanged = true; } ),
    WSlider("Home Dome Speed", "autospeed", 0, MAX_SPEED,
        []()->int { return sSettings.fDomeSpeedHome; },
        [](int val) { sSettings.fDomeSpeedHome = val; sAutoDomeSettingsChanged = true; } ),
    WButton("Save", "save", []() {
        if (sHomeDomeSettingsChanged)
        {
            sUpdateSettings = true;
        }
        sHomeDomeSettingsChanged = false;
    }),
    WHorizontalAlign(),
    WButtonReload("Defaults", "default", []() {
        sSettings.fHomeMode = DEFAULT_HOME_MODE;
        sSettings.fDomeHomeMinDelay = DEFAULT_DOME_AUTO_MIN_DELAY;
        sSettings.fDomeHomeMaxDelay = DEFAULT_DOME_HOME_MAX_DELAY;
        sSettings.fDomeSpeedHome = DEFAULT_DOME_SPEED_HOME;
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

////////////////////////////////

String wifiSSID;
String wifiPass;
bool wifiAP;

WElement wifiContents[] = {
    W1("WiFi Setup"),
    WCheckbox("WiFi Enabled", "enabled",
        []() { return wifiEnabled; },
        [](bool val) { wifiEnabled = val; } ),
    WHR(),
    WCheckbox("Access Point", "apmode",
        []() { return (wifiAP = preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT)); },
        [](bool val) { wifiAP = val; } ),
    WTextField("WiFi:", "wifi",
        []()->String { return (wifiSSID = preferences.getString(PREFERENCE_WIFI_SSID, getHostName())); },
        [](String val) { wifiSSID = val; } ),
    WPassword("Password:", "password",
        []()->String { return (wifiPass = preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE)); },
        [](String val) { wifiPass = val; } ),
    WLabel("WiFi Disables Droid Remote", "label2"),
    WHR(),
    WButton("Save", "save", []() {
        DEBUG_PRINTLN("WiFi Changed");
        preferences.putBool(PREFERENCE_WIFI_ENABLED, wifiEnabled);
        preferences.putBool(PREFERENCE_WIFI_AP, wifiAP);
        preferences.putString(PREFERENCE_WIFI_SSID, wifiSSID);
        preferences.putString(PREFERENCE_WIFI_PASS, wifiPass);
        reboot();
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

////////////////////////////////

String remoteHostName;
String remoteSecret;

WElement remoteContents[] = {
    W1("Droid Remote Setup"),
    WCheckbox("Droid Remote Enabled", "remoteenabled",
        []() { return remoteEnabled; },
        [](bool val) { remoteEnabled = val; } ),
    WHR(),
    WTextField("Device Name:", "hostname",
        []()->String { return (remoteHostName = preferences.getString(PREFERENCE_REMOTE_HOSTNAME, SMQ_HOSTNAME)); },
        [](String val) { remoteHostName = val; } ),
    WPassword("Secret:", "secret",
        []()->String { return (remoteSecret = preferences.getString(PREFERENCE_REMOTE_SECRET, SMQ_SECRET)); },
        [](String val) { remoteSecret = val; } ),
    WButton("Save", "save", []() {
        DEBUG_PRINTLN("Remote Changed");
        preferences.putBool(PREFERENCE_REMOTE_ENABLED, remoteEnabled);
        preferences.putString(PREFERENCE_REMOTE_HOSTNAME, remoteHostName);
        preferences.putString(PREFERENCE_REMOTE_SECRET, remoteSecret);
        reboot();
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

////////////////////////////////

WElement firmwareContents[] = {
    W1("Firmware Setup"),
    WFirmwareFile("Firmware:", "firmware"),
    WFirmwareUpload("Reflash", "firmware"),
    WLabel("Current Firmware Build Date:", "label"),
    WLabel(__DATE__, "date"),
#ifdef BUILD_VERSION
    WHRef(BUILD_VERSION, "Sources"),
#endif
    WButton("Clear Prefs", "clear", []() {
        DEBUG_PRINTLN("Clear all preference settings");
        preferences.clear();
    }),
    WHorizontalAlign(),
    WButton("Reboot", "reboot", []() {
        DEBUG_PRINTLN("Rebooting");
        preferences.end();
        ESP.restart();
    }),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

////////////////////////////////

WMenuData mainMenu[] = {
    { "Dome Position", "/domepos" },
    { "Dome Settings", "/dome" },
    { "Auto Movement", "/auto" },
    { "Home Movement", "/home" },
    { "Serial Commands", "/serial" },
    { "WiFi", "/wifi" },
    { "Remote", "/remote" },
    { "Firmware", "/firmware" },
    { "Back", "/" }
};

WElement mainContents[] = {
    WVerticalMenu("menu", mainMenu, SizeOfArray(mainMenu)),
    rseriesSVG
};


WPage pages[] = {
    WPage("/", mainContents, SizeOfArray(mainContents)),
      WPage("/domepos", domePositionContents, SizeOfArray(domePositionContents)),
      WPage("/dome", domeContents, SizeOfArray(domeContents)),
      WPage("/auto", autoDomeContents, SizeOfArray(autoDomeContents)),
      WPage("/home", homeDomeContents, SizeOfArray(homeDomeContents)),
      WPage("/serial", serialContents, SizeOfArray(serialContents)),
      WPage("/wifi", wifiContents, SizeOfArray(wifiContents)),
      WPage("/remote", remoteContents, SizeOfArray(remoteContents)),
      WPage("/firmware", firmwareContents, SizeOfArray(firmwareContents)),
        WUpload("/upload/firmware",
            [](Client& client)
            {
                if (Update.hasError())
                    client.println("HTTP/1.0 200 FAIL");
                else
                    client.println("HTTP/1.0 200 OK");
                client.println("Content-type:text/html");
                client.println("Vary: Accept-Encoding");
                client.println();
                client.println();
                client.stop();
                if (!Update.hasError())
                {
                    reboot();
                    delay(1000);
                    preferences.end();
                    ESP.restart();
                }
            #ifdef USE_OTA
                otaInProgress = false;
            #endif
            },
            [](WUploader& upload)
            {
                if (upload.status == UPLOAD_FILE_START)
                {
                    otaInProgress = true;
                    unmountFileSystems();
                    Serial.printf("Update: %s\n", upload.filename.c_str());
                    if (!Update.begin(upload.fileSize))
                    {
                        //start with max available size
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_WRITE)
                {
                    float range = (float)upload.receivedSize / (float)upload.fileSize;
                    DEBUG_PRINTLN("Received: "+String(range*100)+"%");
                   /* flashing firmware to ESP*/
                    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
                    {
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_END)
                {
                    DEBUG_PRINTLN("GAME OVER");
                    if (Update.end(true))
                    {
                        //true to set the size to the current progress
                        Serial.printf("Update Success: %u\nRebooting...\n", upload.receivedSize);
                    }
                    else
                    {
                        Update.printError(Serial);
                    }
                }
            })
};

WifiWebServer<10,SizeOfArray(pages)> webServer(pages, wifiAccess);
#endif
