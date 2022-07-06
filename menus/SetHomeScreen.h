///////////////////////////////////////////////////////////////////////////////
//
// Specify dome home position. Specified angle becomes the new 0 position.
// 
///////////////////////////////////////////////////////////////////////////////

class SetHomeScreen : public RotateDomeScreen
{
public:
    SetHomeScreen(ScreenID id = kSetHomeScreen) :
        RotateDomeScreen(id)
    {}

    virtual void buttonInReleased() override
    {
        uint16_t homePosition = 160;//sDomePosition.getDomePosition();
        if (sSettings.fHomePosition != homePosition)
        {
            sSettings.fHomePosition = homePosition;
            restoreDomeSettings();
            sSettings.write();
        }
        popScreen();
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SetHomeScreen sHomeScreen;