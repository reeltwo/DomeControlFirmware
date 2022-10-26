
#define WHITE 0xFFFF
#define UI_BG_COLOR    lv_color_black()
#define UI_FRAME_COLOR lv_color_hex(0x282828)
#define UI_FONT_COLOR  lv_color_white()

class StatusDisplayLVGL
{
public:
    StatusDisplayLVGL() { }

    void refresh()
    {
        ensureConstructed();

        updatePositionDisplay();

        lv_timer_handler();
    }

    void wake()
    {
        if (fSleeping)
        {
            // Turn on LCD and backlight
            digitalWrite(PIN_POWER_ON, HIGH);
            digitalWrite(PIN_LCD_BL, HIGH);
            fSleeping = false;
        }
    }

    void sleep()
    {
        if (!fSleeping)
        {
            // Turn off LCD and backlight
            digitalWrite(PIN_POWER_ON, LOW);
            digitalWrite(PIN_LCD_BL, LOW);
            fSleeping = true;
        }
    }

    void showStatus(const char* msg, void (*callback)() = nullptr)
    {
        printf("Show status: %s\n", msg);
        lv_label_set_text(fCenterStatus, msg);
        if (callback != nullptr)
        {
            fStatusTimeout = millis() + STATUS_SCREEN_DURATION;
            fCallback = callback;
        }
    }

protected:
    lv_obj_t*  fView = nullptr;
    lv_obj_t*  fTextPos = nullptr;
    const lv_font_t* fTextPosFont = nullptr;
    uint32_t   fLastScreenUpdate = 0;
    int16_t    fLastPos = -1;
    int16_t    fLastDisplayPos = -1;
    bool       fSleeping = false;
    lv_obj_t*  fCenterStatus = nullptr;
    lv_obj_t*  fTopLeftStatus = nullptr;
    lv_obj_t*  fTopRightStatus = nullptr;
    lv_obj_t*  fBottomRightStatus = nullptr;
    void       (*fCallback)() = nullptr;
    uint32_t   fStatusTimeout = 0;

    void updatePositionDisplay()
    {
        if (sDomePosition.ready())
        {
            if (millis() > fLastScreenUpdate + 100)
            {
                int16_t pos = sDomePosition.getHomeRelativeDomePosition();
                if (sDomePosition.ready() && pos != fLastPos)
                    fLastPos = pos;
                if (fLastPos != fLastDisplayPos)
                {
                    char buffer[10];
                    snprintf(buffer, sizeof(buffer), "%d", fLastPos);
                    setTextPosFont(font_large);
                    lv_label_set_text(fTextPos, buffer);
                    fLastDisplayPos = fLastPos;
                    fLastScreenUpdate = millis();
                    wake();
                }
            }
        }
        else
        {
            if (fLastDisplayPos != -2)
            {
                /* Position not available */
                setTextPosFont(font_normal);
                lv_label_set_text(fTextPos, "Position\nNot Available");
                fLastDisplayPos = -2;
            }
            fLastScreenUpdate = millis();
        }
        if (fStatusTimeout != 0)
            fLastScreenUpdate = millis();
        // No change in position put screen to sleep
        if (!fSleeping && fLastScreenUpdate + SCREEN_SLEEP_TIMER < millis())
        {
            sleep();
        }
        lv_label_set_text(fTopRightStatus, wifiEnabled ? LV_SYMBOL_WIFI : "");
        if (fStatusTimeout != 0 && fStatusTimeout < millis())
        {
            lv_label_set_text(fCenterStatus, "");
            if (fCallback)
                fCallback();
            fStatusTimeout = 0;
        }
    }

    void ensureConstructed()
    {
        if (fView != nullptr)
            return;
        fView = lv_tileview_create(lv_scr_act());
        lv_obj_align(fView, LV_ALIGN_TOP_RIGHT, 0, 0);
        lv_obj_set_size(fView, LV_PCT(100), LV_PCT(100));

        lv_obj_t *pages = lv_tileview_add_tile(fView, 0, 0, LV_DIR_HOR);

        // Dome position view
        {
            lv_obj_t *main_cout = lv_obj_create(pages);
            lv_obj_set_size(main_cout, LV_PCT(100), LV_PCT(100));
            lv_obj_clear_flag(main_cout, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_style_border_width(main_cout, 0, 0);
            lv_obj_set_style_bg_color(main_cout, UI_BG_COLOR, 0);

            lv_obj_t *menu_cout = lv_obj_create(main_cout);
            lv_obj_set_size(menu_cout, LV_PCT(100), LV_PCT(100));
            lv_obj_align(menu_cout, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_style_bg_color(menu_cout, UI_FRAME_COLOR, 0);
            lv_obj_clear_flag(menu_cout, LV_OBJ_FLAG_SCROLLABLE);

            fTextPos = lv_label_create(menu_cout);
            lv_obj_center(fTextPos);
            lv_label_set_text(fTextPos, "");
            lv_obj_set_style_text_color(fTextPos, UI_FONT_COLOR, 0);
            lv_obj_set_style_text_font(fTextPos, font_large, 0);
        }
        fBottomRightStatus = lv_label_create(lv_scr_act());
        lv_obj_align(fBottomRightStatus, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
        lv_label_set_text(fBottomRightStatus, "");

        fTopRightStatus = lv_label_create(lv_scr_act());
        lv_obj_align(fTopRightStatus, LV_ALIGN_TOP_RIGHT, 0, 0);
        lv_label_set_text(fTopRightStatus, "");

        fTopLeftStatus = lv_label_create(lv_scr_act());
        lv_obj_align(fTopLeftStatus, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_label_set_text(fTopLeftStatus, "");

        fCenterStatus = lv_label_create(lv_scr_act());
        lv_obj_align(fCenterStatus, LV_ALIGN_CENTER, 0, 0);
        lv_label_set_text(fCenterStatus, "");
        lv_obj_set_style_text_color(fCenterStatus, UI_BG_COLOR, 0);
        lv_obj_set_style_bg_opa(fCenterStatus, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(fCenterStatus, UI_FONT_COLOR, 0);
        lv_obj_set_style_text_font(fCenterStatus, font_medium, 0);
    }

    void setTextPosFont(const lv_font_t* font)
    {
        // Cache the set font so we dont refresh the display needlessly
        if (fTextPosFont != font)
        {
            lv_obj_set_style_text_font(fTextPos, font, 0);
            fTextPosFont = font;
        }
    }
};
