
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

protected:
    lv_obj_t*  fView = nullptr;
    lv_obj_t*  fTextPos = nullptr;
    const lv_font_t* fTextPosFont = nullptr;
    uint32_t   fLastScreenUpdate = 0;
    int16_t    fLastPos = -1;
    int16_t    fLastDisplayPos = -1;

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
                }
                fLastScreenUpdate = millis();
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
