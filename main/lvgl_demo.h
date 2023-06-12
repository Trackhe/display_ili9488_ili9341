void setstyle();
void lv_example_menu(lv_disp_t *disp);
void button_event(lv_obj_t * obj, lv_event_t event);

void start_ui(lv_disp_t *disp){
  lv_disp_set_rotation(disp, 0);

  setstyle();
  lv_example_menu(disp);
}

void setstyle(){
  lv_style_t style;
  lv_style_init(&style);
  lv_style_set_bg_color(&style, lv_color_black());
  lv_style_set_border_width(&style, 0);
  lv_style_set_radius(&style, 0);
}

void lv_example_menu(lv_disp_t *disp){
  lv_obj_t *scr = lv_disp_get_scr_act(disp);
  lv_obj_clean(lv_scr_act()); // Clear the Display

  lv_obj_t * button_component = lv_btn_create(lv_scr_act()); // create the container for the states
  lv_obj_set_scrollbar_mode(button_component, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_height(button_component, 0.3 * LCD_V_RES); // set height of the obj
  lv_obj_set_width(button_component, 0.5 * LCD_H_RES); // set width of the obj
  lv_obj_align(button_component, LV_ALIGN_CENTER, 0.0 * LCD_H_RES, 0.0 * LCD_V_RES);
  lv_obj_t * button_component_text = lv_label_create(button_component); // create a label object
  lv_label_set_text(button_component_text, "Button"); // set the text of the object
  lv_obj_align(button_component_text, LV_ALIGN_CENTER, 0, 0); // aligen the objekt inside the container
  lv_obj_set_style_bg_color(button_component, lv_color_hex(0x005aff), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_color(button_component, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_add_event_cb(button_component, button_event, LV_EVENT_RELEASED, NULL);
}

void button_event(lv_obj_t * obj, lv_event_t event){
  ESP_LOGI(TAG, "Hello World!");
}
