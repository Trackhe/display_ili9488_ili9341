#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "lvgl.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_ili9488.h"
#include "esp_lcd_touch_xpt2046.h"

#include "variable.h"
#include "lvgl_demo.h"

void app_main(){

	xTaskCreatePinnedToCore(loop0, "na0", configMINIMAL_STACK_SIZE + 1024 * 10, NULL, 1, &task0, 1);

}

void loop0(void * pvParameters){

	ESP_LOGI(TAG, "Setup display pins");
	gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT};
	ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

	ESP_LOGI(TAG, "Setup dusplay spi bus");
	ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

	//Backlight setup and set to 0%
	ESP_LOGI(TAG, "Initializing LEDC for backlight pin: %d", LCD_PIN_NUM_BK_LIGHT);
	ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
	ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));
	ESP_LOGI(TAG, "Setting backlight to %d%%", 0);
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (1023 * 0) / 100));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

	ESP_LOGI(TAG, "Initializing display SPI");
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));


	ESP_LOGI(TAG, "Initialize touch controller XPT2046 SPI");
  esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(LCD_PIN_NUM_TOUCH_CS);
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &tp_io_config, &tp_io_handle));
	ESP_LOGI(TAG, "Initialize touch controller XPT2046 handle");
  ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));

	#if DISPLAY_ili9341
	ESP_LOGI(TAG, "Install ILI9341 panel driver");
	ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
	#elif DISPLAY_ili9488
	ESP_LOGI(TAG, "Install ILI9488 panel driver");
	ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(io_handle, &panel_config, LV_BUFFER_SIZE, &panel_handle));
	#endif

	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
	ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
  ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

	
	ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();

  ESP_LOGI(TAG, "Allocating %zu bytes for LVGL buffer", LV_BUFFER_SIZE * sizeof(lv_color_t));
  lv_buf_1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
  ESP_LOGI(TAG, "Allocating %zu bytes for second LVGL buffer", LV_BUFFER_SIZE * sizeof(lv_color_t));
  lv_buf_2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);

  ESP_LOGI(TAG, "Creating LVLG display buffer");
  lv_disp_draw_buf_init(&disp_buf, lv_buf_1, lv_buf_2, LV_BUFFER_SIZE);

  ESP_LOGI(TAG, "Initializing %dx%d display", LCD_H_RES, LCD_V_RES);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
	disp_drv.flush_cb = lvgl_flush_cb;
	disp_drv.drv_update_cb = lvgl_port_update_callback;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  lv_display = lv_disp_drv_register(&disp_drv);

  ESP_LOGI(TAG, "Creating LVGL tick timer");
	const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));


	//Touch with LVGL
	ESP_LOGI(TAG, "Initializing LVGL touch driver");
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_driver_read;
  lv_indev_drv_register( &indev_drv );

	start_ui(lv_display);

	for(;;){
		vTaskDelay(1);
		lv_timer_handler();
	}
	vTaskDelete(NULL);
}


bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx){
	lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
	lv_disp_flush_ready(disp_driver);
	return false;
}

void increase_lvgl_tick(void *arg){
	lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map){
	esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
	int offsetx1 = area->x1;
	int offsetx2 = area->x2;
	int offsety1 = area->y1;
	int offsety2 = area->y2;
	// copy a buffer's content to a specific area of the display
	esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

void lvgl_port_update_callback(lv_disp_drv_t *drv){
  panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

	//esp_lcd_panel_swap_xy(panel_handle, true);
	//esp_lcd_panel_mirror(panel_handle, true, true);
}

void touch_driver_read(lv_indev_drv_t *drv, lv_indev_data_t *data){
    ESP_LOGI(TAG, "Read Touch Data");
    uint16_t x[1];
    uint16_t y[1];
    uint16_t strength[1];
    uint8_t count = 0;

    // Update touch point data.
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(tp));

    data->state = LV_INDEV_STATE_REL;

    if (esp_lcd_touch_get_coordinates(tp, x, y, strength, &count, 1))
    {
        ESP_LOGI(TAG, "Touch Data %hu, %hu, %hu", *x, *y, *strength);
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PR;
    }

    data->continue_reading = false;
}