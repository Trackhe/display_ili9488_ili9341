static const char *TAG = "Template";

TaskHandle_t task0;
void loop0(void * pvParameters);


//choose display
#define DISPLAY_ili9341 0
#define DISPLAY_ili9488 1


//LCD & TOUCH pins & resolution & misc
#define LCD_PIN_NUM_LCD_DC     27
#define LCD_PIN_NUM_LCD_RST    2
#define LCD_PIN_NUM_LCD_CS     15
#define LCD_PIN_NUM_TOUCH_CS   4

#if DISPLAY_ili9341
#define LCD_H_RES              320
#define LCD_V_RES              240
size_t LV_BUFFER_SIZE = LCD_H_RES * 20;
#define  LVGL_TICK_PERIOD_MS   2
#elif DISPLAY_ili9488
#define LCD_H_RES              480
#define LCD_V_RES              320
size_t LV_BUFFER_SIZE = LCD_H_RES * 25;
#define  LVGL_TICK_PERIOD_MS   5
#endif

void increase_lvgl_tick(void *arg);
void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void lvgl_port_update_callback(lv_disp_drv_t *drv);
lv_disp_draw_buf_t disp_buf;
lv_disp_t *lv_display = NULL;
lv_color_t *lv_buf_1 = NULL;
lv_color_t *lv_buf_2 = NULL;

//LCD panel config
esp_lcd_panel_handle_t panel_handle = NULL;

esp_lcd_panel_dev_config_t panel_config = {
  .reset_gpio_num = LCD_PIN_NUM_LCD_RST,
  #if DISPLAY_ili9341
  .rgb_endian = LCD_RGB_ENDIAN_BGR,
  .bits_per_pixel = 16,
  #elif DISPLAY_ili9488
  .color_space = LCD_RGB_ENDIAN_BGR,
  .bits_per_pixel = 18,
  .flags = {
    .reset_active_high = 0
  },
  .vendor_config = NULL
  #endif
};


//LCD SPI
#define LCD_HOST               SPI2_HOST
#define LCD_PIN_NUM_SCLK       14
#define LCD_PIN_NUM_MOSI       12
#define LCD_PIN_NUM_MISO       13

//LCD SPI config
spi_bus_config_t buscfg = {
  .sclk_io_num = LCD_PIN_NUM_SCLK,
  .mosi_io_num = LCD_PIN_NUM_MOSI,
  .miso_io_num = LCD_PIN_NUM_MISO,
  .quadwp_io_num = GPIO_NUM_NC,
  .quadhd_io_num = GPIO_NUM_NC,
  .data4_io_num = GPIO_NUM_NC,
  .data5_io_num = GPIO_NUM_NC,
  .data6_io_num = GPIO_NUM_NC,
  .data7_io_num = GPIO_NUM_NC,
  #if DISPLAY_ili9341
  .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
  #elif DISPLAY_ili9488
  .max_transfer_sz = 32768,
  .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER,
  .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM
  #endif
};

//LCD SPI handle
esp_lcd_panel_io_handle_t io_handle = NULL;

bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
lv_disp_drv_t disp_drv; 

esp_lcd_panel_io_spi_config_t io_config = {
  .dc_gpio_num = LCD_PIN_NUM_LCD_DC,
  .cs_gpio_num = LCD_PIN_NUM_LCD_CS,
	#if DISPLAY_ili9341
  .pclk_hz = (20 * 1000 * 1000),
	#elif DISPLAY_ili9488
	.pclk_hz = (40 * 1000 * 1000),
	#endif
  .lcd_cmd_bits = 8,
  .lcd_param_bits = 8,
  .spi_mode = 0,
  .trans_queue_depth = 10,
  .on_color_trans_done = notify_lvgl_flush_ready,
  .user_ctx = &disp_drv,
  .flags = {
    .dc_low_on_data = 0,
    .octal_mode = 0,
    .sio_mode = 0,
    .lsb_first = 0,
    .cs_high_active = 0,
  },
};

//LCD SPI Touch
esp_lcd_touch_handle_t tp = NULL;
esp_lcd_panel_io_handle_t tp_io_handle = NULL;

esp_lcd_touch_config_t tp_cfg = {
    .x_max = LCD_H_RES,
    .y_max = LCD_V_RES,
    .rst_gpio_num = -1,
    .int_gpio_num = -1,
    .flags = {
        .swap_xy = 1,
        .mirror_x = 1,
        .mirror_y = 1,
    },
};

void touch_driver_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

//LCD backlight
#define LCD_PIN_NUM_BK_LIGHT   33

#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

ledc_channel_config_t LCD_backlight_channel = {
	.gpio_num = (gpio_num_t)LCD_PIN_NUM_BK_LIGHT,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.channel = LEDC_CHANNEL_0,
	.intr_type = LEDC_INTR_DISABLE,
	.timer_sel = LEDC_TIMER_1,
	.duty = 0,
	.hpoint = 0,
	.flags = {
			.output_invert = 0
	}
};

ledc_timer_config_t LCD_backlight_timer = {
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.duty_resolution = LEDC_TIMER_10_BIT,
	.timer_num = LEDC_TIMER_1,
	.freq_hz = 5000,
	.clk_cfg = LEDC_AUTO_CLK
};











