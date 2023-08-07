#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "../../../components/esp_lcd/include/esp_lcd_panel_io.h"
#include "../../../components/esp_lcd/include/esp_lcd_panel_vendor.h"
#include "../../../components/esp_lcd/include/esp_lcd_panel_ops.h"
#include "sys/time.h"
#include "time.h"

#define I2C_HOST 0
#define EXAMPLE_FLASH_FREQ_MHZ 40
char strftime_buf[64];

LV_FONT_DECLARE(lv_new);

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           21 //3
#define EXAMPLE_PIN_NUM_SCL           22 //4
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C //0x78

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8 
#define EXAMPLE_LCD_PARAM_BITS         8 

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

int cnt = 0, cnt2 = 0;
struct tm timeinfo;
struct timeval now;
long int time_h=0, time_m=0, time_s=0;

#define PIN_A		4
#define PIN_B		5
#define PIN_BUTTON	19
#define GPIO_INPUT_PIN_SEL 	((1ULL<<PIN_A) | (1ULL<<PIN_BUTTON))

QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static const char *TAG = "log";


/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;

static void obtain_time(void);
static void initialize_sntp(void);

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void gpio_task_example(void* arg)
{
    int io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

        	//enc
        	if (io_num == PIN_A){
        	ESP_LOGI(TAG, "Interrupt pin %d", io_num);
            printf("ENC_A = [%d]\n", gpio_get_level(PIN_A));
            printf("ENC_B = [%d]\n", gpio_get_level(PIN_B));

            if (gpio_get_level(PIN_A) == gpio_get_level(PIN_B))
             { printf("LEFT\n"); cnt--; }
            else
             { printf("RIGHT\n"); cnt++; }
        	}
            
			//btn
        	if (io_num == PIN_BUTTON){
				//Запись в EEPROM
        		ESP_LOGI(TAG, "Interrupt pin %d", io_num);
				cnt2 = cnt;
				time_h = cnt;
				time_m = 03;
				time_s = 55;

				timeinfo.tm_year = 2022-1900;
				timeinfo.tm_mon = 5-1;
				timeinfo.tm_mday = 25;
				timeinfo.tm_hour = time_h;
				timeinfo.tm_min = time_m;
				timeinfo.tm_sec = time_s;
				timeinfo.tm_isdst = 0;
	
				now.tv_sec = mktime(&timeinfo);
				now.tv_usec = 0;
				settimeofday(&now, NULL);
        		//if (gpio_get_level(PIN_BUTTON) == 0) { cnt = 0; }
				
        	}
            printf("Count = %d, BUTTON = %d\n", cnt, gpio_get_level(PIN_BUTTON));

        }
    }
}

void Screen1(lv_obj_t *scr)
{
    
	lv_obj_t *label = lv_label_create(scr);
	//lv_obj_add_event_cb(label, my_event_cb, LV_EVENT_CLICKED, NULL);
    //lv_label_set_long_mode(label,  LV_LABEL_LONG_EXPAND);  //Circular scroll  //LV_LABEL_LONG_CLIP
    //lv_label_set_text_fmt(label, "Экран1, %d %%", cnt);
	
	time_t now;
	struct tm timeinfo;
	time(&now);
	setenv("TZ", "YEKT-5", 1);
	tzset();
	localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
	lv_label_set_text_fmt(label, "Время: %s", strftime_buf);
	//lv_label_set_recolor(label, true);
    lv_obj_set_style_text_font(label, &lv_new, 0);
	lv_obj_set_width(label, 128);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0); //lv_obj_align(label, LV_ALIGN_CENTER, 10, 10);
	lv_obj_set_pos(label, 5, cnt);
	
	//lv_obj_t *label2 = lv_label_create(scr);
	//lv_obj_set_style_text_font(label2, &lvgl_font_arial, 0);
	//lv_label_set_text_fmt(label2, "%d", cnt2);
	//lv_obj_align(label, LV_ALIGN_CENTER, 10, 10); //lv_obj_align(label, LV_ALIGN_CENTER, 10, 10);
	/*
	char* ii;
	if (cnt < 10){  ii = "One"; } else { ii = "Change text"; }
	
	lv_obj_t *label2 = lv_label_create(scr);
	//lv_obj_add_event_cb(label2, my_event_cb, LV_EVENT_CLICKED, NULL);
    lv_label_set_long_mode(label2,  LV_LABEL_LONG_CLIP);  //Circular scroll  //LV_LABEL_LONG_CLIP
    lv_label_set_text_fmt(label2, "%s", ii);
    //lv_obj_set_style_text_font(label2, &lv_new, LV_STATE_DEFAULT); //State -> 0
	lv_obj_set_width(label2, 128);
    lv_obj_align(label2, LV_ALIGN_CENTER, 0, 0); //lv_obj_align(label, LV_ALIGN_CENTER, 10, 10);
	*/
	
}

void Screen2(lv_obj_t *scr){
	
	lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text_fmt(label, "LVGL Ver:%d.%d.%d %d %%", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH, cnt);
	lv_obj_set_style_text_font(label, &lv_new, 0);
	lv_obj_set_width(label, 128);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
		

		
	//time(&now);
	//localtime_r(&now, &timeinfo);
	//printf("Time: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

	
	//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	//lv_label_set_text_fmt(label, "Время: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
	//lv_label_set_text_fmt(label, "Расчет CRC: %d", crc_cal_value(mydata, 0x06));
	
	//lv_label_set_text_fmt(label, "%d", esp_flash_get_chip_size());		
	
		/*
	lv_style_init(&styleIndicatorBar);
	lv_style_set_bg_opa(&styleIndicatorBar, LV_OPA_COVER);
	lv_style_set_bg_color(&styleIndicatorBar, lv_color_black());
	
	lv_obj_t * bar1 = lv_bar_create(scr);
	lv_obj_add_style(bar1, &styleIndicatorBar, LV_PART_INDICATOR);
    lv_obj_set_size(bar1, 100, 8);
    lv_obj_align(bar1, LV_ALIGN_CENTER, 0, 0);
    lv_bar_set_value(bar1, cnt, LV_ANIM_OFF);
	*/
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_set_px_cb(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                                   lv_color_t color, lv_opa_t opa)
{
    uint16_t byte_index = x + (( y >> 3 ) * buf_w);
    uint8_t  bit_index  = y & 0x7;

    if ((color.full == 0) && (LV_OPA_TRANSP != opa)) {
        buf[byte_index] |= (1 << bit_index);
    } else {
        buf[byte_index] &= ~(1 << bit_index);
    }
}

static void example_lvgl_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    area->y1 = area->y1 & (~0x7);
    area->y2 = area->y2 | 0x7;
}

static void example_increase_lvgl_tick(void *arg)
{
     //Tell LVGL how many milliseconds has elapsed
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool encoder_with_keys_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data){

    	data->enc_diff = 9;

      //if(enc_pressed()) data->state = LV_INDEV_STATE_PRESSED;
      //else data->state = LV_INDEV_STATE_RELEASED;

      return false;
    }

void app_main(void)
{
	
	++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    else {
        // add 500 ms error to the current system time.
        // Only to demonstrate a work of adjusting method!
        {
            ESP_LOGI(TAG, "Add a error for test adjtime");
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            int64_t cpu_time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
            int64_t error_time = cpu_time + 500 * 1000L;
            struct timeval tv_error = { .tv_sec = error_time / 1000000L, .tv_usec = error_time % 1000000L };
            settimeofday(&tv_error, NULL);
        }

        ESP_LOGI(TAG, "Time was set, now just adjusting it. Use SMOOTH SYNC method.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
#endif

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in New York is: %s", strftime_buf);

    // Set timezone to China Standard Time
    setenv("TZ", "CST-8", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf);
	
	// Set timezone Ekaterinburg
	setenv("TZ", "YEKT-5", 1);
	tzset();
	localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Екатеринбург is: %s", strftime_buf);
	
    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
        struct timeval outdelta;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
            adjtime(NULL, &outdelta);
            ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                        (long)outdelta.tv_sec,
                        outdelta.tv_usec/1000,
                        outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
	
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_NEGEDGE; //falling edge
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT; //direction input
	io_conf.pull_up_en = 1; //pull-up on
	gpio_config(&io_conf);
	
	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
	//start gpio task
	xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 1, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_A, gpio_isr_handler, (void*) PIN_A);
    gpio_isr_handler_add(PIN_BUTTON, gpio_isr_handler, (void*) PIN_BUTTON);
	
	
	static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS, // According to SSD1306 datasheet
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t)); //20
    assert(buf1);
    lv_color_t *buf2 = malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t)); //20
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.rounder_cb = example_lvgl_rounder;
    disp_drv.set_px_cb = example_lvgl_set_px_cb;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
	
	ESP_LOGI(TAG, "Register input devices driver to LVGL");
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);      
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    //indev_drv.read_cb = encoder_with_keys_read;
    //indev_drv.long_press_time = 10;
    lv_indev_drv_register(&indev_drv);



    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
	
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 100)); //2*1000 us = 2 ms

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
	
    
//10 сек сна
 while (1) {
 		lv_obj_clean(lv_scr_act()); //CLR Display
		lv_obj_t *scr = lv_disp_get_scr_act(disp);
		if (cnt < 10) { Screen1(scr); } else { Screen2(scr); }
        vTaskDelay(pdMS_TO_TICKS(100));
        lv_timer_handler();
 }
 
    //const int deep_sleep_sec = 10;
    //esp_deep_sleep(1000000LL * deep_sleep_sec);
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    /**
     * NTP server address could be aquired via DHCP,
     * see LWIP_DHCP_GET_NTP_SRV menuconfig option
     */
#ifdef LWIP_DHCP_GET_NTP_SRV
    sntp_servermode_dhcp(1);
#endif

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_ERROR_CHECK( example_disconnect() );
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}
