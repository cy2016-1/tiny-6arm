#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/timer.h"
#include "esp32/rom/ets_sys.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/twai.h"
#define DMA_CHAN        2
#define PIN_NUM_MISO    10
#define PIN_NUM_MOSI    3
#define PIN_NUM_CLK     1
#define PIN_NUM_CS      4

#define STORAGE_NAMESPACE "storage"

#define TIMER_DIVIDER         (2)  //  Hardware timer clock divider
#define TIMER_SCALE           ((6*10)/(TIMER_DIVIDER))  // convert counter value to seconds



#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (2) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (100) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (20000) // Frequency in Hertz. Set frequency at 5 kHz

#define STEP_EN  5
#define STEP_PUL 6
#define STEP_DIR 7

#define LED_OI   0
#define KEY_OI   9

#define CAN_TX   18
#define CAN_RX   19

static const char TAG[] = "main";


typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;


esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}





static void pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
int first_angle = 0;
long step_pul = 0;
long tar_pul = 0;
long step_angle = 0;
long pwm = 0;
long max_pwm = 0;
int num=0;
int key = 20;
int circle = 0;
int step_circle = 0;
int pre_angle = 0;
int cur_angle = 0;
int8_t up_foc = 100;
int8_t down_foc = 100;
uint8_t min_pwm = 0;
uint16_t Encoder_calibration[3200];
int is_calibration = 0;
int calibration_num = 0;
int is_save_calibration = 0;

spi_transaction_t t = {
	.length = 16,
	.addr = 0x83,
	.flags = SPI_TRANS_USE_RXDATA,
	.user = (void*)1,
};
esp_err_t ret;
spi_device_handle_t spi;


static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NO_ACK);



static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    gpio_set_level(STEP_PUL, 0);

    num ++;
    if((num%2)==0)
    {
    	pre_angle = cur_angle;

        gpio_set_level(PIN_NUM_CS, 0);
    	ret = spi_device_polling_transmit(spi, &t);
    	gpio_set_level(PIN_NUM_CS, 1);

    	cur_angle = (t.rx_data[0]*64+t.rx_data[1]/4);
    	if(cur_angle<1000&&pre_angle>15000)circle++;
    	if(cur_angle>15000&&pre_angle<1000)circle--;
    	step_angle = circle*16384+cur_angle;
    	pwm = labs(step_angle-(step_circle*16384+Encoder_calibration[(step_pul%3200)<0?(step_pul%3200+3200):(step_pul%3200)]));
    	pwm = (pwm*pwm*6);
    	if(pwm>60000)pwm = 60000;
    	if(pwm<2)pwm = 2;
    	if(max_pwm<pwm)max_pwm = pwm;
    	else max_pwm = max_pwm - 2;
    	if(max_pwm<2)max_pwm = 2;
        if(is_calibration==1)ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 300);
    	else ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (max_pwm/100)+min_pwm*10);
    	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }


    if(is_calibration==1)
    {
    	if((num%1000)==0)
    	{
    		Encoder_calibration[(step_pul+calibration_num)%3200]=cur_angle;
			gpio_set_level(STEP_DIR, 1);
			gpio_set_level(STEP_PUL, 1);
    		calibration_num++;
    		if(calibration_num>=3200)
			{
    			is_calibration = 0;
    			calibration_num = 0;
    			is_save_calibration = 1;
			}
    	}
    }
    else if(is_calibration==0)
	{
		if((step_angle-(step_circle*16384+Encoder_calibration[(step_pul%3200)<0?(step_pul%3200+3200):(step_pul%3200)]))>up_foc)
		{
			gpio_set_level(STEP_DIR, 1);
			gpio_set_level(STEP_PUL, 1);
			step_pul+=1;
			if(Encoder_calibration[((step_pul-1)%3200)<0?((step_pul-1)%3200+3200):((step_pul-1)%3200)]-Encoder_calibration[((step_pul)%3200)<0?((step_pul)%3200+3200):((step_pul)%3200)]>1000)step_circle++;
		}

		else if((step_angle-(step_circle*16384+Encoder_calibration[(step_pul%3200)<0?(step_pul%3200+3200):(step_pul%3200)]))<-down_foc)
		{
			gpio_set_level(STEP_DIR, 0);
			gpio_set_level(STEP_PUL, 1);
			step_pul-=1;
			if(Encoder_calibration[((step_pul+1)%3200)<0?((step_pul+1)%3200+3200):((step_pul+1)%3200)]-Encoder_calibration[((step_pul)%3200)<0?((step_pul)%3200+3200):((step_pul)%3200)]<-1000)step_circle--;
		}

		else if((tar_pul-step_pul)>1&&(num%(1000/((tar_pul-step_pul))+1)==0))
		{
			gpio_set_level(STEP_DIR, 1);
			gpio_set_level(STEP_PUL, 1);
			step_pul+=1;
			if(Encoder_calibration[((step_pul-1)%3200)<0?((step_pul-1)%3200+3200):((step_pul-1)%3200)]-Encoder_calibration[((step_pul)%3200)<0?((step_pul)%3200+3200):((step_pul)%3200)]>1000)step_circle++;

		}
		else if((tar_pul-step_pul)<-1&&(num%(1000/((-tar_pul+step_pul))+1)==0))
		{
			gpio_set_level(STEP_DIR, 0);
			gpio_set_level(STEP_PUL, 1);
			step_pul-=1;
			if(Encoder_calibration[((step_pul+1)%3200)<0?((step_pul+1)%3200+3200):((step_pul+1)%3200)]-Encoder_calibration[((step_pul)%3200)<0?((step_pul)%3200+3200):((step_pul)%3200)]<-1000)step_circle--;

		}


	}

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR

}

static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(group, timer);

    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);
}

static void twai_receive_task(void *arg)
{
    while (1) {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY);
        if (rx_msg.identifier == 0x0B3) 
        {   
            int32_t data = 0;
            for (int i = 0; i < 4; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            }
            tar_pul = data;
            gpio_set_level(LED_OI, 1);
        }
        else if (rx_msg.identifier == 0x0C5)
        {
            up_foc = rx_msg.data[0];
            down_foc = rx_msg.data[1];
            min_pwm = rx_msg.data[2];
            if(rx_msg.data[3]==1)tar_pul = step_pul;
            gpio_set_level(LED_OI, 1);
        }
    }
    vTaskDelete(NULL);
}



static void run_task(void *arg)
{
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    while (1) {
        twai_message_t tx_msg = {.data_length_code = 8, .identifier = 0x0A5};
        tx_msg.data[0] =  step_pul & 0x000000ff; 
        tx_msg.data[1] = (step_pul & 0x0000ff00) >> 8; 
        tx_msg.data[2] = (step_pul & 0x00ff0000) >> 16;
        tx_msg.data[3] = (step_pul & 0xff000000) >> 24;
        tx_msg.data[4] = up_foc;
        tx_msg.data[5] = down_foc;
        tx_msg.data[6] = max_pwm/1000;  
        tx_msg.data[7] = min_pwm;                
        twai_transmit(&tx_msg, portMAX_DELAY);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
//    tcpip_adapter_init();
//    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
//    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
//    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
//    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
//    wifi_config_t sta_config = {
//        .sta = {
//            .ssid = CONFIG_ESP_WIFI_SSID,
//            .password = CONFIG_ESP_WIFI_PASSWORD,
//            .bssid_set = false
//        }
//    };
//    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
//    ESP_ERROR_CHECK( esp_wifi_start() );
//    ESP_ERROR_CHECK( esp_wifi_connect() );
    nvs_handle_t my_handle;
    size_t required_size = 6400;  // value will default to 0, if not set yet in NVS
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) ESP_LOGI(TAG, "Initializing nvs err");

    err = nvs_get_blob(my_handle, "Encoder", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) ESP_LOGI(TAG, "get nvs num err");
    ESP_LOGI(TAG, "nvs num :%d",required_size);

    err = nvs_get_blob(my_handle, "Encoder", Encoder_calibration, &required_size);
    if (err != ESP_OK) ESP_LOGI(TAG, "get nvs err");
//	for(int i=0;i<3200;i++)
//	{
//		ESP_LOGI(TAG, "Encoder_calibration[%d]=%d",i,Encoder_calibration[i]);
//	}

    // Set the LEDC peripheral configuration
    pwm_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 50));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));




    ESP_LOGI(TAG, "Initializing bus SPI%d...", SPI2_HOST+1);

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,                // MISO信号线
        .mosi_io_num = PIN_NUM_MOSI,                // MOSI信号线
        .sclk_io_num = PIN_NUM_CLK,                 // SCLK信号线
        .quadwp_io_num = -1,                        // WP信号线，专用于QSPI的D2
        .quadhd_io_num = -1,                        // HD信号线，专用于QSPI的D3
        .max_transfer_sz = 32,                    // 最大传输数据大小
    };

    spi_device_interface_config_t devcfg={
//    	.command_bits = 16,
		.address_bits = 8,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,      // Clock out at 10 MHz,
//        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
//        .pre_cb = cs_high,
//        .post_cb = cs_low,
		.input_delay_ns =200,
		.mode = 3,                                  // SPI mode 0
        /*
         * The timing requirements to read the busy signal from the EEPROM cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        .spics_io_num = -1,
        .queue_size = 2,                            // 传输队列大小，决定了等待传输数据的数量
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, 3);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);





//    gpio_pad_select_gpio(PIN_NUM_CS);                // 选择一个GPIO
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask =
    		((1ULL<<PIN_NUM_CS)|(1ULL<<STEP_EN)|(1ULL<<STEP_PUL)|(1ULL<<STEP_DIR)|
            (1ULL<<CAN_TX) |(1ULL<<LED_OI));
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


    gpio_config_t i_conf = {};
    //disable interrupt
    i_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    i_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    i_conf.pin_bit_mask =
    		(1ULL<<CAN_RX);
    //disable pull-down mode
    i_conf.pull_down_en = 1;
    //disable pull-up mode
    i_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&i_conf);

    i_conf.pin_bit_mask =
    		((1ULL<<KEY_OI));
    //disable pull-down mode
    i_conf.pull_down_en = 0;
    //disable pull-up mode
    i_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&i_conf);


    gpio_set_level(CAN_TX, 1);

	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 400);
    gpio_set_level(STEP_EN, 1);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
	vTaskDelay(100 / portTICK_PERIOD_MS);

    gpio_set_level(PIN_NUM_CS, 1);
	vTaskDelay(10 / portTICK_PERIOD_MS);

    gpio_set_level(PIN_NUM_CS, 0);
	ret = spi_device_polling_transmit(spi, &t);
	gpio_set_level(PIN_NUM_CS, 1);
    gpio_set_level(PIN_NUM_CS, 0);
	ret = spi_device_polling_transmit(spi, &t);
	gpio_set_level(PIN_NUM_CS, 1);
	cur_angle = (t.rx_data[0]*64+t.rx_data[1]/4);
	pre_angle = cur_angle;

	// step_pul = (cur_angle/64)*64;
	// tar_pul = step_pul;
    gpio_set_level(LED_OI, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_OI, 0);
    if(gpio_get_level(KEY_OI)==0){
        // key--;
        // gpio_set_level(CAN_TX, key%2);
        is_calibration = 1;

    }else
    {
        ESP_LOGI(TAG, "cur_angle: %d", cur_angle);

        int min_angle = abs(cur_angle-Encoder_calibration[0]);
        ESP_LOGI(TAG, "min_angle: %d", min_angle);

        for(int i=0;i<3200;i+=64)
        {
            if(abs(cur_angle-Encoder_calibration[i]) < min_angle){
                min_angle = abs(cur_angle-Encoder_calibration[i]);
                ESP_LOGI(TAG, "min_angle: %d", min_angle);
                ESP_LOGI(TAG, "i: %d", i);
                step_pul = i;
                tar_pul = step_pul;
            } 

        }
        ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(TAG, "Driver started");
        xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, tskNO_AFFINITY);
        xTaskCreatePinnedToCore(run_task, "run_task", 4096, NULL, 7, NULL, tskNO_AFFINITY);

    }
   

    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 10);

    int level = 0;
    while (true) {
        // gpio_set_level(CAN_TX, 0);

    	if(is_save_calibration)
    	{
    	    err = nvs_set_blob(my_handle, "Encoder", Encoder_calibration, 6400);
    	    if (err != ESP_OK) ESP_LOGI(TAG, "set nvs err");
    	    // Commit
    	    err = nvs_commit(my_handle);
    	    if (err != ESP_OK) ESP_LOGI(TAG, "nvs commit err");

    	    // Close
    	    nvs_close(my_handle);
			for(int i=0;i<3200;i++)
			{
				ESP_LOGI(TAG, "Encoder_calibration[%d]=%d",i,Encoder_calibration[i]);
			}
			is_save_calibration = 0;
    	}
        gpio_set_level(LED_OI, 0);
        // ESP_LOGI(TAG, "tar_pul: %ld", tar_pul);
        // ESP_LOGI(TAG, "cur_angle: %d", cur_angle);
        // ESP_LOGI(TAG, "step_angle: %ld", step_angle);
        // ESP_LOGI(TAG, "step_pul: %ld", step_pul);
        // ESP_LOGI(TAG, "step: %d", cur_angle/64);
        // ESP_LOGI(TAG, "max_pwm: %ld", max_pwm);
        // ESP_LOGI(TAG, "Encoder_calibration[%d]: %d",step_pul%3200<0?(step_pul%3200+3200):(step_pul%3200), (step_circle*16384+Encoder_calibration[((step_pul%3200)<0)?((step_pul%3200)+3200):(step_pul%3200)]));
        level = !level;            

        vTaskDelay(200 / portTICK_PERIOD_MS);


    }
}

