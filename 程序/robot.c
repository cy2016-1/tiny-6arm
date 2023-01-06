#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "driver/timer.h"
#include "math.h"
#include "freertos/task.h"
#include <esp_http_server.h>
#include "cJSON.h"
//#include "extmod/vfs.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_littlefs.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "esp_log.h"
#define TIMER_DIVIDER         (2)  //  Hardware timer clock divider
#define TIMER_SCALE           ((8*10)/(TIMER_DIVIDER))  // convert counter value to seconds


// #define TX_485      GPIO_NUM_8
// #define CTRL_485    GPIO_NUM_19
// #define RX_485      GPIO_NUM_20

#define ROBOT_X1    GPIO_NUM_3
#define ROBOT_X2    GPIO_NUM_46
#define ROBOT_X3    GPIO_NUM_9
#define ROBOT_X4    GPIO_NUM_10
#define ROBOT_X5    GPIO_NUM_11
#define ROBOT_X6    GPIO_NUM_12


#define ROBOT_Y1    GPIO_NUM_45
#define ROBOT_Y2    GPIO_NUM_48
#define ROBOT_Y3    GPIO_NUM_47
#define ROBOT_Y4    GPIO_NUM_21
#define ROBOT_Y5    GPIO_NUM_14
#define ROBOT_Y6    GPIO_NUM_13

#define ROBOT_LED1    GPIO_NUM_35
#define ROBOT_LED2    GPIO_NUM_18

#define CAN_TX_GPIO    GPIO_NUM_1
#define CAN_RX_GPIO    GPIO_NUM_2


// #define TAG "RS485_ECHO_APP"
#define ECHO_TEST_TXD   (8)
#define ECHO_TEST_RXD   (20)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (17)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS   (UART_PIN_NO_CHANGE)
#define BUF_SIZE        (127)

#define ECHO_UART_PORT          (UART_NUM_2)
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

#define BYTE0(dwTemp)           (*(char *)(&dwTemp))
#define BYTE1(dwTemp)           (*(char *)(&dwTemp)+1)
#define BYTE2(dwTemp)           (*(char *)(&dwTemp)+2)
#define BYTE3(dwTemp)           (*(char *)(&dwTemp)+3)

esp_vfs_spiffs_conf_t file_conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true
};

esp_vfs_littlefs_conf_t littlefs_conf = {
    .base_path = "/littlefs",
    .partition_label = "littlefs",
    .format_if_mount_failed = true,
    .dont_mount = false,
};

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


static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NO_ACK);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}


unsigned long mysqrt(unsigned long x)
{
 register unsigned long xr;  // result register
 register unsigned long q2;  // scan-bit register
 register unsigned char f;   // flag (one bit)

 xr = 0;                     // clear result
 q2 = 0x40000000L;           // higest possible result bit
 do
 {
   if((xr + q2) <= x)
   {
     x -= xr + q2;
     f = 1;                  // set flag
   }
   else{
     f = 0;                  // clear flag
   }
   xr >>= 1;
   if(f){
     xr += q2;               // test flag
   }
 } while(q2 >>= 2);          // shift twice
 if(xr < x){
   return xr +1;             // add for rounding
 }
 else{
   return xr;
 }
}

//电机1参数
long tar_pulse1=0;
long cur_pulse1=0;
long pre_pulse1=0;
long pulse1_scale=0;
int8_t step1_up_foc = 100;
int8_t step1_down_foc = 100;

//电机2参数
long tar_pulse2=0;
long cur_pulse2=0;
long pre_pulse2=0;
long pulse2_scale=0;
int8_t step2_up_foc = 100;
int8_t step2_down_foc = 100;

//电机3参数
long tar_pulse3=0;
long cur_pulse3=0;
long pre_pulse3=0;
long pulse3_scale=0;
int8_t step3_up_foc = 100;
int8_t step3_down_foc = 100;

//电机4参数
long tar_pulse4=0;
long cur_pulse4=0;
long pre_pulse4=0;
long pulse4_scale=0;
int8_t step4_up_foc = 100;
int8_t step4_down_foc = 100;

//电机5参数
long tar_pulse5=0;
long cur_pulse5=0;
long pre_pulse5=0;
long pulse5_scale=0;
int8_t step5_up_foc = 100;
int8_t step5_down_foc = 100;

//电机6参数
long tar_pulse6=0;
long cur_pulse6=0;
long pre_pulse6=0;
long pulse6_scale=0;
int8_t step6_up_foc = 100;
int8_t step6_down_foc = 100;

//机械臂参数
long robot_tar_pulse = 0;
long robot_cur_pulse = 0;
long robot_global_speed = 50;
long robot_tar_speed = 100;
long robot_cur_speed = 0;
long robot_tar_acc = 100;
long robot_cur_acc = 0;
unsigned long robot_key= 0; //1-12 bit : control , 13- bit : continue or pause  , 17-32bit : IO 
long robot_model= 0;//  0:lock , 1:control , 2:joint move , 3:line move ,  3:arc move 
long comtrol_time_out = 500; //500ms
int time_is_run = 0;
int is_send_twai = 0;
int robot_foc[6];
// char file_string[20000];

int is_step4_run = 0;

long system_val[200];

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

static QueueHandle_t tx_task_queue;
typedef enum {
    TX_SEND_PINGS,
    TX_SEND_START_CMD,
    TX_SEND_STOP_CMD,
    TX_TASK_EXIT,
    TX_SEND_DATA1,
    TX_SEND_DATA2,
    TX_SEND_DATA3,
} tx_task_action_t;

long max(long a, long b){
	if(a>b)return a;
	else return b;
}


void robot_change_pulse(){
	robot_tar_pulse = max(labs(tar_pulse1-cur_pulse1),labs(tar_pulse2-cur_pulse2));
	robot_tar_pulse = max(robot_tar_pulse,labs(tar_pulse3-cur_pulse3));
	robot_tar_pulse = max(robot_tar_pulse,labs(tar_pulse4-cur_pulse4));
	robot_tar_pulse = max(robot_tar_pulse,labs(tar_pulse5-cur_pulse5));
	robot_tar_pulse = max(robot_tar_pulse,labs(tar_pulse6-cur_pulse6));
	if(robot_tar_pulse!=0)
	{
		pulse1_scale=1000*labs(tar_pulse1-cur_pulse1)/robot_tar_pulse;
		pulse2_scale=1000*labs(tar_pulse2-cur_pulse2)/robot_tar_pulse;
		pulse3_scale=1000*labs(tar_pulse3-cur_pulse3)/robot_tar_pulse;
		pulse4_scale=1000*labs(tar_pulse4-cur_pulse4)/robot_tar_pulse;
		pulse5_scale=1000*labs(tar_pulse5-cur_pulse5)/robot_tar_pulse;
		pulse6_scale=1000*labs(tar_pulse6-cur_pulse6)/robot_tar_pulse;
	}
	robot_tar_pulse  = robot_tar_pulse*1000;
	robot_cur_pulse = 0;

}


int level2 = 0;
long num=0;
long L1=10,L2=30,L3=20;



int num1=0;

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    num++;
    
    if(num%100==0)
    {
   	comtrol_time_out = comtrol_time_out -1;
    	if(comtrol_time_out<=0)
    	{
    	    comtrol_time_out = 0;
    	    robot_key = robot_key&0xfffff000;
    	}
    	if(gpio_get_level(ROBOT_X1)) robot_key = robot_key|0x00010000;
    	else robot_key = robot_key&0xfffeffff;
     	if(gpio_get_level(ROBOT_X2)) robot_key = robot_key|0x00020000;
    	else robot_key = robot_key&0xfffdffff;
    	if(gpio_get_level(ROBOT_X3)) robot_key = robot_key|0x00040000;
    	else robot_key = robot_key&0xfffbffff;
    	if(gpio_get_level(ROBOT_X4)) robot_key = robot_key|0x00080000;
    	else robot_key = robot_key&0xfff7ffff;
    	if(gpio_get_level(ROBOT_X5)) robot_key = robot_key|0x00100000;
    	else robot_key = robot_key&0xffefffff;
     	if(gpio_get_level(ROBOT_X6)) robot_key = robot_key|0x00200000;
    	else robot_key = robot_key&0xffdfffff;
    	// if(gpio_get_level(ROBOT_X7)) robot_key = robot_key|0x00400000;
    	// else robot_key = robot_key&0xffbfffff;
    	// if(gpio_get_level(ROBOT_X8)) robot_key = robot_key|0x00800000;
    	// else robot_key = robot_key&0xff7fffff;

    	   	
    }    
    
    if(robot_model==1)
    {

    	if((robot_key&0x00000001)&&(num%(2000/(robot_global_speed+1))==0))
    	{  
    	    cur_pulse1 = cur_pulse1+1;
    	    
    	}
    	if((robot_key&0x00000002)&&(num%(2000/(robot_global_speed+1))==0))
    	{ 	 
    	    cur_pulse1 = cur_pulse1-1;
    	    
    	}
    	
    	if((robot_key&0x00000004)&&(num%(2000/(robot_global_speed+1))==0))
    	{

    	    cur_pulse2 = cur_pulse2+1;
    	    
    	}
    	if((robot_key&0x00000008)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    cur_pulse2 = cur_pulse2-1;
    	    
    	}    	
    	
  	    if((robot_key&0x00000010)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    cur_pulse3 = cur_pulse3+1;
    	    
    	}
    	if((robot_key&0x00000020)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    cur_pulse3 = cur_pulse3-1;
    	    
    	}
    	if((robot_key&0x00000040)&&(num%(2000/(robot_global_speed+1))==0))
    	{

    	    cur_pulse4 = cur_pulse4+1;

    	}
    	if((robot_key&0x00000080)&&(num%(2000/(robot_global_speed+1))==0))
    	{

    	    cur_pulse4 = cur_pulse4-1;

    	    
    	}      	
  	    if((robot_key&0x00000100)&&(num%(2000/(robot_global_speed+1))==0))
    	{

    	    cur_pulse5 = cur_pulse5-1;
            cur_pulse6 = cur_pulse6+1;

    	    
    	}
    	if((robot_key&0x00000200)&&(num%(2000/(robot_global_speed+1))==0))
    	{

    	    cur_pulse5 = cur_pulse5+1;
            cur_pulse6 = cur_pulse6-1;

    	}
    	if((robot_key&0x00000400)&&(num%(2000/(robot_global_speed+1))==0))
    	{

    	    cur_pulse6 = cur_pulse6+1;
            cur_pulse5 = cur_pulse5+1;

    	    
    	}
    	if((robot_key&0x00000800)&&(num%(2000/(robot_global_speed+1))==0))
    	{

    	    cur_pulse6 = cur_pulse6-1;
            cur_pulse5 = cur_pulse5-1;

    	}
    	
    	
    	gpio_set_level(ROBOT_Y1, (robot_key>>24)&0x00000001);    
    	gpio_set_level(ROBOT_Y2, (robot_key>>25)&0x00000001);
    	gpio_set_level(ROBOT_Y3, (robot_key>>26)&0x00000001);
    	gpio_set_level(ROBOT_Y4, (robot_key>>27)&0x00000001);
    	gpio_set_level(ROBOT_Y5, (robot_key>>28)&0x00000001);
    	gpio_set_level(ROBOT_Y6, (robot_key>>29)&0x00000001);
    	// gpio_set_level(ROBOT_Y7, (robot_key>>30)&0x00000001);
    	// gpio_set_level(ROBOT_Y8, (robot_key>>31)&0x00000001);	    	

   
    }else
    {
    	if(gpio_get_level(ROBOT_Y1)) robot_key = robot_key|0x01000000;
    	else robot_key = robot_key&0xfeffffff;
     	if(gpio_get_level(ROBOT_Y2)) robot_key = robot_key|0x02000000;
    	else robot_key = robot_key&0xfdffffff;
    	if(gpio_get_level(ROBOT_Y3)) robot_key = robot_key|0x04000000;
    	else robot_key = robot_key&0xfbffffff;
    	if(gpio_get_level(ROBOT_Y4)) robot_key = robot_key|0x08000000;
    	else robot_key = robot_key&0xf7ffffff;
    	if(gpio_get_level(ROBOT_Y5)) robot_key = robot_key|0x10000000;
    	else robot_key = robot_key&0xefffffff;
     	if(gpio_get_level(ROBOT_Y6)) robot_key = robot_key|0x20000000;
    	else robot_key = robot_key&0xdfffffff;        
    }
    

    if(robot_model==2)    //joint move
    {
    	if((robot_tar_pulse-robot_cur_pulse)>0)robot_cur_pulse+=robot_cur_speed;


    	if(labs(cur_pulse1-pre_pulse1)<(pulse1_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse1<tar_pulse1)
    		{

    			cur_pulse1 = cur_pulse1+1;
    			

    		}
    		if(cur_pulse1>tar_pulse1)
    		{

    			cur_pulse1 = cur_pulse1-1;


    		}
    	}

    	
    	if(labs(cur_pulse2-pre_pulse2)<(pulse2_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse2<tar_pulse2)
    		{

    			cur_pulse2 = cur_pulse2+1;


    		}
    		if(cur_pulse2>tar_pulse2)
    		{

    			cur_pulse2 = cur_pulse2-1;


    		}
    	}



    	
    	if(labs(cur_pulse3-pre_pulse3)<(pulse3_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse3<tar_pulse3)
    		{

    			cur_pulse3 = cur_pulse3+1;
 

    		}
    		if(cur_pulse3>tar_pulse3)
    		{

    			cur_pulse3 = cur_pulse3-1;


    		}
    	}


    	if(labs(cur_pulse4-pre_pulse4)<(pulse4_scale*(robot_cur_pulse/1000))/1000)
    	{

    		if(cur_pulse4<tar_pulse4)
    		{
    	    	cur_pulse4 = cur_pulse4+1;

    		}
    		if(cur_pulse4>tar_pulse4)
    		{

    			cur_pulse4 = cur_pulse4-1;
    		}
    	}

    	if((labs(cur_pulse5-pre_pulse5)<(pulse5_scale*(robot_cur_pulse/1000))/1000)&&(is_step4_run==0))
    	{
    		if(cur_pulse5<tar_pulse5)
    		{
    			cur_pulse5 = cur_pulse5+1;
 
    		}
    		if(cur_pulse5>tar_pulse5)
    		{

    			cur_pulse5 = cur_pulse5-1;

    		}
    	}


    	if(labs(cur_pulse6-pre_pulse6)<(pulse6_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse6<tar_pulse6)
    		{
    			cur_pulse6 = cur_pulse6+1;



    		}
    		if(cur_pulse6>tar_pulse6)
    		{

    			cur_pulse6 = cur_pulse6-1;



    		}
    	}

    	if(num%4==0)
    	{
    		if(robot_cur_pulse<(robot_tar_pulse/2))robot_cur_speed = mysqrt(robot_cur_pulse/1000*robot_tar_acc*robot_global_speed/100);
    		else robot_cur_speed = mysqrt((robot_tar_pulse-robot_cur_pulse)/1000*robot_tar_acc*robot_global_speed/100);
    		if(robot_cur_speed>(robot_global_speed*robot_tar_speed)/10)robot_cur_speed=(robot_global_speed*robot_tar_speed)/10;
    		if(robot_cur_speed<2)robot_cur_speed=2;
    		if(robot_cur_speed>1000)robot_cur_speed=1000;
    		if(robot_global_speed==0)robot_cur_speed=0;
    	}

        

    }

    if(num%100==0&&is_send_twai==0x3f)
    {
        // tx_task_action_t tx_action;
        // tx_action = TX_SEND_DATA1;
        // xQueueSend(tx_task_queue, &tx_action, 1);

        twai_message_t tx_msg = {.data_length_code = 8, .ss = 1, .identifier = 0x0B1};
        tx_msg.data[3] = (cur_pulse1 & 0xff000000) >> 24; 
        tx_msg.data[2] = (cur_pulse1 & 0x00ff0000) >> 16;
        tx_msg.data[1] = (cur_pulse1 & 0x0000ff00) >> 8; 
        tx_msg.data[0] = (cur_pulse1 & 0x000000ff);
        tx_msg.data[7] = (cur_pulse2 & 0xff000000) >> 24; 
        tx_msg.data[6] = (cur_pulse2 & 0x00ff0000) >> 16;   
        tx_msg.data[5] = (cur_pulse2 & 0x0000ff00) >> 8; 
        tx_msg.data[4] = (cur_pulse2 & 0x000000ff);                                 
        twai_transmit(&tx_msg, portMAX_DELAY); 

        tx_msg.identifier = 0x0B2;
        tx_msg.data[3] = (cur_pulse3 & 0xff000000) >> 24; 
        tx_msg.data[2] = (cur_pulse3 & 0x00ff0000) >> 16;
        tx_msg.data[1] = (cur_pulse3 & 0x0000ff00) >> 8; 
        tx_msg.data[0] = (cur_pulse3 & 0x000000ff);
        tx_msg.data[7] = (cur_pulse4 & 0xff000000) >> 24; 
        tx_msg.data[6] = (cur_pulse4 & 0x00ff0000) >> 16;   
        tx_msg.data[5] = (cur_pulse4 & 0x0000ff00) >> 8; 
        tx_msg.data[4] = (cur_pulse4 & 0x000000ff);                                 
        twai_transmit(&tx_msg, portMAX_DELAY);     

        tx_msg.identifier = 0x0B3;
        tx_msg.data[3] = (cur_pulse5 & 0xff000000) >> 24; 
        tx_msg.data[2] = (cur_pulse5 & 0x00ff0000) >> 16;
        tx_msg.data[1] = (cur_pulse5 & 0x0000ff00) >> 8; 
        tx_msg.data[0] = (cur_pulse5 & 0x000000ff);
        tx_msg.data[7] = (cur_pulse6 & 0xff000000) >> 24; 
        tx_msg.data[6] = (cur_pulse6 & 0x00ff0000) >> 16;   
        tx_msg.data[5] = (cur_pulse6 & 0x0000ff00) >> 8; 
        tx_msg.data[4] = (cur_pulse6 & 0x000000ff);                                 
        twai_transmit(&tx_msg, portMAX_DELAY);               

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











static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
//            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
//            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
//            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
//            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
//                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
//                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
//                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
//        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static const httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "Hello World!"
};




static esp_err_t state_get_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "application/json; charset=utf-8");
    //const char* resp_str = (const char*) req->user_ctx;
    char send_str[600];
    
    sprintf(send_str,"{\"robot_key\":%lu,\"robot_model\":%ld,\"robot_global_speed\":%ld,\"robot_tar_speed\":%ld,"
    "\"robot_cur_speed\":%ld,\"robot_tar_acc\":%ld,\"robot_cur_acc\":%ld,"
    "\"cur_pulse1\":%ld,\"cur_pulse2\":%ld,\"cur_pulse3\":%ld,"
    "\"cur_pulse4\":%ld,\"cur_pulse5\":%ld,\"cur_pulse6\":%ld,"
    "\"robot_foc\":[%d,%d,%d,%d,%d,%d],\"system_val\":[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]}",
    		robot_key,robot_model,robot_global_speed,robot_tar_speed,
			robot_cur_speed,robot_tar_acc,robot_cur_acc,
			cur_pulse1,cur_pulse2,cur_pulse3,
			cur_pulse4,cur_pulse5,cur_pulse6,
            robot_foc[0],robot_foc[1],robot_foc[2],robot_foc[3],robot_foc[4],robot_foc[5],
            system_val[0],system_val[1],system_val[2],system_val[3],system_val[4] ,system_val[5],
            system_val[6],system_val[7],system_val[8],system_val[9],system_val[10],system_val[11]);
    httpd_resp_send(req, send_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t state = {
    .uri       = "/state",
    .method    = HTTP_GET,
    .handler   = state_get_handler,
    .user_ctx  = "Hello World!"
};

#ifndef myMIN
# define myMIN(a,b) ((a) < (b) ? (a) : (b))
#endif

static esp_err_t control_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;
    comtrol_time_out = 300;
    
    while (remaining > 0) {

        if ((ret = httpd_req_recv(req, buf,
        		myMIN(remaining, sizeof(buf)))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {

                continue;
            }
            return ESP_FAIL;
        }


        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

    }

    cJSON *root = NULL;
	cJSON *node1 = NULL;
	//cJSON *node2 = NULL;
	root = cJSON_Parse(buf);
	if(root)
	{
		node1 = cJSON_GetObjectItem(root, "robot_global_speed");
		if(node1)
		{
			robot_global_speed = node1->valueint;
            system_val[150] = robot_global_speed;
		}
		node1 = cJSON_GetObjectItem(root, "robot_key");
		if(node1)
		{
			robot_key = node1->valueint;
		}
		node1 = cJSON_GetObjectItem(root, "robot_model");
		if(node1)
		{
			robot_model = node1->valueint;
		}


	}
	cJSON_Delete(root);
	

    // End response
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "application/json; charset=utf-8");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t control = {
    .uri       = "/control",
    .method    = HTTP_POST,
    .handler   = control_post_handler,
    .user_ctx  = NULL
};


static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
        		myMIN(remaining, sizeof(buf)))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
//        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
//        ESP_LOGI(TAG, "%.*s", ret, buf);
//        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = NULL
};




static esp_err_t readfile_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    char send_str[100]={0};
    char file_path[200]={0};   

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    //const char* resp_str = (const char*) req->user_ctx;
    
    
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char param[180];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "name", param, sizeof(param)) == ESP_OK) {
                sprintf(file_path,"/littlefs/%s",param);
            }

        }
        free(buf);
    }

    FILE* f = fopen(file_path, "r");
    size_t fileSize;
    while(!feof(f)) 
    {
        fileSize = fread(send_str,sizeof(char),100,f);
        httpd_resp_send_chunk(req, send_str, fileSize);
    }
    // fread(send_str,sizeof(char),300,f);
    
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    // httpd_resp_send(req, send_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t readfile = {
    .uri       = "/readfile",
    .method    = HTTP_GET,
    .handler   = readfile_get_handler,
    .user_ctx  = "read file"
};



static esp_err_t writefile_post_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    char file_str[100]={0};
    char file_path[200]={0};
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char param[180];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "name", param, sizeof(param)) == ESP_OK) {
                sprintf(file_path,"/littlefs/%s",param);
            }

        }
        free(buf);
    }

    int ret, remaining = req->content_len;
    FILE* f = fopen(file_path, "w");
    while (remaining > 0) {

        if ((ret = httpd_req_recv(req, file_str,
        		myMIN(remaining, sizeof(file_str)))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {

                continue;
            }
            return ESP_FAIL;
        }
        fwrite(file_str,sizeof(char),ret,f);

        // httpd_resp_send_chunk(req, file_str, ret);
        remaining -= ret;

    }

    // fprintf(f, file_str);
    fclose(f);

    // End response
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    // httpd_resp_send_chunk(req, NULL, 0);
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t writefile = {
    .uri       = "/writefile",
    .method    = HTTP_POST,
    .handler   = writefile_post_handler,
    .user_ctx  = NULL
};



static esp_err_t filemsg_get_handler(httpd_req_t *req)
{
     //const char* resp_str = (const char*) req->user_ctx;
    char send_str[300]={0};
    char is_fist = 1;
    // esp_vfs_spiffs_register(&file_conf);
    size_t total = 0, used = 0;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "application/json; charset=utf-8");
    // esp_spiffs_info(file_conf.partition_label, &total, &used);
    esp_littlefs_info(littlefs_conf.partition_label, &total, &used);
    sprintf(send_str,"{\"total\":%d,\"used\":%d,\"name\":[",
            total,used);
    httpd_resp_send_chunk(req, send_str, HTTPD_RESP_USE_STRLEN);
    DIR *pDir = NULL;
    struct dirent *pEnt = NULL;
    pDir = opendir("/littlefs");
    if(pDir !=NULL)
    {
        while(1)
        {
            pEnt = readdir(pDir);
            if(pEnt !=NULL)
            {
                if(is_fist)
                {
                    sprintf(send_str,"\"%s\"",pEnt->d_name);
                    httpd_resp_send_chunk(req, send_str, HTTPD_RESP_USE_STRLEN);
                    is_fist =0;
                }else
                {
                    sprintf(send_str,",\"%s\"",pEnt->d_name);
                    httpd_resp_send_chunk(req, send_str, HTTPD_RESP_USE_STRLEN);
                }
            }else 
            {
                if(is_fist)
                {
                    sprintf(send_str,"\"null\"]}");
                }else
                {
                    sprintf(send_str,"]}");
                }
                httpd_resp_send_chunk(req, send_str, HTTPD_RESP_USE_STRLEN);
                break;
            }

        }
    }
    closedir(pDir);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t filemsg = {
    .uri       = "/filemsg",
    .method    = HTTP_GET,
    .handler   = filemsg_get_handler,
    .user_ctx  = "filemsg"
};



static esp_err_t delfile_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    char file_path[200]={0};   

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    //const char* resp_str = (const char*) req->user_ctx;
    
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char param[180];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "name", param, sizeof(param)) == ESP_OK) {
                sprintf(file_path,"/littlefs/%s",param);
            }

        }
        free(buf);
    }
    remove(file_path);

    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t delfile = {
    .uri       = "/delfile",
    .method    = HTTP_GET,
    .handler   = delfile_get_handler,
    .user_ctx  = "delete file"
};




static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.server_port = 8080;
    
    // Start the httpd server
//    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
//        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &state);
        httpd_register_uri_handler(server, &control);
        httpd_register_uri_handler(server, &echo);
        httpd_register_uri_handler(server, &readfile);
        httpd_register_uri_handler(server, &writefile);
        httpd_register_uri_handler(server, &filemsg);
        httpd_register_uri_handler(server, &delfile);


//        httpd_register_uri_handler(server, &ctrl);
        #if CONFIG_EXAMPLE_BASIC_AUTH
        httpd_register_basic_auth(server);
        #endif
        return server;
    }

//    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
//        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
//        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}




void robot_init(void)
{
   
     //zero-initialize the config structure.
    gpio_config_t output_conf = {};
    //disable interrupt
    output_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    output_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    output_conf.pin_bit_mask =
    	(
            (1ULL<<ROBOT_Y1) | (1ULL<<ROBOT_Y2) | (1ULL<<ROBOT_Y3) | (1ULL<<ROBOT_Y4) |
	(1ULL<<ROBOT_Y5) | (1ULL<<ROBOT_Y6)| (1ULL<<ROBOT_LED2)| 
    // (1ULL<<ROBOT_LED1)| (1ULL<<ROBOT_LED2)|
    (1ULL<<CAN_TX_GPIO));
    //disable pull-down mode
    output_conf.pull_down_en = 1;
    //disable pull-up mode
    output_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&output_conf);
   

    //zero-initialize the config structure.
    gpio_config_t input_conf = {};
    //disable interrupt
    input_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    input_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    input_conf.pin_bit_mask =
    	(
            (1ULL<<ROBOT_X1) | (1ULL<<ROBOT_X2) | (1ULL<<ROBOT_X3) | (1ULL<<ROBOT_X4) |
	    (1ULL<<ROBOT_X5) | (1ULL<<ROBOT_X6) |  
        (1ULL<<ECHO_TEST_RXD));
    //disable pull-down mode
    input_conf.pull_down_en = 1;
    //disable pull-up mode
    input_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&input_conf);


    //zero-initialize the config structure.
    gpio_config_t i_conf = {};
    //disable interrupt
    i_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    i_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    i_conf.pin_bit_mask =
    	((1ULL<<0)|(1ULL<<CAN_RX_GPIO));
    //disable pull-down mode
    i_conf.pull_down_en = 0;
    //disable pull-up mode
    i_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&i_conf);

    example_tg_timer_init(TIMER_GROUP_0, TIMER_1, true, 10);

}


static void twai_receive_task(void *arg)
{

    while (1) 
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY);
        if (rx_msg.identifier == 0x0A1) {
            int32_t data = 0;
            for (int i = 0; i < 4; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            } 
            robot_foc[0] = rx_msg.data[6];
            if(gpio_get_level(ROBOT_X5)==1||(is_send_twai&0x01)==0){
                cur_pulse1 = data;
                tar_pulse1 = data;
                is_send_twai = is_send_twai|0x01;
            }   
            // gpio_set_level(ROBOT_LED1, 1);
        }
        else if (rx_msg.identifier == 0x0A2) {
            int32_t data = 0;
            for (int i = 0; i < 4; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            } 
            robot_foc[1] = rx_msg.data[6];
            if(gpio_get_level(ROBOT_X5)==1||(is_send_twai&0x02)==0){
                cur_pulse2 = data;
                tar_pulse2 = data;
                is_send_twai = is_send_twai|0x02;
            }   
            // gpio_set_level(ROBOT_LED1, 1);
        }
        else if (rx_msg.identifier == 0x0A3) {
            int32_t data = 0;
            for (int i = 0; i < 4; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            } 
            robot_foc[2] = rx_msg.data[6];
            if(gpio_get_level(ROBOT_X5)==1||(is_send_twai&0x04)==0){
                cur_pulse3 = data;
                tar_pulse3 = data;
                is_send_twai = is_send_twai|0x04;
            }   
            // gpio_set_level(ROBOT_LED1, 1);
        }
        else if (rx_msg.identifier == 0x0A4) {
            int32_t data = 0;
            for (int i = 0; i < 4; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            } 
            robot_foc[3] = rx_msg.data[6];
            if(gpio_get_level(ROBOT_X5)==1||(is_send_twai&0x08)==0){
                cur_pulse4 = data;
                tar_pulse4 = data;
                is_send_twai = is_send_twai|0x08;
            }   
            // gpio_set_level(ROBOT_LED1, 1);
        }

        else if (rx_msg.identifier == 0x0A5) {
            int32_t data = 0;
            for (int i = 0; i < 4; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            } 
            robot_foc[4] = rx_msg.data[6];
            if(gpio_get_level(ROBOT_X5)==1||(is_send_twai&0x10)==0){
                cur_pulse5 = data;
                tar_pulse5 = data;
                is_send_twai = is_send_twai|0x10;
            }   
            // gpio_set_level(ROBOT_LED1, 1);
        }
        else if (rx_msg.identifier == 0x0A6) {
            int32_t data = 0;
            for (int i = 0; i < 4; i++) {
                data |= (rx_msg.data[i] << (i * 8));
            } 
            robot_foc[5] = rx_msg.data[6];
            if(gpio_get_level(ROBOT_X5)==1||(is_send_twai&0x20)==0){
                cur_pulse6 = data;
                tar_pulse6 = data;
                is_send_twai = is_send_twai|0x20;
            }   
            // gpio_set_level(ROBOT_LED1, 1);
        }


    }
}


static void twai_transmit_task(void *arg)
{
    vTaskDelay(5000 / portTICK_PERIOD_MS); 
    is_send_twai = 0x3f;
    while (1) {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_DATA1) {
            twai_message_t tx_msg = {.data_length_code = 8, .identifier = 0x0B1, .ss = 1};
            tx_msg.data[3] = (cur_pulse1 & 0xff000000) >> 24; 
            tx_msg.data[2] = (cur_pulse1 & 0x00ff0000) >> 16;
            tx_msg.data[1] = (cur_pulse1 & 0x0000ff00) >> 8; 
            tx_msg.data[0] = (cur_pulse1 & 0x000000ff);
            tx_msg.data[7] = (cur_pulse2 & 0xff000000) >> 24; 
            tx_msg.data[6] = (cur_pulse2 & 0x00ff0000) >> 16;   
            tx_msg.data[5] = (cur_pulse2 & 0x0000ff00) >> 8; 
            tx_msg.data[4] = (cur_pulse2 & 0x000000ff);                                 
            twai_transmit(&tx_msg, portMAX_DELAY);
            
        } else if (action == TX_SEND_DATA2) {
            twai_message_t tx_msg = {.data_length_code = 8, .identifier = 0x0B1, .ss = 1};
            tx_msg.data[3] = (cur_pulse1 & 0xff000000) >> 24; 
            tx_msg.data[2] = (cur_pulse1 & 0x00ff0000) >> 16;
            tx_msg.data[1] = (cur_pulse1 & 0x0000ff00) >> 8; 
            tx_msg.data[0] = (cur_pulse1 & 0x000000ff);
            tx_msg.data[7] = (cur_pulse2 & 0xff000000) >> 24; 
            tx_msg.data[6] = (cur_pulse2 & 0x00ff0000) >> 16;   
            tx_msg.data[5] = (cur_pulse2 & 0x0000ff00) >> 8; 
            tx_msg.data[4] = (cur_pulse2 & 0x000000ff);                                 
            twai_transmit(&tx_msg, portMAX_DELAY);
        } else if (action == TX_SEND_DATA3) {
            twai_message_t tx_msg = {.data_length_code = 8, .identifier = 0x0B1, .ss = 1};
            tx_msg.data[3] = (cur_pulse1 & 0xff000000) >> 24; 
            tx_msg.data[2] = (cur_pulse1 & 0x00ff0000) >> 16;
            tx_msg.data[1] = (cur_pulse1 & 0x0000ff00) >> 8; 
            tx_msg.data[0] = (cur_pulse1 & 0x000000ff);
            tx_msg.data[7] = (cur_pulse2 & 0xff000000) >> 24; 
            tx_msg.data[6] = (cur_pulse2 & 0x00ff0000) >> 16;   
            tx_msg.data[5] = (cur_pulse2 & 0x0000ff00) >> 8; 
            tx_msg.data[4] = (cur_pulse2 & 0x000000ff);                                 
            twai_transmit(&tx_msg, portMAX_DELAY);
        } else if (action == TX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}


static void twai_control_task(void *arg)
{
    vTaskDelay(100 / portTICK_PERIOD_MS); 
    // is_send_twai = 1;
    // twai_transmit(&ping_message, portMAX_DELAY);
    while (1) 
    {
        twai_message_t tx_msg = {.data_length_code = 4, .identifier = 0x0C1};
        if(gpio_get_level(ROBOT_X5))
        {  
            tx_msg.identifier = 0x0C1;
            tx_msg.data[0] = 10; 
            tx_msg.data[1] = 10; 
            tx_msg.data[2] = 0; 
            tx_msg.data[3] = 1; 
            twai_transmit(&tx_msg, portMAX_DELAY); 
            vTaskDelay(20 / portTICK_PERIOD_MS); 

            tx_msg.identifier = 0x0C2;
            twai_transmit(&tx_msg, portMAX_DELAY); 
            vTaskDelay(20 / portTICK_PERIOD_MS);   

            tx_msg.identifier = 0x0C3;
            twai_transmit(&tx_msg, portMAX_DELAY); 
            vTaskDelay(20 / portTICK_PERIOD_MS); 

            tx_msg.identifier = 0x0C4;
            twai_transmit(&tx_msg, portMAX_DELAY); 
            vTaskDelay(20 / portTICK_PERIOD_MS); 

            tx_msg.identifier = 0x0C5;
            twai_transmit(&tx_msg, portMAX_DELAY); 
            vTaskDelay(20 / portTICK_PERIOD_MS); 

            tx_msg.identifier = 0x0C6;
            twai_transmit(&tx_msg, portMAX_DELAY); 
            vTaskDelay(20 / portTICK_PERIOD_MS); 

        }
        else 
        {
            tx_msg.identifier = 0x0C1;
            tx_msg.data[0] = step1_up_foc;  
            tx_msg.data[1] = step1_down_foc;
            tx_msg.data[2] = 5; 
            tx_msg.data[3] = 0; 
            twai_transmit(&tx_msg, portMAX_DELAY);   
            vTaskDelay(20 / portTICK_PERIOD_MS); 

            tx_msg.identifier = 0x0C2;
            tx_msg.data[0] = step2_up_foc;  
            tx_msg.data[1] = step2_down_foc;
            tx_msg.data[2] = 5; 
            twai_transmit(&tx_msg, portMAX_DELAY);   
            vTaskDelay(20 / portTICK_PERIOD_MS);            
            
            tx_msg.identifier = 0x0C3;
            tx_msg.data[0] = step3_up_foc;  
            tx_msg.data[1] = step3_down_foc;
            tx_msg.data[2] = 5; 
            twai_transmit(&tx_msg, portMAX_DELAY);   
            vTaskDelay(20 / portTICK_PERIOD_MS); 
            
            tx_msg.identifier = 0x0C4;
            tx_msg.data[0] = step4_up_foc;  
            tx_msg.data[1] = step4_down_foc;
            tx_msg.data[2] = 5; 
            twai_transmit(&tx_msg, portMAX_DELAY);   
            vTaskDelay(20 / portTICK_PERIOD_MS); 
             
            tx_msg.identifier = 0x0C5;
            tx_msg.data[0] = step5_up_foc;  
            tx_msg.data[1] = step5_down_foc;
            tx_msg.data[2] = 5; 
            twai_transmit(&tx_msg, portMAX_DELAY);   
            vTaskDelay(20 / portTICK_PERIOD_MS); 

            tx_msg.identifier = 0x0C6;
            tx_msg.data[0] = step6_up_foc;  
            tx_msg.data[1] = step6_down_foc;
            tx_msg.data[2] = 5; 
            twai_transmit(&tx_msg, portMAX_DELAY);   
            vTaskDelay(20 / portTICK_PERIOD_MS);                                   
        }
        // gpio_set_level(ROBOT_LED1, 0);
        vTaskDelay(20 / portTICK_PERIOD_MS);   
    }
}


unsigned short CRC16_Modbus ( unsigned char *pdata, int len)
{
  unsigned short crc=0xFFFF;
  int i, j;
  for ( j=0; j<len;j++)
  {
    crc=crc^pdata[j];
    for ( i=0; i<8; i++)
    {
      if( ( crc&0x0001) >0)
      {
        crc=crc>>1;
        crc=crc^ 0xa001;
      }
      else
        crc=crc>>1;
    }
  }
  return crc;
}


static void echo_send(const int port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        // ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}


// An example of uart_485_task test with hardware flow control on UART
static void uart_485_task(void *arg)
{
    uint16_t crc = 0;
    const int uart_num = ECHO_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    // Set UART log level
    // esp_log_level_set(TAG, ESP_LOG_INFO);

    // ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));

    // Allocate buffers for UART
    uint8_t* rx_modbus = (uint8_t*) malloc(BUF_SIZE);
    uint8_t* tx_modbus = (uint8_t*) malloc(BUF_SIZE);
    // ESP_LOGI(TAG, "UART start recieve loop.\r\n");
 
    echo_send(uart_num, "Start RS485 UART test.\r\n", 24);



    while(1) {
        //Read data from UART

        int len = uart_read_bytes(uart_num, rx_modbus, BUF_SIZE, PACKET_READ_TICS);

        //Write data back to UART
        if (len > 0) {
            // uart_write_bytes(uart_num, rx_modbus, len);
            crc = CRC16_Modbus(rx_modbus,len-2);
            system_val[10]=BYTE0(crc);
            system_val[11]=BYTE1(crc);
            system_val[12]=crc;
            if(BYTE0(crc)==rx_modbus[len-2]&&BYTE1(crc)==rx_modbus[len-1])
            {
                if(rx_modbus[1]==0x03)
                {
                    uint16_t tx_len=0;
                    uint16_t adrr=0;
                    tx_modbus[tx_len++] = rx_modbus[0];
                    tx_modbus[tx_len++] = rx_modbus[1];
                    adrr = rx_modbus[2]*256+rx_modbus[3];
                    tx_modbus[tx_len++] = rx_modbus[5]*2;
                    for(int i=0; i<rx_modbus[5];i++,adrr++)
                    {
                        tx_modbus[tx_len++] = BYTE1(system_val[adrr]);
                        tx_modbus[tx_len++] = BYTE0(system_val[adrr]);
                    }
                    crc=CRC16_Modbus(tx_modbus,tx_len);
                    tx_modbus[tx_len++] = BYTE0(crc);
                    tx_modbus[tx_len++] = BYTE1(crc);
                    uart_write_bytes(uart_num, tx_modbus, tx_len);
                }
            }
            

        } 
        // else {
        //     // Echo a "." to show we are alive while we wait for input
        //     echo_send(uart_num, ".", 1);
        //     ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
        // }
    }

    vTaskDelete(NULL);
}




// robot.start()
STATIC mp_obj_t robot_start(void) {

    robot_init();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, 9, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, 10, NULL, tskNO_AFFINITY);
    xTaskCreate(uart_485_task, "uart_485", 4096, NULL, 11, NULL);
    return mp_obj_new_int(1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(robot_start_obj, robot_start);



STATIC mp_obj_t robot_get_cur_pulse(void) {

    return mp_obj_new_int(robot_cur_pulse);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(robot_get_cur_pulse_obj, robot_get_cur_pulse);




STATIC mp_obj_t robot_get_tar_pulse(void) {

    return mp_obj_new_int(robot_tar_pulse);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(robot_get_tar_pulse_obj, robot_get_tar_pulse);




STATIC mp_obj_t robot_set_pulse(size_t n_args, const mp_obj_t *args) {
    robot_model = 0;
    pre_pulse1 = cur_pulse1;
    pre_pulse2 = cur_pulse2;
    pre_pulse3 = cur_pulse3;
    pre_pulse4 = cur_pulse4;
    pre_pulse5 = cur_pulse5;
    pre_pulse6 = cur_pulse6;

    tar_pulse1 = mp_obj_get_int(args[0]);
    tar_pulse2 = mp_obj_get_int(args[1]);
    tar_pulse3 = mp_obj_get_int(args[2]);
    tar_pulse4 = mp_obj_get_int(args[3]);
    tar_pulse5 = mp_obj_get_int(args[4]);
    tar_pulse6 = mp_obj_get_int(args[5]);

    robot_change_pulse();
    robot_model = 2;
    // while(labs(robot_tar_pulse-robot_cur_pulse)>1000)vTaskDelay(10 / portTICK_PERIOD_MS);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_set_pulse_obj, 6, 6, robot_set_pulse);



STATIC mp_obj_t robot_set_speed(size_t n_args, const mp_obj_t *args) {

    robot_tar_speed = mp_obj_get_int(args[0]);
    robot_tar_acc = mp_obj_get_int(args[1]);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_set_speed_obj, 2, 2, robot_set_speed);



STATIC mp_obj_t robot_set_upfoc(size_t n_args, const mp_obj_t *args) {

    step1_up_foc = mp_obj_get_int(args[0]);

    step2_up_foc = mp_obj_get_int(args[1]);

    step3_up_foc = mp_obj_get_int(args[2]);

    step4_up_foc = mp_obj_get_int(args[3]);

    step5_up_foc = mp_obj_get_int(args[4]);

    step6_up_foc = mp_obj_get_int(args[5]);

    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_set_upfoc_obj, 6, 6, robot_set_upfoc);

STATIC mp_obj_t robot_set_downfoc(size_t n_args, const mp_obj_t *args) {

    step1_down_foc = mp_obj_get_int(args[0]);

    step2_down_foc = mp_obj_get_int(args[1]);

    step3_down_foc = mp_obj_get_int(args[2]);

    step4_down_foc = mp_obj_get_int(args[3]);

    step5_down_foc = mp_obj_get_int(args[4]);

    step6_down_foc = mp_obj_get_int(args[5]);
    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_set_downfoc_obj, 6, 6, robot_set_downfoc);




void http_server_init(void)
{
    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    // server = start_webserver();
}


// robot.http_server()
STATIC mp_obj_t robot_http_server(void) {

    static httpd_handle_t server = NULL;
    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &connect_handler, &server));
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_handler, &server));
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &connect_handler, &server));
    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_handler, &server));

   
    server = start_webserver();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(robot_http_server_obj, robot_http_server);



static void off_storage_task(void *arg)
{

    nvs_handle_t my_handle;
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    long var_speed = 0;
    uint16_t var_num = 0;
    // esp_vfs_spiffs_register(&file_conf);
    esp_vfs_littlefs_register(&littlefs_conf);
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    err = nvs_get_blob(my_handle, "sys_val", NULL, &required_size);
    err = nvs_get_blob(my_handle, "sys_val", system_val, &required_size);
    nvs_commit(my_handle);
    nvs_close(my_handle);
    robot_global_speed = system_val[150];
    // ESP_LOGI("ROBOT","sys_val size = %d",required_size);
    while (1) {
        if(gpio_get_level(ROBOT_X6)==1)
        {
            if(robot_global_speed>0)
            {
                var_speed = robot_global_speed;
                for(;robot_global_speed>0;robot_global_speed--)vTaskDelay(10 / portTICK_PERIOD_MS);
            }else
            {
                for(;robot_global_speed<var_speed;robot_global_speed++)vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            
            nvs_open("storage", NVS_READWRITE, &my_handle);
            nvs_set_blob(my_handle, "sys_val", system_val, 800);
            nvs_commit(my_handle);
            nvs_close(my_handle);
            while(gpio_get_level(ROBOT_X6)==1)vTaskDelay(100 / portTICK_PERIOD_MS);
            // // esp_vfs_spiffs_unregister(file_conf.partition_label);
            // esp_vfs_littlefs_unregister(littlefs_conf.partition_label);
            // while(1)
            // {
            //     gpio_set_level(ROBOT_LED2, 0);
            //     vTaskDelay(100 / portTICK_PERIOD_MS);
            //     gpio_set_level(ROBOT_LED2, 1);
            //     vTaskDelay(100 / portTICK_PERIOD_MS);  
            // }


    	}
        else if(gpio_get_level(0)==0)//reboot
        {
            gpio_set_level(ROBOT_LED2, 1);
            nvs_open("storage", NVS_READWRITE, &my_handle);
            nvs_set_blob(my_handle, "sys_val", system_val, 800);
            nvs_commit(my_handle);
            nvs_close(my_handle);
            gpio_set_level(ROBOT_LED2, 0);
            abort();
        }
        else
        {
            var_num++;
            if(robot_global_speed<=0)
            {
                if(var_num%10==0)gpio_set_level(ROBOT_LED2, 1);
                if(var_num%10==5)gpio_set_level(ROBOT_LED2, 0);
            }else gpio_set_level(ROBOT_LED2, 0);

            // if(gpio_get_level(0)==0){
            //     gpio_set_level(ROBOT_LED1, 0);
            //     vTaskDelay(100 / portTICK_PERIOD_MS);
            //     gpio_set_level(ROBOT_LED1, 1);
            //     vTaskDelay(100 / portTICK_PERIOD_MS);   
            // }
            vTaskDelay(20 / portTICK_PERIOD_MS); 

        }
     
    }
    vTaskDelete(NULL);
}

// robot.off_storage()
STATIC mp_obj_t robot_off_storage(void) {

    xTaskCreatePinnedToCore(off_storage_task, "OFF_STORGE", 4096, NULL, 7, NULL, tskNO_AFFINITY);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(robot_off_storage_obj, robot_off_storage);


// robot.get_sys_val(int n)
STATIC mp_obj_t robot_get_sys_val(size_t n_args, const mp_obj_t *args) { 
    return mp_obj_new_int(system_val[mp_obj_get_int(args[0])]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_get_sys_val_obj, 1, 1, robot_get_sys_val);

// robot.set_sys_val(int n)
STATIC mp_obj_t robot_set_sys_val(size_t n_args, const mp_obj_t *args) { 
    system_val[mp_obj_get_int(args[0])] = mp_obj_get_int(args[1]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_set_sys_val_obj, 2, 2, robot_set_sys_val);



STATIC mp_obj_t robot_record(mp_obj_t tuple) {
    size_t len1;
    size_t len2;
    mp_obj_t *elem1;
    mp_obj_t *elem2;
    mp_obj_get_array(tuple, &len1, &elem1);
    mp_obj_get_array(elem1[1], &len2, &elem2);
    system_val[0]= len1;
    system_val[1]= len2;
    system_val[2]= mp_obj_get_int(elem2[0]);

    // return mp_obj_new_int_from_uint(timeutils_mktime(mp_obj_get_int(elem[0]),
    //     mp_obj_get_int(elem[1]), mp_obj_get_int(elem[2]), mp_obj_get_int(elem[3]),
    //     mp_obj_get_int(elem[4]), mp_obj_get_int(elem[5])));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(robot_record_obj, robot_record);


STATIC mp_obj_t robot_open_file (mp_obj_t str) {
    char file_path[180]={0};
    const char *path = mp_obj_str_get_str(str);
    sprintf(file_path,"/littlefs/%s",path);
    FILE* f = fopen(file_path, "r");
    if(f)
    {
        size_t fileSize;
        fseek(f,0,SEEK_END);
        fileSize = ftell(f);
        char* file_string = malloc(fileSize);
        rewind(f);
        fileSize = fread(file_string,sizeof(char),20000,f); 
        fclose(f);
        return mp_obj_new_str(file_string, (fileSize - 1));               
    }
    else
    {
        fclose(f);
        return mp_const_none;       
    }


}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(robot_open_file_obj, robot_open_file);



STATIC const mp_rom_map_elem_t robot_module_globals_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_robot) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&robot_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pulse), MP_ROM_PTR(&robot_set_pulse_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_speed), MP_ROM_PTR(&robot_set_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_upfoc), MP_ROM_PTR(&robot_set_upfoc_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_downfoc), MP_ROM_PTR(&robot_set_downfoc_obj) },
    { MP_ROM_QSTR(MP_QSTR_http_server), MP_ROM_PTR(&robot_http_server_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cur_pulse), MP_ROM_PTR(&robot_get_cur_pulse_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tar_pulse), MP_ROM_PTR(&robot_get_tar_pulse_obj) },
    { MP_ROM_QSTR(MP_QSTR_off_storage), MP_ROM_PTR(&robot_off_storage_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sys_val), MP_ROM_PTR(&robot_get_sys_val_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sys_val), MP_ROM_PTR(&robot_set_sys_val_obj) },
    { MP_ROM_QSTR(MP_QSTR_record), MP_ROM_PTR(&robot_record_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_file), MP_ROM_PTR(&robot_open_file_obj) },

};


STATIC MP_DEFINE_CONST_DICT(robot_module_globals, robot_module_globals_table);

const mp_obj_module_t robot_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&robot_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_robot, robot_user_cmodule);
