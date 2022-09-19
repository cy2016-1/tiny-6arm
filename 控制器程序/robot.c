#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "driver/timer.h"
#include "math.h"

#include <esp_http_server.h>
#include "cJSON.h"
//#include "extmod/vfs.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#define TIMER_DIVIDER         (2)  //  Hardware timer clock divider
#define TIMER_SCALE           ((8*10)/(TIMER_DIVIDER))  // convert counter value to seconds

#define ROBOT_STP1    GPIO_NUM_0
#define ROBOT_DIR1    GPIO_NUM_1
#define ROBOT_STP2    GPIO_NUM_9
#define ROBOT_DIR2    GPIO_NUM_3
#define ROBOT_STP3    GPIO_NUM_4
#define ROBOT_DIR3    GPIO_NUM_11
#define ROBOT_STP4    GPIO_NUM_46
#define ROBOT_DIR4    GPIO_NUM_45
#define ROBOT_STP5    GPIO_NUM_42
#define ROBOT_DIR5    GPIO_NUM_41
#define ROBOT_STP6    GPIO_NUM_40
#define ROBOT_DIR6    GPIO_NUM_16

#define ROBOT_X1    GPIO_NUM_7
#define ROBOT_X2    GPIO_NUM_12
#define ROBOT_X3    GPIO_NUM_6
#define ROBOT_X4    GPIO_NUM_10
#define ROBOT_X5    GPIO_NUM_5
#define ROBOT_X6    GPIO_NUM_8
#define ROBOT_X7    GPIO_NUM_14
#define ROBOT_X8    GPIO_NUM_15

#define ROBOT_Y1    GPIO_NUM_15
#define ROBOT_Y2    GPIO_NUM_37
#define ROBOT_Y3    GPIO_NUM_36
#define ROBOT_Y4    GPIO_NUM_35
#define ROBOT_Y5    GPIO_NUM_34
#define ROBOT_Y6    GPIO_NUM_33
#define ROBOT_Y7    GPIO_NUM_48
#define ROBOT_Y8    GPIO_NUM_21


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

//电机2参数
long tar_pulse2=0;
long cur_pulse2=0;
long pre_pulse2=0;
long pulse2_scale=0;

//电机3参数
long tar_pulse3=0;
long cur_pulse3=0;
long pre_pulse3=0;
long pulse3_scale=0;

//电机4参数
long tar_pulse4=0;
long cur_pulse4=0;
long pre_pulse4=0;
long pulse4_scale=0;

//电机5参数
long tar_pulse5=0;
long cur_pulse5=0;
long pre_pulse5=0;
long pulse5_scale=0;

//电机6参数
long tar_pulse6=0;
long cur_pulse6=0;
long pre_pulse6=0;
long pulse6_scale=0;

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
long comtrol_time_out = 200; //200ms
int time_is_run = 0;


int is_step4_run = 0;

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
    	if(gpio_get_level(ROBOT_X7)) robot_key = robot_key|0x00400000;
    	else robot_key = robot_key&0xffbfffff;
    	if(gpio_get_level(ROBOT_X8)) robot_key = robot_key|0x00800000;
    	else robot_key = robot_key&0xff7fffff;

    	   	
    }    
    
    if(robot_model==1)
    {
    	gpio_set_level(ROBOT_STP1, 1);
    	gpio_set_level(ROBOT_STP2, 1);
    	gpio_set_level(ROBOT_STP3, 1);
    	gpio_set_level(ROBOT_STP4, 1);
    	gpio_set_level(ROBOT_STP5, 1);
    	gpio_set_level(ROBOT_STP6, 1);
    	if((robot_key&0x00000001)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR1, 0); 	    
    	    cur_pulse1 = cur_pulse1+1;
    	    gpio_set_level(ROBOT_STP1, 0);
    	    
    	}
    	if((robot_key&0x00000002)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR1, 1);    	 
    	    cur_pulse1 = cur_pulse1-1;
    	    gpio_set_level(ROBOT_STP1, 0);
    	    
    	}
    	
    	if((robot_key&0x00000004)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR2, 0);
    	    cur_pulse2 = cur_pulse2+1;
    	    gpio_set_level(ROBOT_STP2, 0);
    	    
    	}
    	if((robot_key&0x00000008)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR2, 1);
    	    cur_pulse2 = cur_pulse2-1;
    	    gpio_set_level(ROBOT_STP2, 0);
    	    
    	}    	
    	
  	if((robot_key&0x00000010)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR3, 0);
    	    cur_pulse3 = cur_pulse3+1;
    	    gpio_set_level(ROBOT_STP3, 0);
    	    
    	}
    	if((robot_key&0x00000020)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR3, 1);
    	    cur_pulse3 = cur_pulse3-1;
    	    gpio_set_level(ROBOT_STP3, 0);
    	    
    	}
    	if((robot_key&0x00000040)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR4, 0);
    	    gpio_set_level(ROBOT_DIR5, 1);
    	    cur_pulse4 = cur_pulse4+1;
    	    gpio_set_level(ROBOT_STP4, 0);
    	    gpio_set_level(ROBOT_STP5, 0);
    	    
    	}
    	if((robot_key&0x00000080)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR4, 1);
    	    gpio_set_level(ROBOT_DIR5, 0);
    	    cur_pulse4 = cur_pulse4-1;
    	    gpio_set_level(ROBOT_STP4, 0);
    	    gpio_set_level(ROBOT_STP5, 0);
    	    
    	}      	
  	if((robot_key&0x00000100)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR5, 0);
    	    gpio_set_level(ROBOT_DIR4, 0);
    	    cur_pulse5 = cur_pulse5+1;
    	    gpio_set_level(ROBOT_STP5, 0);
    	    gpio_set_level(ROBOT_STP4, 0);
    	    
    	}
    	if((robot_key&0x00000200)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR5, 1);
    	    gpio_set_level(ROBOT_DIR4, 1);
    	    cur_pulse5 = cur_pulse5-1;
    	    gpio_set_level(ROBOT_STP5, 0);
    	    gpio_set_level(ROBOT_STP4, 0);
    	    
    	}
    	if((robot_key&0x00000400)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR6, 0);
    	    cur_pulse6 = cur_pulse6+1;
    	    gpio_set_level(ROBOT_STP6, 0);
    	    
    	}
    	if((robot_key&0x00000800)&&(num%(2000/(robot_global_speed+1))==0))
    	{
    	    gpio_set_level(ROBOT_DIR6, 1);
    	    cur_pulse6 = cur_pulse6-1;
    	    gpio_set_level(ROBOT_STP6, 0);
    	    
    	}
    	
    	
    	gpio_set_level(ROBOT_Y1, (robot_key>>24)&0x00000001);    
    	gpio_set_level(ROBOT_Y2, (robot_key>>25)&0x00000001);
    	gpio_set_level(ROBOT_Y3, (robot_key>>26)&0x00000001);
    	gpio_set_level(ROBOT_Y4, (robot_key>>27)&0x00000001);
    	gpio_set_level(ROBOT_Y5, (robot_key>>28)&0x00000001);
    	gpio_set_level(ROBOT_Y6, (robot_key>>29)&0x00000001);
    	gpio_set_level(ROBOT_Y7, (robot_key>>30)&0x00000001);
    	gpio_set_level(ROBOT_Y8, (robot_key>>31)&0x00000001);	    	

   
    }
    

    if(robot_model==2)    //joint move
    {
    	if((robot_tar_pulse-robot_cur_pulse)>0)robot_cur_pulse+=robot_cur_speed;

    	gpio_set_level(ROBOT_STP1, 1);
    	if(labs(cur_pulse1-pre_pulse1)<(pulse1_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse1<tar_pulse1)
    		{
    			gpio_set_level(ROBOT_DIR1, 0);
    			cur_pulse1 = cur_pulse1+1;
    			gpio_set_level(ROBOT_STP1, 0);

    		}
    		if(cur_pulse1>tar_pulse1)
    		{
    			gpio_set_level(ROBOT_DIR1, 1);
    			cur_pulse1 = cur_pulse1-1;
    			gpio_set_level(ROBOT_STP1, 0);

    		}
    	}

    	gpio_set_level(ROBOT_STP2, 1);
    	if(labs(cur_pulse2-pre_pulse2)<(pulse2_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse2<tar_pulse2)
    		{
    			gpio_set_level(ROBOT_DIR2, 0);
    			cur_pulse2 = cur_pulse2+1;
    			gpio_set_level(ROBOT_STP2, 0);

    		}
    		if(cur_pulse2>tar_pulse2)
    		{
    			gpio_set_level(ROBOT_DIR2, 1);
    			cur_pulse2 = cur_pulse2-1;
    			gpio_set_level(ROBOT_STP2, 0);

    		}
    	}



    	gpio_set_level(ROBOT_STP3, 1);
    	if(labs(cur_pulse3-pre_pulse3)<(pulse3_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse3<tar_pulse3)
    		{
    			gpio_set_level(ROBOT_DIR3, 0);
    			cur_pulse3 = cur_pulse3+1;
    			gpio_set_level(ROBOT_STP3, 0);

    		}
    		if(cur_pulse3>tar_pulse3)
    		{
    			gpio_set_level(ROBOT_DIR3, 1);
    			cur_pulse3 = cur_pulse3-1;
    			gpio_set_level(ROBOT_STP3, 0);

    		}
    	}



    	gpio_set_level(ROBOT_STP4, 1);
    	gpio_set_level(ROBOT_STP5, 1);
    	is_step4_run = 0;
    	if(labs(cur_pulse4-pre_pulse4)<(pulse4_scale*(robot_cur_pulse/1000))/1000)
    	{
    		is_step4_run = 1;
    		if(cur_pulse4<tar_pulse4)
    		{
    			gpio_set_level(ROBOT_DIR4, 0);
    			gpio_set_level(ROBOT_DIR5, 1);
    			cur_pulse4 = cur_pulse4+1;
    			gpio_set_level(ROBOT_STP4, 0);
    			gpio_set_level(ROBOT_STP5, 0);

    		}
    		if(cur_pulse4>tar_pulse4)
    		{
    			gpio_set_level(ROBOT_DIR4, 1);
    			gpio_set_level(ROBOT_DIR5, 0);
    			cur_pulse4 = cur_pulse4-1;
    			gpio_set_level(ROBOT_STP4, 0);
    			gpio_set_level(ROBOT_STP5, 0);

    		}
    	}

    	if((labs(cur_pulse5-pre_pulse5)<(pulse5_scale*(robot_cur_pulse/1000))/1000)&&(is_step4_run==0))
    	{
    		if(cur_pulse5<tar_pulse5)
    		{
    			gpio_set_level(ROBOT_DIR5, 0);
    			gpio_set_level(ROBOT_DIR4, 0);
    			cur_pulse5 = cur_pulse5+1;
    			gpio_set_level(ROBOT_STP5, 0);
    			gpio_set_level(ROBOT_STP4, 0);


    		}
    		if(cur_pulse5>tar_pulse5)
    		{
    			gpio_set_level(ROBOT_DIR5, 1);
    			gpio_set_level(ROBOT_DIR4, 1);
    			cur_pulse5 = cur_pulse5-1;
    			gpio_set_level(ROBOT_STP5, 0);
    			gpio_set_level(ROBOT_STP4, 0);


    		}
    	}


    	gpio_set_level(ROBOT_STP6, 1);
    	if(labs(cur_pulse6-pre_pulse6)<(pulse6_scale*(robot_cur_pulse/1000))/1000)
    	{
    		if(cur_pulse6<tar_pulse6)
    		{
    			gpio_set_level(ROBOT_DIR6, 0);
    			cur_pulse6 = cur_pulse6+1;
    			gpio_set_level(ROBOT_STP6, 0);


    		}
    		if(cur_pulse6>tar_pulse6)
    		{
    			gpio_set_level(ROBOT_DIR6, 1);
    			cur_pulse6 = cur_pulse6-1;
    			gpio_set_level(ROBOT_STP6, 0);


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
    char send_str[300];
    
    sprintf(send_str,"{\"robot_key\":%lu,\"robot_model\":%ld,\"robot_global_speed\":%ld,\"robot_tar_speed\":%ld,"
    "\"robot_cur_speed\":%ld,\"robot_tar_acc\":%ld,\"robot_cur_acc\":%ld,"
    "\"cur_pulse1\":%ld,\"cur_pulse2\":%ld,\"cur_pulse3\":%ld,"
    "\"cur_pulse4\":%ld,\"cur_pulse5\":%ld,\"cur_pulse6\":%ld}",
    		robot_key,robot_model,robot_global_speed,robot_tar_speed,
			robot_cur_speed,robot_tar_acc,robot_cur_acc,
			cur_pulse1,cur_pulse2,cur_pulse3,
			cur_pulse4,cur_pulse5,cur_pulse6);
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
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask =
    	((1ULL<<ROBOT_STP1) | (1ULL<<ROBOT_DIR1) | (1ULL<<ROBOT_STP2) | (1ULL<<ROBOT_DIR2) |
    	(1ULL<<ROBOT_STP3) | (1ULL<<ROBOT_DIR3) | (1ULL<<ROBOT_STP4) | (1ULL<<ROBOT_DIR4) |
	(1ULL<<ROBOT_STP5) | (1ULL<<ROBOT_DIR5) | (1ULL<<ROBOT_STP6) | (1ULL<<ROBOT_DIR6) );
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    
     //zero-initialize the config structure.
    gpio_config_t output_conf = {};
    //disable interrupt
    output_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    output_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    output_conf.pin_bit_mask =
    	((1ULL<<ROBOT_Y1) | (1ULL<<ROBOT_Y2) | (1ULL<<ROBOT_Y3) | (1ULL<<ROBOT_Y4) |
	(1ULL<<ROBOT_Y5) | (1ULL<<ROBOT_Y6) | (1ULL<<ROBOT_Y7) | (1ULL<<ROBOT_Y8) );
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
    	((1ULL<<ROBOT_X1) | (1ULL<<ROBOT_X2) | (1ULL<<ROBOT_X3) | (1ULL<<ROBOT_X4) |
	(1ULL<<ROBOT_X5) | (1ULL<<ROBOT_X6) | (1ULL<<ROBOT_X7) | (1ULL<<ROBOT_X8));
    //disable pull-down mode
    input_conf.pull_down_en = 1;
    //disable pull-up mode
    input_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&input_conf);

    example_tg_timer_init(TIMER_GROUP_0, TIMER_1, true, 10);

}




// robot.start()
STATIC mp_obj_t robot_start(void) {

    robot_init();
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
    //while(labs(robot_tar_pulse-robot_cur_pulse)>1000)vTaskDelay(10 / portTICK_PERIOD_MS);
    return mp_obj_new_int(1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_set_pulse_obj, 6, 6, robot_set_pulse);



STATIC mp_obj_t robot_set_speed(size_t n_args, const mp_obj_t *args) {

    robot_tar_speed = mp_obj_get_int(args[0]);
    robot_tar_acc = mp_obj_get_int(args[1]);

    return mp_obj_new_int(1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(robot_set_speed_obj, 2, 2, robot_set_speed);


void http_server_init(void)
{
    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();
}


// robot.http_server()
STATIC mp_obj_t robot_http_server(void) {

    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();
    return mp_obj_new_int(1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(robot_http_server_obj, robot_http_server);

STATIC const mp_rom_map_elem_t robot_module_globals_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_robot) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&robot_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pulse), MP_ROM_PTR(&robot_set_pulse_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_speed), MP_ROM_PTR(&robot_set_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_http_server), MP_ROM_PTR(&robot_http_server_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cur_pulse), MP_ROM_PTR(&robot_get_cur_pulse_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tar_pulse), MP_ROM_PTR(&robot_get_tar_pulse_obj) },
	
};


STATIC MP_DEFINE_CONST_DICT(robot_module_globals, robot_module_globals_table);

const mp_obj_module_t robot_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&robot_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_robot, robot_user_cmodule, 1);
