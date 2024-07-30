#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "st7789.h"
#include "driver/gptimer.h"
#include "utime.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "MLX90640_I2C_Driver.h"
#include "esp_adc/adc_oneshot.h"
#include "MLX90640_API.h"
#include <math.h>

#define LV_TICK_PERIOD_MS 1

static const char *TAG = "Thermal Camera";

#define SENS_H	2*24
#define SENS_V  2*32

SemaphoreHandle_t xGuiSemaphore;

TaskHandle_t gui_task_Handle;

/*	ADC Handles*/
adc_oneshot_unit_handle_t bat_lev_adc_handle = NULL;
adc_cali_handle_t adc2_cali_handle = NULL;
float battery_lev;

/*Graphical objects*/
lv_img_dsc_t my_png;
static lv_obj_t * label_ta;
static lv_obj_t * label_tmax;
static lv_obj_t * label_tmin;
static lv_obj_t * label_bat;
lv_obj_t * bar;
/*Buffer for IR image*/
uint8_t *img_ir;
/* Flag to refresh data on screen*/
uint8_t redraw_f = 0;
uint8_t redraw_bat = 1;

/* Temperature variables definitions */
float tir_amb, tir_max, tir_min;

/* String for support in writing data on UI */
char supp_str[100];

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

/****************************************************************************************************
 * FUNCT:   gui_task
 * BRIEF:   task for UI management
 * RETURN:  void
 * ARG:     *arg
 ****************************************************************************************************/
static void gui_task(void *arg){

    lv_init();

    lv_display_t *display = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);

    static lv_color_t buf1[DISP_BUF_SIZE / 10];                        /*Declare a buffer for 1/10 screen size*/
    static lv_color_t buf2[DISP_BUF_SIZE / 10];                        /*Declare a buffer for 1/10 screen size*/
    lv_display_set_buffers(display, buf1, buf2, sizeof(buf1),LV_DISPLAY_RENDER_MODE_PARTIAL );  /*Initialize the display buffer.*/

    lv_display_set_flush_cb(display, st7789_flush);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /*Create label for ambient temperature visualization*/
    label_ta = lv_label_create(lv_scr_act());
    lv_label_set_text(label_ta, "Ta: 22°C");
    lv_obj_align(label_ta, LV_ALIGN_BOTTOM_LEFT, 12, -4);

    /*Create label for maximum temperature visualization*/
    label_tmax = lv_label_create(lv_scr_act());
    lv_label_set_text(label_tmax, "Tmax: 55.2°C");
    lv_obj_align(label_tmax, LV_ALIGN_BOTTOM_RIGHT, -12, -4);

    /*Create label for minimum temperature visualization*/
    label_tmin = lv_label_create(lv_scr_act());
    lv_label_set_text(label_tmin, "Tmin: 5.2°C");
    lv_obj_align(label_tmin, LV_ALIGN_BOTTOM_MID, -5, -4);

    /*	Create image buffer for IR array visualization	*/
    img_ir = malloc(SENS_H*SENS_V*2);
    for(int asd = 0; asd < SENS_H*SENS_V; asd ++){
    	img_ir[asd*2] = 0xFF;//(uint8_t)lv_color_hex(asd * 2180).full;
    	img_ir[asd*2+1] = 0;//lv_color_hex(asd * 2180).full>>8;
    }


    /*Initialize png to contain the image buffer*/
    my_png.header.w = SENS_V;
    my_png.header.h = SENS_H;
	my_png.data_size = SENS_V * SENS_H * 2;
	my_png.header.cf = LV_COLOR_FORMAT_RGB565;
	my_png.data = img_ir;


	/*Create graphical object fo thermal image visualization*/
    lv_obj_t * img1 = lv_img_create(lv_scr_act());
    lv_img_set_src(img1, &my_png);
    lv_obj_remove_style_all(img1);
    lv_obj_align(img1, LV_ALIGN_CENTER, 2, 0);
    lv_obj_set_size(img1, SENS_V, SENS_H);
    lv_img_set_antialias(img1, 1);
    lv_img_set_zoom(img1, 256 * 4.1);


    /*Create battery indicator*/
    static lv_style_t style_indic;
    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_LIGHT_GREEN));

    bar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(bar, 60, 15);
    lv_obj_align(bar,LV_ALIGN_TOP_RIGHT,-5,2);
    lv_bar_set_value(bar, 85, LV_ANIM_ON);
    lv_obj_add_style(bar, &style_indic, LV_PART_INDICATOR);

    label_bat = lv_label_create(lv_scr_act());
    lv_label_set_text(label_bat, "58%");
    lv_obj_align(label_bat, LV_ALIGN_TOP_RIGHT, -15, 2);
    lv_label_set_text(label_bat, "85%");


    while(1){
    	vTaskDelay(10);
    	if (pdTRUE == xSemaphoreTake(xGuiSemaphore, 50)) {
        	if(redraw_f){
        		redraw_f = 0;
        		lv_img_set_src(img1, &my_png);
        		sprintf(supp_str,"Ta: %.1f°C",tir_amb);
        		lv_label_set_text(label_ta, supp_str);
        		sprintf(supp_str,"Tmin: %.1f°C",tir_min);
        		lv_label_set_text(label_tmin, supp_str);
        		sprintf(supp_str,"Tmax: %.1f°C",tir_max);
        		lv_label_set_text(label_tmax, supp_str);
        	}
        	if(redraw_bat){
        		redraw_bat = 0;
        		if(battery_lev < 60){
        			lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_ORANGE));
        		    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_LIGHT_GREEN));
        		    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_HOR);
        		    lv_obj_report_style_change(&style_indic);
        		}else if(battery_lev > 65){
        			lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_LIGHT_GREEN));
        			lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_NONE);
        			lv_obj_report_style_change(&style_indic);
        		}
        		sprintf(supp_str,"%.0f%%",battery_lev);
        		lv_label_set_text(label_bat, supp_str);
        		lv_bar_set_value(bar, (int32_t)battery_lev, LV_ANIM_ON);
        	}
    		lv_task_handler();
    		xSemaphoreGive(xGuiSemaphore);
    	}

    }

}


/****************************************************************************************************
 * FUNCT:   temp_to_jet_color
 * BRIEF:    Function to map temperature to color using the Jet colormap
 * RETURN:  void
 * ARG:     float temp = Temp to map
 * 			float min_temp, float max_temp = Min and Max for normalizing
 * 			uint8_t *r, uint8_t *g, uint8_t *b = r b g output colors
 ****************************************************************************************************/
void temp_to_jet_color(float temp, float min_temp, float max_temp, uint8_t *r, uint8_t *g, uint8_t *b) {
    float norm_temp = (temp - min_temp) / (max_temp - min_temp);
    norm_temp = fmin(fmax(norm_temp, 0.0), 1.0); // Clamp value between 0 and 1

    if (norm_temp < 0.125) {
        *r = 0;
        *g = 0;
        *b = (uint8_t)(128 + 127 * norm_temp / 0.125);
    } else if (norm_temp < 0.375) {
        *r = 0;
        *g = (uint8_t)(255 * (norm_temp - 0.125) / 0.25);
        *b = 255;
    } else if (norm_temp < 0.625) {
        *r = (uint8_t)(255 * (norm_temp - 0.375) / 0.25);
        *g = 255;
        *b = (uint8_t)(255 - 255 * (norm_temp - 0.375) / 0.25);
    } else if (norm_temp < 0.875) {
        *r = 255;
        *g = (uint8_t)(255 - 255 * (norm_temp - 0.625) / 0.25);
        *b = 0;
    } else {
        *r = (uint8_t)(255 - 128 * (norm_temp - 0.875) / 0.125);
        *g = 0;
        *b = 0;
    }
}

/****************************************************************************************************
 * FUNCT:   IR_Array_task
 * BRIEF:   Read IR array sensor MLX90960
 * RETURN:  void
 * ARG:     *arg
 ****************************************************************************************************/
static void IR_Array_task(void *arg){

static uint16_t eeMLX90640[832];
static uint16_t mlx90640Frame[834];
paramsMLX90640 mlx90640;
static float mlx90640To[768];
int status;
float t_pixel = 0;
uint8_t colo_r,colo_g,colo_b;
float t_d_min, t_d_max;
int temp_min, temp_max;
uint16_t data_t;

printf("Starting IR Cam...\n");
MLX90640_I2CInit();
MLX90640_SetRefreshRate(MLX90640_I2C_ADD,MLX90640_REFRESH_RATE_16HZ);
status = MLX90640_DumpEE (MLX90640_I2C_ADD, eeMLX90640);
if(status != ESP_OK){
	ESP_LOGI(TAG, "Error in reading MLX90640_DumpEE");
}
status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
if(status != ESP_OK){
	ESP_LOGI(TAG, "Error in reading MLX90640_ExtractParameters");
}


while (1){
	if(MLX90640_GetFrameData(MLX90640_I2C_ADD, mlx90640Frame) >= 0){
		tir_amb = MLX90640_GetTa(mlx90640Frame, &mlx90640) - TA_SHIFT;
	    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMISSIVITY, tir_amb, mlx90640To);

	    /* Semaphore to update user interface data */
	    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
	    	/* find the minimum and maximum */
			tir_max = -40;		//Set as mlx90640 absolute minimum temperature
			tir_min = 300;		//Set as mlx90640 absolute maximum temperature
		    for(int r_i = 0; r_i < SENS_H/2; r_i ++){
		        for(int h_i = 0; h_i < SENS_V/2; h_i ++){
		        	t_pixel = mlx90640To[(SENS_V/2 - 1 - h_i) + r_i * SENS_V/2];
		        	if(tir_max  < t_pixel ){
		        		tir_max = t_pixel;
		        	}
		        	if(tir_min > t_pixel ){
		        		tir_min = t_pixel;
		        	}
		        }
		    }
		    /* Map the thermal image */
		    for(int r_i = 0; r_i < SENS_H/2; r_i ++){
		        for(int h_i = 0; h_i < SENS_V/2; h_i ++){
		        	t_pixel = mlx90640To[(SENS_V/2 - 1 - h_i) + r_i * SENS_V/2];
		        	/* Set min and max for the colored image dinamic range */
		        	t_d_min = 27;
		        	t_d_max = 34;

		        	/* Convert temparature in colors */
		        	temp_to_jet_color(t_pixel, t_d_min, t_d_max, &colo_r, &colo_g, &colo_b);

		        	data_t = lv_color_to_u16(lv_color_make(colo_r,colo_g,colo_b));

		        	/* Copy data into the image buffer*/
		            img_ir[(((2*r_i)*SENS_V)+(h_i*2))*2]	= (uint8_t) data_t;
		            img_ir[(((2*r_i)*SENS_V)+(h_i*2))*2+1]= (uint8_t) (data_t >> 8);

		    		img_ir[(((2*r_i)*SENS_V)+(h_i*2 + 1))*2]= (uint8_t) data_t;
		    		img_ir[(((2*r_i)*SENS_V)+(h_i*2 + 1))*2+1]= (uint8_t) (data_t >> 8);

		    		img_ir[(((2*r_i+1)*SENS_V)+(h_i*2))*2]= (uint8_t) data_t;
		    		img_ir[(((2*r_i+1)*SENS_V)+(h_i*2))*2+1]= (uint8_t) (data_t >> 8);

		    		img_ir[(((2*r_i+1)*SENS_V)+(h_i*2 + 1))*2]= (uint8_t) data_t;
		    		img_ir[(((2*r_i+1)*SENS_V)+(h_i*2 + 1))*2+1]= (uint8_t) (data_t >> 8);

		        }
		    }
		    if(redraw_f == 0){
		    	redraw_f = 1;
		    }
	    	xSemaphoreGive(xGuiSemaphore);
	    }

	}else{
		ESP_LOGI("MLX90640","Error get frame");
	}
    vTaskDelay( 5 );
}
}

/****************************************************************************************************
 * FUNCT:   configure_bat_lev_adc
 * BRIEF:   Configure the ADC for battery level.
 * RETURN:  void
 * ARG:     void
 ****************************************************************************************************/
void configure_bat_lev_adc(void){


    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_2,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &bat_lev_adc_handle));


    adc_cali_curve_fitting_config_t cali_config;
    cali_config.unit_id = ADC_UNIT_2;
    cali_config.chan = ADC_CHANNEL_3;
	cali_config.atten = ADC_ATTEN_DB_11;//ADC_ATTEN_DB_0,
	cali_config.bitwidth = ADC_BITWIDTH_12;
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc2_cali_handle);

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,// ADC_ATTEN_DB_0,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(bat_lev_adc_handle, ADC_CHANNEL_3, &config));

}

/****************************************************************************************************
 * FUNCT:   bat_lev_task
 * BRIEF:   Read ADC for battery level
 * RETURN:  void
 * ARG:     *arg
 ****************************************************************************************************/
static void bat_lev_task(void *arg){

	static int adc_raw[10];
	static int adc_mv[10];

	while(1){
	    ESP_ERROR_CHECK(adc_oneshot_read(bat_lev_adc_handle, ADC_CHANNEL_3, &adc_raw[0]));
	    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[0], &adc_mv[0]));

	    /* Convert Voltage into bat % - this is battery specific so you need to adjust for your battery*/
	    battery_lev = adc_mv[0];
	    battery_lev *= 1.71438;
	    battery_lev /= 1000;
	    if(battery_lev > 3.75){
	    	battery_lev = 100 - (4.1 - battery_lev)/0.4 * 10;
	    	if(battery_lev > 100){
	    		battery_lev = 100;
	    	}
	    }else if(battery_lev > 3.55){
	    	battery_lev = 90 - (3.75 - battery_lev)/0.2 * 70 ;
	    }else{
	    	battery_lev = (3.4 - battery_lev)/0.15 * 20;
	    	if(battery_lev < 0){
	    		battery_lev = 0;
	    	}
	    }
	    redraw_bat = 1;
		vTaskDelay(1000);
	}

}

void app_main(void)
{

    xGuiSemaphore = xSemaphoreCreateMutex();
	configure_bat_lev_adc();
    spi_display_init();
    st7789_init();

    xTaskCreatePinnedToCore(bat_lev_task, "batt_read", 4*1024, NULL, 5, NULL,0 );
    xTaskCreatePinnedToCore(gui_task, "gui", 18*1024, NULL, 5, &gui_task_Handle,1 );
    vTaskDelay(1000);
    xTaskCreatePinnedToCore(IR_Array_task, "ir_array", 18*1024, NULL, 5, NULL,1 );


    while (1) {

    	vTaskDelay(1000);
    }
}
