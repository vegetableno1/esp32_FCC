#include "preloader.h"
#include "lvgl.h"

static void bgk_anim(lv_task_t* t)
{
	static uint32_t x = 0;
	lv_obj_t* bg = t->user_data;
	x = !x;
	lv_obj_set_style_local_bg_color(bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, x?LV_COLOR_YELLOW: LV_COLOR_OLIVE);//设置背景颜色为绿色
}

void create_preloader_demo(){
    lv_obj_t * preload = lv_spinner_create(lv_scr_act(),NULL);
    lv_obj_set_size(preload,100,100);
    lv_obj_align(preload,NULL,LV_ALIGN_CENTER,0,0);
}


void Home_Page_Create(void)
 
{
 

	lv_obj_t* bgk;
	bgk = lv_obj_create(lv_scr_act(), NULL);//创建对象
	lv_obj_clean_style_list(bgk, LV_OBJ_PART_MAIN); //清空对象风格
	lv_obj_set_style_local_bg_opa(bgk, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_100);//设置颜色覆盖度100%，数值越低，颜色越透。
	lv_obj_set_style_local_bg_color(bgk, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_OLIVE);//设置背景颜色为绿色
	//省去下方两行代码，默认是从0,0处开始绘制
	lv_obj_set_x(bgk, 0);//设置X轴起点
	lv_obj_set_y(bgk, 0);//设置Y轴起点
 
	lv_obj_set_size(bgk, 240, 240);//设置覆盖大小
	lv_task_create(bgk_anim, 50, LV_TASK_PRIO_LOW, bgk);//创建任务 500ms一次 
}

void lv_font_demo(void)
{
	lv_obj_t* font_label;
	lv_obj_t* font_label1;
	lv_obj_t* src = lv_scr_act();

	static lv_style_t font_style;
	lv_style_init(&font_style);
	lv_style_set_text_font(&font_style, LV_STATE_DEFAULT, &lv_font_montserrat_16);
	font_label = lv_label_create(src, NULL);
	lv_obj_add_style(font_label,LV_LABEL_PART_MAIN, &font_style);
	lv_label_set_text_static(font_label, LV_SYMBOL_AUDIO " audio");
	lv_obj_align(font_label, NULL, LV_ALIGN_CENTER, 0, 0);

	static lv_style_t font_style1;
	lv_style_init(&font_style1);
	lv_style_set_text_font(&font_style1, LV_STATE_DEFAULT, &lv_font_montserrat_16);
	font_label1 = lv_label_create(src, NULL);
	lv_obj_add_style(font_label1, LV_LABEL_PART_MAIN, &font_style1);
	lv_label_set_text_static(font_label1, LV_SYMBOL_VIDEO " video");
	lv_obj_align(font_label1, NULL, LV_ALIGN_CENTER, 0, 30);
}
