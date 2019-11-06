#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef struct ramp_t
{
  int32_t count;
  int32_t scale;
  float   out;
  void  (*init)(struct ramp_t *ramp, int32_t scale);
  float (*calc)(struct ramp_t *ramp);
}ramp_t;

#define RAMP_GEN_DAFAULT \
{ \
              .count = 0, \
              .scale = 0, \
              .out = 0, \
              .init = &ramp_init, \
              .calc = &ramp_calc, \
            } \

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;
typedef __packed struct
{
		uint32_t press_time;
	
} D_TIME;
//快速开方
extern fp32 invSqrt(fp32 num);

//斜波函数初始化
void  ramp_init(ramp_t *ramp, int32_t scale);
//斜波函数计算
float ramp_calc(ramp_t *ramp);
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);
//判断符号位
extern fp32 sign(fp32 value);
extern int16_t sign_int16(int16_t value);
//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);
//角度格式化为-PI~PI
extern fp32 WrapToPi(fp32 Ang);
extern void Gyro_Init(uint32_t Delay);
fp32 Get_Sin(uint32_t max_angle,fp32 add_time);//add_time 角度改变的快慢，最小越快
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
#define RAD2DEG(x) ((x)*57.30)  //弧度转角度 //1rad = 180/π = 57.30°
#define DEG2RAD(x) ((x)*0.01745)  //角度转弧度//1°=π/180 ≈ 0.01745 rad
#endif
