/********************************* main.c *********************************
 * 手环质检器 - 简化PA4功能：测量时切换标准组，停止时切换显示模式
 * PA2:启动/停止  PA3:清空统计  PA4:多功能  PA6:分页显示标准
 ****************************************************************************/

#include "stm32f10x.h"
#include "AD.h"
#include "Freq.h"
#include "OLED.h"
#include <stdio.h>

// ==================== 标准组定义 ====================
typedef struct {
    float volt_good_min;
    float volt_good_max;
    float volt_pass_min;
    float volt_pass_max;
    uint32_t freq_good_min;
    uint32_t freq_good_max;
    uint32_t freq_pass_min;
    uint32_t freq_pass_max;
} StdSet;

const StdSet std_table[] = {
    { 2.0f, 3.0f, 1.0f, 2.0f, 5000, 10000, 1000, 5000 },   // 原始
    { 2.2f, 2.8f, 1.2f, 1.8f, 6000, 9000, 2000, 4000 },    // 严格
    { 1.8f, 3.2f, 0.8f, 1.8f, 4000, 11000, 800, 4000 }      // 宽松
};
#define STD_GROUPS  (sizeof(std_table)/sizeof(StdSet))
static uint8_t current_std_idx = 0;

// ==================== 校准数据 ====================
#define ADC_RAW_0V      0
#define ADC_RAW_3V3     4095
#define VOLT_SLOPE      (3300.0f / (ADC_RAW_3V3 - ADC_RAW_0V))
#define VOLT_OFFSET     (-ADC_RAW_0V * VOLT_SLOPE)

#define VOLT_TOLERANCE  0.005f
#define FREQ_TOLERANCE  5

// 引脚定义
#define KEY_START_PIN   GPIO_Pin_2
#define KEY_START_PORT  GPIOA
#define KEY_CLEAR_PIN   GPIO_Pin_3
#define KEY_CLEAR_PORT  GPIOA
#define KEY_RATE_PIN    GPIO_Pin_4   // PA4
#define KEY_RATE_PORT   GPIOA
#define KEY_STD_PIN     GPIO_Pin_6
#define KEY_STD_PORT    GPIOA

#define LED_GOOD_PIN    GPIO_Pin_0
#define LED_GOOD_PORT   GPIOB
#define LED_PASS_PIN    GPIO_Pin_1
#define LED_PASS_PORT   GPIOB
#define LED_BAD_PIN     GPIO_Pin_10
#define LED_BAD_PORT    GPIOB

static uint32_t total_cnt = 0, good_cnt = 0, pass_cnt = 0, bad_cnt = 0;
static uint8_t measuring = 1;          // 1=测量中，0=停止
static uint8_t display_mode = 0;       // 0=正常模式，1=统计模式

// 延时函数
void delay_us(uint32_t us) {
    uint32_t i; for (i = 0; i < us * 8; i++);
}
void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

// GPIO初始化
static void GPIO_Config(void) {
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = KEY_START_PIN | KEY_CLEAR_PIN | KEY_RATE_PIN | KEY_STD_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY_START_PORT, &gpio);
    gpio.GPIO_Pin = LED_GOOD_PIN | LED_PASS_PIN | LED_BAD_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_WriteBit(LED_GOOD_PORT, LED_GOOD_PIN, Bit_RESET);
    GPIO_WriteBit(LED_PASS_PORT, LED_PASS_PIN, Bit_RESET);
    GPIO_WriteBit(LED_BAD_PORT, LED_BAD_PIN, Bit_RESET);
}

// 按键扫描（下降沿 + 消抖）
static uint8_t Key_Get(uint8_t pin, GPIO_TypeDef *port) {
    static uint8_t last[4] = {0};
    uint8_t idx = 0;
    if (pin == KEY_START_PIN && port == KEY_START_PORT) idx = 0;
    else if (pin == KEY_CLEAR_PIN && port == KEY_CLEAR_PORT) idx = 1;
    else if (pin == KEY_RATE_PIN && port == KEY_RATE_PORT) idx = 2;
    else if (pin == KEY_STD_PIN && port == KEY_STD_PORT) idx = 3;
    else return 0;
    uint8_t now = GPIO_ReadInputDataBit(port, pin) == Bit_RESET ? 1 : 0;
    if (now && !last[idx]) {
        delay_ms(20);
        if (GPIO_ReadInputDataBit(port, pin) == Bit_RESET) {
            last[idx] = now;
            return 1;
        }
    }
    if (!now) last[idx] = 0;
    return 0;
}

// 直接使用宏简化调用
#define KEY_START()  Key_Get(KEY_START_PIN, KEY_START_PORT)
#define KEY_CLEAR()  Key_Get(KEY_CLEAR_PIN, KEY_CLEAR_PORT)
#define KEY_PA4()    Key_Get(KEY_RATE_PIN, KEY_RATE_PORT)
#define KEY_STD()    Key_Get(KEY_STD_PIN, KEY_STD_PORT)

// 检测任意按键（用于退出标准界面）
static uint8_t Key_Any(void) {
    return (GPIO_ReadInputDataBit(KEY_START_PORT, KEY_START_PIN) == Bit_RESET) ||
           (GPIO_ReadInputDataBit(KEY_CLEAR_PORT, KEY_CLEAR_PIN) == Bit_RESET) ||
           (GPIO_ReadInputDataBit(KEY_RATE_PORT, KEY_RATE_PIN) == Bit_RESET) ||
           (GPIO_ReadInputDataBit(KEY_STD_PORT, KEY_STD_PIN) == Bit_RESET);
}

// 品质判断（使用当前标准组）
static uint8_t JudgeQuality(uint16_t volt_mv, uint32_t freq_hz) {
    float volt = volt_mv / 1000.0f;
    uint8_t volt_type = 2, freq_type = 2;
    const StdSet *std = &std_table[current_std_idx];
    if (volt >= (std->volt_good_min - VOLT_TOLERANCE) && volt <= (std->volt_good_max + VOLT_TOLERANCE))
        volt_type = 0;
    else if (volt >= (std->volt_pass_min - VOLT_TOLERANCE) && volt <= (std->volt_pass_max + VOLT_TOLERANCE))
        volt_type = 1;
    if (freq_hz >= (std->freq_good_min - FREQ_TOLERANCE) && freq_hz <= (std->freq_good_max + FREQ_TOLERANCE))
        freq_type = 0;
    else if (freq_hz >= (std->freq_pass_min - FREQ_TOLERANCE) && freq_hz <= (std->freq_pass_max + FREQ_TOLERANCE))
        freq_type = 1;
    if (volt_type == 0 && freq_type == 0) return 0;
    if (volt_type == 1 && freq_type == 1) return 1;
    return 2;
}

// LED指示
static void LightLED(uint8_t quality) {
    GPIO_WriteBit(LED_GOOD_PORT, LED_GOOD_PIN, Bit_RESET);
    GPIO_WriteBit(LED_PASS_PORT, LED_PASS_PIN, Bit_RESET);
    GPIO_WriteBit(LED_BAD_PORT, LED_BAD_PIN, Bit_RESET);
    if (quality == 0) GPIO_WriteBit(LED_GOOD_PORT, LED_GOOD_PIN, Bit_SET);
    else if (quality == 1) GPIO_WriteBit(LED_PASS_PORT, LED_PASS_PIN, Bit_SET);
    else GPIO_WriteBit(LED_BAD_PORT, LED_BAD_PIN, Bit_SET);
    delay_ms(1000);
    GPIO_WriteBit(LED_GOOD_PORT, LED_GOOD_PIN, Bit_RESET);
    GPIO_WriteBit(LED_PASS_PORT, LED_PASS_PIN, Bit_RESET);
    GPIO_WriteBit(LED_BAD_PORT, LED_BAD_PIN, Bit_RESET);
}

// 读取电压（mV）
static uint16_t Get_Voltage_mV(void) {
    #define SAMPLES 8
    uint32_t sum = 0;
    for (int i = 0; i < SAMPLES; i++) sum += AD_GetValue();
    uint16_t adc_avg = sum / SAMPLES;
    float volt_mv = adc_avg * VOLT_SLOPE + VOLT_OFFSET;
    if (volt_mv < 0) volt_mv = 0;
    if (volt_mv > 3300) volt_mv = 3300;
    return (uint16_t)volt_mv;
}

// 正常模式显示
static void OLED_ShowNormalMode(uint16_t volt_mv, uint32_t freq_hz, uint8_t quality) {
    char *quality_str = "Bad";
    if (quality == 0) quality_str = "Good";
    else if (quality == 1) quality_str = "Pass";
    char buf[17];
    OLED_Clear();
    sprintf(buf, "V:%.2fV", volt_mv / 1000.0f);
    OLED_ShowString(1, 1, buf);
    sprintf(buf, "F:%05dHz", (int)freq_hz);
    OLED_ShowString(2, 1, buf);
    sprintf(buf, "Q:%s", quality_str);
    OLED_ShowString(3, 1, buf);
    sprintf(buf, "T%03lu G%03lu P%03lu", total_cnt, good_cnt, pass_cnt);
    OLED_ShowString(4, 1, buf);
}

// 统计模式显示
static void OLED_ShowStatMode(void) {
    float good_rate = 0.0f, pass_rate = 0.0f, bad_rate = 0.0f;
    if (total_cnt > 0) {
        good_rate = (float)good_cnt / total_cnt * 100.0f;
        pass_rate = (float)pass_cnt / total_cnt * 100.0f;
        bad_rate  = (float)bad_cnt  / total_cnt * 100.0f;
    }
    char buf[17];
    OLED_Clear();
    OLED_ShowString(1, 1, "=== Statistics ===");
    sprintf(buf, "Total: %lu", total_cnt);
    OLED_ShowString(2, 1, buf);
    sprintf(buf, "G:%.1f%% P:%.1f%%", good_rate, pass_rate);
    OLED_ShowString(3, 1, buf);
    sprintf(buf, "B:%.1f%%", bad_rate);
    OLED_ShowString(4, 1, buf);
}

// 分页显示标准（第一页电压，第二页频率）
static void OLED_ShowStdPage1(void) {
    const StdSet *std = &std_table[current_std_idx];
    char buf[17];
    OLED_Clear();
    sprintf(buf, "Std%d V", current_std_idx+1);
    OLED_ShowString(1, 1, buf);
    sprintf(buf, "G:%.1f-%.1fV", std->volt_good_min, std->volt_good_max);
    OLED_ShowString(2, 1, buf);
    sprintf(buf, "P:%.1f-%.1fV", std->volt_pass_min, std->volt_pass_max);
    OLED_ShowString(3, 1, buf);
    OLED_ShowString(4, 1, "Any key next");
}
static void OLED_ShowStdPage2(void) {
    const StdSet *std = &std_table[current_std_idx];
    char buf[17];
    OLED_Clear();
    sprintf(buf, "Std%d Freq", current_std_idx+1);
    OLED_ShowString(1, 1, buf);
    sprintf(buf, "G:%d-%dHz", std->freq_good_min, std->freq_good_max);
    OLED_ShowString(2, 1, buf);
    sprintf(buf, "P:%d-%dHz", std->freq_pass_min, std->freq_pass_max);
    OLED_ShowString(3, 1, buf);
    OLED_ShowString(4, 1, "Any key quit");
}

// 显示切换标准组提示
static void OLED_ShowStdSwitch(void) {
    char buf[17];
    OLED_Clear();
    sprintf(buf, "Std Set %d", current_std_idx+1);
    OLED_ShowString(1, 1, buf);
    delay_ms(800);
    OLED_Clear();
}

int main(void) {
    OLED_Init(); OLED_Clear();
    AD_Init();
    FREQ_Init();
    GPIO_Config();

    OLED_ShowString(1, 1, "Ready");
    delay_ms(1000);
    OLED_Clear();

    while (1) {
        // ===== 处理按键 =====
        // 启动/停止 (PA2)
        if (KEY_START()) {
            measuring = !measuring;
            OLED_Clear();
            if (measuring) OLED_ShowString(1, 1, "Measuring");
            else OLED_ShowString(1, 1, "Stopped");
            delay_ms(500);
            OLED_Clear();
        }
        // 清空统计 (PA3)
        if (KEY_CLEAR()) {
            total_cnt = good_cnt = pass_cnt = bad_cnt = 0;
            OLED_Clear();
            OLED_ShowString(1, 1, "Cleared");
            delay_ms(500);
            if (!measuring && display_mode == 0) OLED_Clear();  // 停止且正常模式则清屏
        }
        // PA4 多功能：测量时切换标准组，停止时切换显示模式
        if (KEY_PA4()) {
            if (measuring) {
                // 测量状态：切换标准组
                current_std_idx = (current_std_idx + 1) % STD_GROUPS;
                OLED_ShowStdSwitch();
            } else {
                // 停止状态：切换显示模式（正常/统计）
                display_mode = !display_mode;
                OLED_Clear();
                OLED_ShowString(1, 1, display_mode ? "Stat Mode" : "Normal Mode");
                delay_ms(500);
                OLED_Clear();
            }
        }
        // 显示标准 (PA6) 分两页
        if (KEY_STD()) {
            OLED_ShowStdPage1();
            // 等待任意按键进入下一页
            while (!Key_Any()) delay_ms(50);
            while (Key_Any()) delay_ms(10); // 消抖释放
            OLED_ShowStdPage2();
            while (!Key_Any()) delay_ms(50);
            while (Key_Any()) delay_ms(10);
            OLED_Clear();
        }

        // ===== 测量与显示 =====
        if (measuring) {
            uint16_t volt_mv = Get_Voltage_mV();
            uint32_t freq = FREQ_Get();
            uint8_t quality = JudgeQuality(volt_mv, freq);
            total_cnt++;
            if (quality == 0) good_cnt++;
            else if (quality == 1) pass_cnt++;
            else bad_cnt++;

            if (display_mode == 0) {
                OLED_ShowNormalMode(volt_mv, freq, quality);
            } else {
                // 停止状态下 display_mode 切换不影响测量时的显示，这里测量时也支持统计模式（符合题目要求）
                OLED_ShowStatMode();
            }
            LightLED(quality);
        } else {
            // 停止状态：根据 display_mode 显示内容
            if (display_mode == 0) {
                OLED_Clear();
                OLED_ShowString(1, 1, "Stopped");
                OLED_ShowString(2, 1, "Press Start");
            } else {
                OLED_ShowStatMode();
            }
        }
        delay_ms(200); // 刷新间隔
    }
}
