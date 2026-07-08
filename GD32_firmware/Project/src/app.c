#include "gd32f3x0.h"
#include "drv_usb_hw.h"
#include "cdc_acm_core.h"

usb_core_driver cdc_acm;

/* 【终极双向缓冲】：分配足够的内存应对大文件流 */
#define BUF_SIZE   2048

/* 通道 B：电脑 USB -> ESP32 串口 */
uint8_t pc_to_esp_buf[BUF_SIZE];
volatile uint32_t pc_to_esp_w = 0;
uint32_t pc_to_esp_r = 0;

/* 通道 A：ESP32 串口 -> 电脑 USB */
uint8_t esp_to_pc_buf[BUF_SIZE];
volatile uint32_t esp_to_pc_w = 0;
uint32_t esp_to_pc_r = 0;

static uint8_t esp32_last_dtr = 0U;
static uint8_t esp32_last_rts = 0U;
#define ESP32_IO0_PIN   GPIO_PIN_0 // 你自己板子上的配置
#define ESP32_EN_PIN    GPIO_PIN_4

void esp32_hardware_init(void);

int main(void)
{
    usb_rcu_config();
    usb_timer_init();
    esp32_hardware_init();
    usbd_init(&cdc_acm, &cdc_desc, &cdc_class);
    usb_intr_config();

    while(1) {
        if(USBD_CONFIGURED == cdc_acm.dev.cur_status) {
            usb_cdc_handler *cdc = (usb_cdc_handler *)cdc_acm.dev.class_data[CDC_COM_INTERFACE];

            /* ========================================================
               1. 真正非阻塞的下发：把电脑发来的数据，异步喂给串口
               ======================================================== */
            if (pc_to_esp_r < pc_to_esp_w) {
                // 如果串口发送寄存器空闲，丢一个字节进去
                if (SET == usart_flag_get(USART1, USART_FLAG_TBE)) {
                    usart_data_transmit(USART1, pc_to_esp_buf[pc_to_esp_r % BUF_SIZE]);
                    pc_to_esp_r++;
                }
            }

            /* ========================================================
               2. 智能流控接收：缓存充足时才允许 USB 接收新数据
               ======================================================== */
            if (cdc->packet_receive == 1U) {
                // 如果缓存剩余空间足够容纳一个最大 USB 包 (64 字节)
                if ((pc_to_esp_w - pc_to_esp_r) <= (BUF_SIZE - 64)) {
                    cdc_acm_data_receive(&cdc_acm); // 重新开启 USB 接收
                }
            }

            /* ========================================================
               3. 高效上传：把 ESP32 发来的数据，严格切片上传给电脑
               ======================================================== */
            if (cdc->packet_sent == 1U) {
                if (esp_to_pc_w > esp_to_pc_r) {
                    uint32_t to_send = esp_to_pc_w - esp_to_pc_r;
                    
                    // 严格卡死在 USB 最大包限制，兼容所有上位机系统的 ZLP(零包) 机制
                    if (to_send > 64) {
                        to_send = 64;
                    }
                    
                    uint32_t read_idx = esp_to_pc_r % BUF_SIZE;
                    
                    // 防止环形数组物理越界读取
                    if (read_idx + to_send > BUF_SIZE) {
                        to_send = BUF_SIZE - read_idx;
                    }

                    extern void cdc_acm_custom_send(usb_dev *udev, uint8_t *buffer, uint32_t length);
                    cdc_acm_custom_send(&cdc_acm, &esp_to_pc_buf[read_idx], to_send);
                    esp_to_pc_r += to_send;
                }
            }
        }
    }
}

/* 电脑发数据过来时，USB驱动触发此回调 */
void cdc_acm_data_receive_callback(uint8_t *buffer, uint32_t length)
{
    // 【终极解药】：绝对不能在这里用 while 死等串口发送，拿到数据立刻塞进缓存就跑！
    for (uint32_t i = 0; i < length; i++) {
        pc_to_esp_buf[pc_to_esp_w % BUF_SIZE] = buffer[i];
        pc_to_esp_w++;
    }
}



/* 根据上位机 DTR/RTS 信号驱动 ESP32 的复位与下载模式 */

void esp32_reset_control(uint8_t dtr, uint8_t rts)
{
    if((dtr != esp32_last_dtr) || (rts != esp32_last_rts)) {
        
        if((1U == dtr) && (0U == rts)) {
            /* ======= 进入下载模式 ======= */
            // 1. 先把 IO0 牢牢拉低
            gpio_bit_reset(GPIOA, ESP32_IO0_PIN);  
            
            // 2. 极其微小的延时（几个微秒即可），确保电容彻底放电
            for(volatile uint32_t i = 0; i < 2000; i++) { __NOP(); } 
            
            // 3. 此时再拉高 EN。ESP32 瞬间醒来，稳稳采样到 IO0 为低电平
            gpio_bit_set(GPIOA, ESP32_EN_PIN);       
        } 
        else if((0U == dtr) && (1U == rts)) {
            /* ======= 正常硬件复位 ======= */
            gpio_bit_set(GPIOA, ESP32_IO0_PIN);  // IO0 必须为高
            gpio_bit_reset(GPIOA, ESP32_EN_PIN); // 拉低 EN 复位
        } 
        else if((0U == dtr) && (0U == rts)) {
            /* ======= 正常运行释放状态 ======= */
            gpio_bit_set(GPIOA, ESP32_EN_PIN);       
            gpio_bit_set(GPIOA, ESP32_IO0_PIN);    
        }
    }
    esp32_last_dtr = dtr;
    esp32_last_rts = rts;
}
//void esp32_reset_control(uint8_t dtr, uint8_t rts)
//{
//    // 只在状态变化时触发
//    if((dtr != esp32_last_dtr) || (rts != esp32_last_rts)) {
//        if((1U == dtr) && (0U == rts)) { // 下载模式触发
//					// 进入下载模式前，顺手清空双向环形队列
//						pc_to_esp_w = 0; pc_to_esp_r = 0;
//						esp_to_pc_w = 0; esp_to_pc_r = 0;
//            // 1. 初始状态：EN 和 IO0 同时下拉
//            gpio_bit_reset(GPIOA, ESP32_IO0_PIN);
//            gpio_bit_reset(GPIOA, ESP32_EN_PIN);
//            
//            // 2. 模拟官方时序：持续 100ms 下拉
//            for(volatile uint32_t i = 0; i < 200000; i++) { __NOP(); }
//            
//            // 3. 模拟“微上拉”：瞬间释放 IO0 和 EN
//            gpio_bit_set(GPIOA, ESP32_IO0_PIN);
//            gpio_bit_set(GPIOA, ESP32_EN_PIN);
//            for(volatile uint32_t i = 0; i < 5000; i++) { __NOP(); } // 几毫秒的上拉
//            
//            // 4. 再次下拉，锁定进入下载模式
//            gpio_bit_reset(GPIOA, ESP32_IO0_PIN);
//            gpio_bit_reset(GPIOA, ESP32_EN_PIN);
//            for(volatile uint32_t i = 0; i < 100000; i++) { __NOP(); } // 50ms 左右的持续锁定
//            
//            // 5. 释放 EN 让 ESP32 启动，但保持 IO0 为低 (因为下载模式需要 IO0 在启动瞬间为低)
//            gpio_bit_set(GPIOA, ESP32_EN_PIN);
//            
//            // 6. 等待一会后释放 IO0，确保芯片已经进入下载模式
//            for(volatile uint32_t i = 0; i < 200000; i++) { __NOP(); }
//            gpio_bit_set(GPIOA, ESP32_IO0_PIN);
//        }
//        else if((0U == dtr) && (1U == rts)) { // 正常复位
//            gpio_bit_reset(GPIOA, ESP32_EN_PIN);
//            gpio_bit_set(GPIOA, ESP32_IO0_PIN); // 正常复位时 IO0 必须保持高
//            for(volatile uint32_t i = 0; i < 100000; i++) { __NOP(); }
//            gpio_bit_set(GPIOA, ESP32_EN_PIN);
//        }
//        // 其他情况恢复默认高电平（由外部上拉电阻保证）
//        else {
//            gpio_bit_set(GPIOA, ESP32_IO0_PIN);
//            gpio_bit_set(GPIOA, ESP32_EN_PIN);
//        }
//    }
//    esp32_last_dtr = dtr;
//    esp32_last_rts = rts;
//}
//void esp32_reset_control(uint8_t dtr, uint8_t rts)

//{

//    if((dtr != esp32_last_dtr) || (rts != esp32_last_rts)) {
//        if((0U == dtr) && (0U == rts)) {
//            // 待机常态
//            gpio_bit_set(GPIOA, ESP32_EN_PIN);       
//            gpio_bit_set(GPIOA, ESP32_IO0_PIN);    
//        } 
//        else if((1U == dtr) && (0U == rts)) {
//            // 【重点优化：解决重试 3 次才能下载的问题】
//            // 1. 先死死拉低 IO0，准备进入下载模式
//            gpio_bit_reset(GPIOA, ESP32_IO0_PIN);  
//            
//            // 2. 故意延时一小会儿 (约几百微秒)，确保电路板上的滤波电容彻底放电，IO0 完全降到 0V
//            for(volatile uint32_t i = 0; i < 10000; i++) { __NOP(); } 
//            // 3. 再拉高 EN。此时 ESP32 醒来采样 IO0，绝对是稳稳的低电平！百分百进入下载模式！
//            gpio_bit_set(GPIOA, ESP32_EN_PIN);       
//        } 
//        else if((0U == dtr) && (1U == rts)) {
//            // 正常复位状态！EN拉低，此时 IO0 必须强制设为高电平
//            gpio_bit_set(GPIOA, ESP32_IO0_PIN);  
//            gpio_bit_reset(GPIOA, ESP32_EN_PIN);     
//        } 
//        else {
//            // dtr=1, rts=1
//            gpio_bit_set(GPIOA, ESP32_IO0_PIN); 
//            gpio_bit_set(GPIOA, ESP32_EN_PIN);       
//        }
//    }
//    esp32_last_dtr = dtr;
//    esp32_last_rts = rts;
//}
/* 初始化连接 ESP32 的引脚：PA2(TX) 和 PA3(RX) */
/* 初始化连接 ESP32 的引脚 */
void esp32_hardware_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);

    // 【关键破局点 1：先设高电平，再设输出模式】
    // 这样在开启输出的瞬间，引脚直接是高电平，绝不会出现把 IO0 拉低的毛刺
    gpio_bit_set(GPIOA, ESP32_IO0_PIN | ESP32_EN_PIN);

    // 配置 IO0 (PA0, PA1) 和 EN (PA4) 为推挽输出
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ESP32_IO0_PIN);
  //  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, ESP32_IO0_PIN);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, ESP32_IO0_PIN);
	 // gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, ESP32_IO0_PIN);
	
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ESP32_EN_PIN);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, ESP32_EN_PIN);

	 
    // 【关键破局点 2：开机主动复位，强制 ESP32 正常启动】
    // 因为电池上电瞬间序不可控，GD32 启动后主动发一次“正常开机”时序
    gpio_bit_set(GPIOA, ESP32_IO0_PIN);   // 确保 IO0 保持高电平
    gpio_bit_reset(GPIOA, ESP32_EN_PIN);  // 拉低 EN，让 ESP32 硬件复位
    
    for(volatile uint32_t i = 0; i < 100000; i++) { __NOP(); } // 等待 ESP32 完全断电
    
    gpio_bit_set(GPIOA, ESP32_EN_PIN);    // 拉高 EN，此时 ESP32 读取 IO0 为高，正常进入游戏/程序！
    for(volatile uint32_t i = 0; i < 100000; i++) { __NOP(); } // 给 ESP32 留出启动时间

    // --- 后面是原有的串口初始化代码 ---
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2); 
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3); 

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    usart_deinit(USART1);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_baudrate_set(USART1, 115200U); 
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    
    nvic_irq_enable(USART1_IRQn, 1, 1);
    usart_interrupt_enable(USART1, USART_INT_RBNE); 
    usart_enable(USART1);
}

/* 串口接收中断：只管存入缓存，彻底解耦 */
void USART1_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)){
        uint8_t ch = (uint8_t)usart_data_receive(USART1);
        esp_to_pc_buf[esp_to_pc_w % BUF_SIZE] = ch;
        esp_to_pc_w++; 
    }
}
