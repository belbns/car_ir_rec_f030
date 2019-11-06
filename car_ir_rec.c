/*
 * Author: Nikolay Belov
 *
 * Tools: Linux Mint, gcc-arm-none-eabi, OpenOCD, LIBOPENCM3, Sublime Text.
 * 
 * The steering wheel buttons imitator based on STM32F030F4P6 demo board with using
 * digitally-controlled potentiometer X9C103P for imitating resistive matrix
 * of real steering wheel buttons.
 * The device allow imitate up to 16 buttons with 400 Ohm (4 pulses) steps.
 * Any IR remote control with NEC protocol can be used as a steering wheel buttons.
 * I'm using the cheap "Steering Wheel Remote Control For Car multimedia Player"
 * from Aliexpress (rem_ctrl.jpg).
 * The Device must be connected to KEY1 or KEY2 input of car multimedia device,
 * Android based multimedia player in my case.
 * The Lerning mode (the Learn button) allow to detect and to remember remote codes 
 * and to save them to the internal flash memory.
 * The Test button is using to manual connect the potentiometer to multimedia player
 * during setting buttons procedure.
 *
 *
 * Имитатор резистивной матрицы кнопок руля на базе демо-платы STM32F030F4P6 и цифрового
 * потенциометра X9C103P.
 * Устройство позволяет имитировать нажатие до 16 кнопок руля с шагом 400 Ом (4 импульса).
 * Я использую блок кнопок руля с инфракрасным излучателем с Алиэкспресс (11 кнопок).
 * Устройство подключается ко входу KEY1 или ко входу KEY2 радио/плеера на базе Андроид.
 * Режим обучения (кнопка Learn) позволяет распознать и запомнить во внутреннюю память
 * коды IR излучателя.
 * Кнопка Test используется при обучении радио/плеера. 
 *
 * NEC IR protocol detecting idea:
 * https://blog.csdn.net/u011303443/article/details/76945003
 * https://programmer.help/blogs/stm32-timer-for-infrared-remote-control-data-reception.html
 *
 * Waveform generation idea: General-purpose timer cookbook (AN4776, STMicroelectronics).
*/

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f0/gpio.h>
#include <libopencm3/stm32/f0/exti.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f0/dma.h>
#include <libopencm3/stm32/f0/usart.h>
#include <libopencm3/stm32/f0/rcc.h>
#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f0/timer.h>
#include <libopencm3/stm32/flash.h>

typedef union {
        uint8_t adcod[2];       // adcod[0] - address, adcod[1] - code
        uint16_t adrcode;       // both
} ir_codes;

static void clock_setup(void);
static void systick_setup(void);
static void gpio_setup(void);
static void usart_setup(void);
static void tim1_setup(void);
static void tim2_setup(void);
static void dma_write(void);

void led_blink(uint8_t cnt);
void save_codes_to_flash(void);
uint8_t get_pulses(ir_codes adcom);
void gen_pulses(uint16_t number);
ir_codes decode_package(void);
void dbg_print(char * st);
char * itoa(int val, int base);

uint64_t millis(void);
void delay(uint64_t duration);

#define PAGE_15_ADDR            0x08003C00  // upper page of the flash memory
#define CODES_SIGNATURE         0x55AA
#define CODES_MAX               17  // CODES_SIGNATURE + 16 codes 
#define PULSES_MAX              99

#define KEY_PULSE               150 // duration of the connected state of Potentiometer, mS

#define SNDBUF_SZ               64

#define DEBUG_PRINT

uint8_t flag_pulse = 0;
uint8_t flagCounting = 0;
uint16_t ucTim2Flag = 0;
// IR packet variables
uint8_t irdata[33];             //Used to record the time between two falling edges
uint8_t receiveComplete;        //Receive Complete Flag Bits
uint8_t idx;                    //Number received for indexing
uint8_t startflag;              //Indicates start of reception

char sndbuf[SNDBUF_SZ];
char tbuff[SNDBUF_SZ];

ir_codes butt_codes[CODES_MAX]; // signature + IR codes array

uint8_t learningFlag = 0;       // 1 - the learning mode
uint8_t learn_counter = 0;

uint8_t transmit = 0;

static volatile uint64_t _millis = 0;

uint64_t millis(void) {
    return _millis;
}

void sys_tick_handler(void) {
    _millis++;
}

void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until) ;
}

// send debug messages via USART
void dbg_print(char * st)
{
    delay(10);
    while (transmit != 0)
    {
        delay(4);
    }
    strcpy(sndbuf, st);
    strcat(sndbuf, "\0");
    transmit = 1;
    dma_write();    
}

/* N-pulse waveform generation for change 
    the wiper position of the potentiometer. */
void gen_pulses(uint16_t number)
{
    timer_set_period(TIM1, 7);  // ARR         - Period
    timer_set_oc_value(TIM1, TIM_OC1, 4);   // - Pulse
    timer_set_repetition_counter(TIM1, number - 1);
    timer_generate_event(TIM1, TIM_EGR_UG);
    flag_pulse = 1;
    timer_enable_counter(TIM1);
}

int main(void)
{
    clock_setup();
    systick_setup();
    gpio_setup();
    usart_setup();
    tim1_setup();
    tim2_setup();

    ir_codes ircode;
    int8_t curr_count = 0;

#ifdef DEBUG_PRINT
    dbg_print("***** Started! *****\n");
#endif    
    receiveComplete = 0;
    startflag = 0;
    gpio_clear(GPIOB, GPIO1);   // break Resistor out - to Q1
    gpio_set(GPIOA, GPIO4);     // board LED off

    // FLASH read
    uint32_t addr = PAGE_15_ADDR;
    for (uint8_t i = 0; i < (CODES_MAX); i++)
    {
        butt_codes[i].adrcode = (*(uint16_t *)(addr));
        addr += 2;
    }

    if (butt_codes[0].adrcode != CODES_SIGNATURE) {
        // saved array not exists -> set learning mode
        delay(200);
#ifdef DEBUG_PRINT
        dbg_print("\n** No saved codes - switch to learning! **\n");
#endif
        learningFlag = 1;
        learn_counter = 1;
        led_blink(6);
    }

    timer_enable_irq(TIM1, TIM_DIER_UIE);
    // Set Potentiometer to 0 Ohm - count down 99 steps
    timer_disable_counter(TIM1);
    gpio_clear(GPIOA, GPIO6);
    gen_pulses(100);
    delay(100);

    while (1)
    {
        if (receiveComplete)  // if packet received?
        {
            ircode = decode_package();
            if (ircode.adcod[1] < 0xFF) // 0xFF - broken IR package (code != ~code)
            {
                if (learningFlag)
                {
                    if (learn_counter < CODES_MAX)
                    {
                        butt_codes[learn_counter].adrcode = ircode.adrcode;
                        learn_counter++;
                    }
                    if (learn_counter == CODES_MAX)
                    {
                        delay(200);
#ifdef DEBUG_PRINT
                        dbg_print("\n** End of lerning - switch to work! **\n");
#endif
                        led_blink(4); // 4 blinks - save codes and switch to work mode
                        save_codes_to_flash();
                        learningFlag = 0;
                        learn_counter = 1;
                    }
                    else
                    {
                        led_blink(2); // 2 blinks - expect next code
                    }
                }
                else  // work mode
                {
                    gpio_clear(GPIOA, GPIO4);   // board LED ON
                          
                    int8_t npulses = get_pulses(ircode);
                    while (flag_pulse)
                    {
                        delay(4);
                    }
                    uint8_t pulses = 0;
                    int8_t d = curr_count - npulses;
                    if ( d != 0)
                    {
                        if (d > 0)
                        {
                            gpio_clear(GPIOA, GPIO6);   // count down
                            pulses = d;
                            if (pulses > curr_count)
                            {
                                curr_count = 0;
                            }
                            else
                            {
                                curr_count -= pulses;
                            }
                        }
                        else
                        {
                            gpio_set(GPIOA, GPIO6); // count up
                            pulses = -d;
                            if ((curr_count + pulses) > PULSES_MAX)
                            {
                                curr_count = PULSES_MAX;
                            }
                            else
                            {
                                curr_count += pulses;
                            }
                        }
                        gen_pulses(pulses);
                        receiveComplete = 0;
                        startflag = 0;
                        while (flag_pulse)
                        {
                            delay(4);
                        }
                    }
                    gpio_set(GPIOB, GPIO1);     // connect the Potentiometer
                    delay(KEY_PULSE);
                    gpio_clear(GPIOB, GPIO1);   // disconnect the Potentiometer
                    gpio_set(GPIOA, GPIO4); // board LED OFF
                }
            }
            receiveComplete = 0;
            startflag = 0;
        }     // if (receiveComplete)
        else
        {
            // Check if Learning mode button is pressed
            if (gpio_get(GPIOA, GPIO5) == 0)
            {
                if (!learningFlag)
                {
                    // Set Learning mode - 6 blinks
                    delay(200);
#ifdef DEBUG_PRINT
                    dbg_print("\n** Button pressed - switch to learning! **\n");
#endif
                    learningFlag = 1;
                    learn_counter = 1;
                    led_blink(6);
                }
            }
        }
    }

    return 0;
}

// Board LED blink
void led_blink(uint8_t cnt)
{
    for (uint8_t i = 0; i < cnt; i++)
    {
        gpio_clear(GPIOA, GPIO4);
        delay(200);
        gpio_set(GPIOA, GPIO4);
        delay(200);
    }                         
}

// save codes to internal flash memory
void save_codes_to_flash(void)
{
    butt_codes[0].adrcode = CODES_SIGNATURE;
    uint32_t addr = PAGE_15_ADDR;
    flash_unlock();
    flash_erase_page(PAGE_15_ADDR);
    flash_lock();

    flash_unlock();
    for (uint8_t i = 0; i < CODES_MAX; i++)
    {
        flash_program_half_word(addr, butt_codes[i].adrcode);
        addr += 2;
    }
    flash_lock();
}

/* return the number of pulses for switching from 0 Ohm
 to the destination resistanse with 500 OHm steps */
uint8_t get_pulses(ir_codes adcom)
{       
    uint8_t res = 0;

    for (uint8_t i = 1; i < CODES_MAX; i++)
    {
        if ((butt_codes[i].adcod[1] == adcom.adcod[1])
            && (butt_codes[i].adcod[0] == adcom.adcod[0]))
        {
            res = i * 4;
            break;
        }
    }
#ifdef DEBUG_PRINT
    strcpy(tbuff, " => pulses: ");
    strcat(tbuff, itoa(res, 10));
    strcat(tbuff, "\n");
    dbg_print(tbuff);
#endif
    return res;
}

// decode the received IR package
ir_codes decode_package(void)
{
    uint8_t i, j;
    uint8_t indx = 1;   // indx starts with 1 to exclude the synchronization header
    uint8_t temp = 0;
    uint8_t remote_code[4];
    ir_codes res;

    for(i = 0; i < 4; i++)
    {
        for(j = 0; j < 8; j++)
        {
            if ((irdata[indx] >=8) && (irdata[indx] < 15))   //Represents 0
            {
                temp = 0;
            }
            else if ((irdata[indx] >=18) && (irdata[indx]<25)) //Represents 1
            {
                temp = 0x80; //temp = 1;
            }
            remote_code[i] >>= 1;
            remote_code[i] |= temp;
            indx++;
        }
    }

#ifdef DEBUG_PRINT
    // debug message 
    strcpy(tbuff, "A1:0x");
    strcat(tbuff, itoa(remote_code[0], 16));
    strcat(tbuff, ", A2:0x");
    strcat(tbuff, itoa(remote_code[1], 16));
    strcat(tbuff, ", C1:0x");
    strcat(tbuff, itoa(remote_code[2], 16));
    strcat(tbuff, ", C2:0x");
    strcat(tbuff, itoa(remote_code[3], 16));
    strcat(tbuff, ", Cmd:");
    strcat(tbuff, itoa(remote_code[2], 10));
    strcat(tbuff, "\n");
    dbg_print(tbuff);
#endif
    if ((remote_code[2] | remote_code[3]) == 0xFF)
    {
        res.adcod[0] = remote_code[0];
        res.adcod[1] = remote_code[2];
    }
    else
    {
        res.adrcode = 0xFFFF;
    }
    return res;
}

/*  itoa() isn't ANSI C v.99 standard and doesn't work on my Linux gcc by default.
    Using sprintf is too much for 16 kB flash memory of STM32F030F4P6.
*/
char * itoa(int val, int base) {
    static char buf[32] = {0};
    
    if (val == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return &buf[0];
    }
    else {
        int i = 30;
    
        for(; val && i ; --i, val /= base)
        {
            buf[i] = "0123456789abcdef"[val % base];
        }
        return &buf[i+1];
    }
}


// ----------------------------------------------------------------------
static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA);
}

static void systick_setup(void) {
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void gpio_setup(void) {
    //  Board LED
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    // Potentiometer enable
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    // Potentiometer Up/Dn
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    // Learn button input
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO5);

    // IR receiver input
    nvic_enable_irq(NVIC_EXTI2_3_IRQ);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO3);
    exti_select_source(EXTI3, GPIOA);
    exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);  // falling edge interrupt
    exti_enable_request(EXTI3);
}

static void usart_setup(void) {
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    //gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    //gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX);

    usart_enable(USART1);
}

static void dma_write(void)
{
    // Using channel 2 for USART1_TX
    //Reset DMA channel
    dma_channel_reset(DMA1, DMA_CHANNEL2);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&USART1_TDR);
    dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)sndbuf);
    dma_set_number_of_data(DMA1, DMA_CHANNEL2, strlen(sndbuf));
    dma_set_read_from_memory(DMA1, DMA_CHANNEL2);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);

    dma_enable_channel(DMA1, DMA_CHANNEL2);
    
    usart_enable_tx_dma(USART1);
}

// TIM1 - waveform generation
static void tim1_setup(void) {
    rcc_periph_clock_enable(RCC_TIM1);

    nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
    nvic_set_priority(NVIC_TIM1_BRK_UP_TRG_COM_IRQ, 1);

    // Potentiometer ~Inc -> TIM1, CH1N
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO7);

    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 48 - 1);
    timer_set_period(TIM1, 7);
    timer_set_oc_value(TIM1, TIM_OC1, 4);
    timer_set_repetition_counter(TIM1, 0);
    timer_generate_event(TIM1, TIM_EGR_UG);
    timer_one_shot_mode(TIM1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
    timer_disable_preload(TIM1);
    
    timer_set_oc_polarity_low(TIM1, TIM_OC1N);  // low level pulses for ~Inc
    timer_set_oc_idle_state_set(TIM1, TIM_OC1N);

    timer_enable_oc_output(TIM1, TIM_OC1N);
    timer_enable_break_main_output(TIM1);
}

// measuring ir signal period
static void tim2_setup(void) {
    rcc_periph_clock_enable(RCC_TIM2);

    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 1);

    rcc_periph_reset_pulse(RST_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 48 - 1);
    timer_enable_preload(TIM2);
    timer_set_period(TIM2, 99);  // ARR
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    timer_enable_counter(TIM2);
    timer_generate_event(TIM2, TIM_EGR_UG); // 1-st Update Event
}

// USART DMA interrupt
void dma1_channel2_3_dma2_channel1_2_isr(void)
{
    if ((DMA1_ISR &DMA_ISR_TCIF2) != 0)
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF2;
    }
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
    usart_disable_tx_dma(USART1);
    dma_disable_channel(DMA1, DMA_CHANNEL2);
    transmit = 0;
}

// TIM1 interrupt
void tim1_brk_up_trg_com_isr(void)
{
    if (timer_get_flag(TIM1, TIM_SR_UIF))   // Update event interrupt
    {
        timer_clear_flag(TIM1, TIM_SR_UIF);
        flag_pulse = 0;
    }
}

// TIM2 interrupt
void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {
        timer_clear_flag(TIM2, TIM_SR_UIF);
        ucTim2Flag++;
    }
}

// IR receiver fallling edge interrupt
void exti2_3_isr(void)
{
    if (exti_get_flag_status(EXTI3))
    {
        exti_reset_request(EXTI3);

        if(startflag)
        {
            uint16_t ir_time = ucTim2Flag;
            if((ir_time < 150) && (ir_time >= 50)) // < 15mS and > 5mS - Received SyncHeader
            {
                idx = 0;            // Array subscript zeroing
            }

            irdata[idx] = ir_time;  // Get Count Time
            ucTim2Flag = 0;         // Zero count time for next count
            idx++;                  // Received a data, index plus 1
            
            if (idx == 33)          // received 33 bits - 32 bits and a sync header
            {
                idx = 0;
                ucTim2Flag = 0;
                receiveComplete = 1;
            }
        }
        else   // Drop Edge First Trigger
        {
            idx = 0;
            ucTim2Flag = 0;
            startflag = 1;
        }

    }
}
