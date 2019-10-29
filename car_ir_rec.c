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
ir_codes decode_package(void);
char * itoa(int val, int base);
void dbg_print(char * st);
void gen_pulses(uint16_t number);

uint64_t millis(void);
void delay(uint64_t duration);

#define PAGE_15_ADDR            0x08003C00
#define CODES_SIGNATURE         0x55AA
#define CODES_MAX               25
#define PULSES_MAX              99
// duration connected state of Potentiometer, mS
#define KEY_PULSE               150

#define SNDBUF_SZ               64

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

uint8_t teachingFlag = 0;               //
uint8_t teach_counter = 0;


// Storage for our monotonic system clock.
// Note that it needs to be volatile since we're modifying it from an interrupt.
static volatile uint64_t _millis = 0;

uint64_t millis(void) {
    return _millis;
}

// This is our interrupt handler for the systick reload interrupt.
// The full list of interrupt services routines that can be implemented is
// listed in libopencm3/include/libopencm3/stm32/f0/nvic.h
void sys_tick_handler(void) {
    // Increment our monotonic clock
    _millis++;
}

/**
 * Delay for a real number of milliseconds
 */
void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until) ;
}

void dbg_print(char * st)
{
    strcpy(sndbuf, st);
    strcat(sndbuf, "\0");
    dma_write();    
}

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

    dbg_print("***** Started! *****\n");
    
    receiveComplete = 0;
    startflag = 0;
    gpio_clear(GPIOB, GPIO1);     // break Resistor out
    gpio_set(GPIOA, GPIO4);     // board LED

    // FLASH read
    uint32_t addr = PAGE_15_ADDR;
    for (uint8_t i = 0; i < (CODES_MAX); i++)
    {
        butt_codes[i].adrcode = (*(uint16_t *)(addr));
        addr += 2;
    }

    if (butt_codes[0].adrcode != CODES_SIGNATURE) {
        // saved array not exists -> Set Teaching mode
        delay(200);
        dbg_print("\n** No saved codes - switch to learning! **\n");
        teachingFlag = 1;
        teach_counter = 1;
        led_blink(6);
    }

    timer_enable_irq(TIM1, TIM_DIER_UIE);
    // Set Potentiometer to 0 - count down 99 steps
    timer_disable_counter(TIM1);
    gpio_clear(GPIOA, GPIO6);
    gen_pulses(99);

    while (1)
    {
        if (receiveComplete)  // packet received
        {
            ircode = decode_package();    // 0xFF - broken IR package (code != ~code)
            if (ircode.adcod[1] < 0xFF)
            {
                if (teachingFlag)
                {
                    if (teach_counter < CODES_MAX)
                    {
                        butt_codes[teach_counter].adrcode = ircode.adrcode;
                        teach_counter++;
                    }
                    if (teach_counter == CODES_MAX)
                    {
                        delay(200);
                        dbg_print("\n** End lerning - switch to work! **\n");
                        led_blink(4); // 4 - save codes and switch to work mode
                        save_codes_to_flash();
                        teachingFlag = 0;
                        teach_counter = 1;
                    }
                    else
                    {
                        led_blink(2); // 2 - expect next code
                    }
                }
                else  // work mode
                {
                    gpio_clear(GPIOA, GPIO4);
                          
                    int8_t npulses = get_pulses(ircode);
                    while (flag_pulse)
                    {
                        delay(4);
                    }
                    uint8_t pulses = 0;
                    int8_t d = curr_count - npulses;
                    if ( d != 0)
                    {
                        if (d > 0)    // count down
                        {
                            gpio_clear(GPIOA, GPIO6);
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
                        else  // count up
                        {
                            gpio_set(GPIOA, GPIO6);
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
                        gen_pulses(pulses - 1);
                        receiveComplete = 0;
                        startflag = 0;
                        while (flag_pulse)
                        {
                            delay(4);
                        }
                        // connect Resistor out
                        gpio_set(GPIOB, GPIO1);
                        delay(KEY_PULSE);
                        // break Resistor out
                        gpio_clear(GPIOB, GPIO1);
                    }
                    gpio_set(GPIOA, GPIO4);
                }
            }
            receiveComplete = 0;
            startflag = 0;
            //dma_write(sndbuf, strlen(sndbuf));
        }     // if (receiveComplete)
        else
        {
            // Check if Learning mode button is pressed
            if (gpio_get(GPIOA, GPIO5) == 0)
            {
                if (!teachingFlag)
                {
                    // Set Learning mode - 6 blinks
                    delay(200);
                    dbg_print("\n** Button pressed - switch to learning! **\n");
                    teachingFlag = 1;
                    teach_counter = 1;
                    led_blink(6);
                }
            }
        }
    }

    return 0;
}

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
    return res;
}

ir_codes decode_package(void)
{
    uint8_t i, j;
    // idx starts with 1 to indicate that
    // the synchronization header time is not handled
    uint8_t indx = 1;
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
            //remote_code[i] <<= 1;
            remote_code[i] >>= 1;
            remote_code[i] |= temp;
            indx++;
        }
    }

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


// ------------------------------------------------------------------------------
static void clock_setup(void) {
    // First, let's ensure that our clock is running off the high-speed internal
    // oscillator (HSI) at 48MHz.
    //rcc_clock_setup_in_hsi_out_48mhz();
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
    // In order to trigger an interrupt every millisecond, we can set the reload
    // value to be the speed of the processor / 1000 -1
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    // Enable interrupts from the system tick clock
    systick_interrupt_enable();
    // Enable the system tick counter
    systick_counter_enable();
}

static void gpio_setup(void) {
    //  Board Led
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    // Resistor enable
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    // Resistor Inc -> tim1_setup (TIM1, CH1)
    // Resistor Up/Dn
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    // Button Lern
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO5);

    // VS1838 input
    nvic_enable_irq(NVIC_EXTI2_3_IRQ);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO3);   // PULLUP->NONE ?
    exti_select_source(EXTI3, GPIOA);
    exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI3);
}


static void usart_setup(void) {
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);

    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    /* Setup USART1 TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    /* Setup GPIO pins for USART1 receive */
    //gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    /* Setup USART1 RX pin as alternate function. */
    //gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

        /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX);

    /* Enable USART1 Receive interrupt. */
    //USART_CR1(USART1) |= USART_CR1_RXNEIE;
    //usart_enable_rx_interrupt(USART1);
    /* Finally enable the USART. */
    usart_enable(USART1);

}

//static void dma_write(char *data, int size)
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


// ------------------------------------------------------------------------------
static void tim1_setup(void) {
    rcc_periph_clock_enable(RCC_TIM1);

    nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
    nvic_set_priority(NVIC_TIM1_BRK_UP_TRG_COM_IRQ, 1);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO7);

    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 48 - 1);
    timer_set_period(TIM1, 7);  // ARR         - Period
    timer_set_oc_value(TIM1, TIM_OC1, 4);   // - Pulse
    timer_set_repetition_counter(TIM1, 0);
    timer_generate_event(TIM1, TIM_EGR_UG);
    timer_one_shot_mode(TIM1);

    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);

    timer_disable_preload(TIM1);
    

//    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_set_oc_polarity_low(TIM1, TIM_OC1N);
    timer_set_oc_idle_state_set(TIM1, TIM_OC1N);

    timer_enable_oc_output(TIM1, TIM_OC1N);
    timer_enable_break_main_output(TIM1);

    //timer_enable_break_main_output(TIM1);

    //timer_enable_irq(TIM1, TIM_DIER_UIE);
    //timer_enable_counter(TIM1);
}

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

// ------------------ isr ---------------------------

void dma1_channel2_3_dma2_channel1_2_isr(void)
{
    if ((DMA1_ISR &DMA_ISR_TCIF2) != 0)
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF2;
    }
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
    usart_disable_tx_dma(USART1);
    dma_disable_channel(DMA1, DMA_CHANNEL2);
}

void tim1_brk_up_trg_com_isr(void)
{
    if (timer_get_flag(TIM1, TIM_SR_UIF))
    {
        timer_clear_flag(TIM1, TIM_SR_UIF);
        flag_pulse = 0;
        //timer_disable_counter(TIM1);
    }
}

void tim2_isr(void)
{
    //TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {
        timer_clear_flag(TIM2, TIM_SR_UIF);
        ucTim2Flag++;
    }
}

void exti2_3_isr(void)
{
    if (exti_get_flag_status(EXTI3))
    {
        exti_reset_request(EXTI3);

        if(startflag)
        {
            uint16_t ir_time = ucTim2Flag;
            if(ir_time < 150 && ir_time >= 50 ) // < 15mS and > 5mS - Received SyncHeader
            {
                idx=0;              // Array subscript zeroing
            }

            irdata[idx] = ir_time;  // Get Count Time
            ucTim2Flag = 0;         // Zero count time for next count
            idx++;                  // Received a data, index plus 1
            
            if(idx==33)             // received 33 bits - 32 bits and a sync header
            {
                idx=0;
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
