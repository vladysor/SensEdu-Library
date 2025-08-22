#include "timer.h"

/* -------------------------------------------------------------------------- */
/*                                  Variables                                 */
/* -------------------------------------------------------------------------- */

#define TIM_CLK         240000000
#define DAC_PRESC_FREQ  120000000       // Desired freq after prescaler for DAC
#define ADC_PRESC_FREQ  60000000        // Desired freq after prescaler for ADC


static TIMER_ERROR error = TIMER_ERROR_NO_ERRORS;
static volatile uint8_t delay_flag = 0;

/* -------------------------------------------------------------------------- */
/*                                Declarations                                */
/* -------------------------------------------------------------------------- */
void tim1_adc1_init(void);
void tim2_delay_init(void);
void tim4_dac1_init(void);
void tim8_pwm_init(void);

/* -------------------------------------------------------------------------- */
/*                              Public Functions                              */
/* -------------------------------------------------------------------------- */
void SensEdu_TIMER_DelayInit(void) {
    tim2_delay_init();
}

void SensEdu_TIMER_Delay_us(uint32_t delay_us) {
    delay_flag = 1;
    WRITE_REG(TIM2->ARR, delay_us);
    WRITE_REG(TIM2->CNT, 0U);
    SET_BIT(TIM2->CR1, TIM_CR1_CEN);
    while(delay_flag == 1);
}

TIMER_ERROR TIMER_GetError(void) {
    return error;
}

void TIMER_ADC1Init(void) {
    tim1_adc1_init();
}

void TIMER_DAC1Init(uint32_t freq) {
    tim4_dac1_init();
    TIMER_DAC1SetFreq(freq);
}

void TIMER_ADC1Enable(void) {
    WRITE_REG(TIM1->CNT, 0U);
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

void TIMER_DAC1Enable(void) {
    WRITE_REG(TIM4->CNT, 0U);
    SET_BIT(TIM4->CR1, TIM_CR1_CEN);
}

void TIMER_ADC1Disable(void) {
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
}

void TIMER_DAC1Disable(void) {
    CLEAR_BIT(TIM4->CR1, TIM_CR1_CEN);
}

void TIMER_ADC1SetFreq(uint32_t freq) {
    if (freq < 0 || freq > (TIM_CLK/2)) {
        //error = TIMER_ERROR_TIM1_BAD_SET_FREQUENCY;
        return;
    }
    // Calculate the timer values
    uint32_t psc, arr;
    calculate_timer_values(freq, &psc, &arr);

    // Set the timer registers
    WRITE_REG(TIM1->PSC, psc);
    WRITE_REG(TIM1->ARR, arr);
}

void TIMER_DAC1SetFreq(uint32_t freq) {
    if (freq < 0 || freq > (DAC_PRESC_FREQ/2)) {
        // minimum ARR is 1
        error = TIMER_ERROR_TIM4_BAD_SET_FREQUENCY;
        return;
    }
    float periodf = (float)DAC_PRESC_FREQ/freq;
    uint32_t period = (uint32_t)lroundf(periodf); // period = ARR + 1
    WRITE_REG(TIM4->ARR, period - 1U);
}

void TIMER_PWMInit(void) {
    tim8_pwm_init();
}

void TIMER_PWMEnable(void) {
    SET_BIT(TIM8->EGR, TIM_EGR_UG);
    SET_BIT(TIM8->CR1, TIM_CR1_CEN);
}

void TIMER_PWMDisable(void) {
    CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);
    SET_BIT(TIM8->EGR, TIM_EGR_UG);
}

void TIMER_PWMSetFreq(uint32_t freq) {
    uint32_t arr = 0;
    uint16_t psc = 0;

    // try PSC = 0
    arr = TIM_CLK/freq - 1;
    if (arr > 0 && arr <= 65535) {
        WRITE_REG(TIM8->PSC, 0U);
        WRITE_REG(TIM8->ARR, arr);
        return;
    }
    if (arr < 1) {
        error = TIMER_ERROR_TIM8_CRITICAL_FREQ_CALCULATION_BUG;
        return;
    }

    // compute minimal PSC
    psc = TIM_CLK/(freq*65536) - 1;
    if (psc > 65535) {
        error = TIMER_ERROR_TIM8_CRITICAL_FREQ_CALCULATION_BUG;
        return;
    }
    arr = TIM_CLK/(freq * (psc + 1)) - 1;
    if (arr > 65535) {
        psc += 1; // fix for rounding errors
        arr = TIM_CLK/(freq * (psc + 1)) - 1;
        if (arr > 65535) {
            error = TIMER_ERROR_TIM8_CRITICAL_FREQ_CALCULATION_BUG;
            return;
        }
    }
    WRITE_REG(TIM8->PSC, psc);
    WRITE_REG(TIM8->ARR, arr);
}

void TIMER_PWMSetDutyCycle(uint8_t channel, uint8_t duty_cycle) {
    uint32_t arr = READ_REG(TIM8->ARR) + 1;
    uint32_t ccr = (arr*(100 - duty_cycle))/100;
    switch(channel) {
        case 1:
            WRITE_REG(TIM8->CCR1, ccr);
            break;
        case 2:
            WRITE_REG(TIM8->CCR2, ccr);
            break;
        case 3:
            WRITE_REG(TIM8->CCR3, ccr);
            break;
        case 4:
            WRITE_REG(TIM8->CCR4, ccr);
            break;
        default:
            error = TIMER_ERROR_TIM8_WRONG_DUTY_CHANNEL;
            break;
    }
    
}

/* -------------------------------------------------------------------------- */
/*                              Private Functions                             */
/* -------------------------------------------------------------------------- */
void calculate_timer_values(uint32_t freq, uint32_t *PSC, uint32_t *ARR) {
    // uint32_t PSC_MAX = 65535; // 2^16-1
    // uint32_t ARR_MAX = 65535; 
    uint32_t psc, arr; 
    float min_error = 9999999; // very large number
    uint32_t best_psc = 0, best_arr = 1;
    
    // go through psc values
    for (psc = 1; psc <= 65535; psc++) {
        // calculate arr value from the formula
        arr = (uint32_t)((float)TIM_CLK / (freq * (psc + 1)) - 1);

        // check if it goes beyond the max limit
        if (arr > 65535) continue;

        // calculate the error between the desired frequency and frequency using the current parameters
        float error = fabsf(freq - (float)TIM_CLK / ((psc + 1) * (arr + 1)));

        // update best parameters if the error is smaller
        if (error < min_error) {
            min_error = error;
            best_psc = psc;
            best_arr = arr;
        }
    }
    // assign the values 
    *PSC = best_psc;
    *ARR = best_arr;
}

void tim1_adc1_init(void) {
    // Clock
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);

    // Frequency settings
    WRITE_REG(TIM1->PSC, 2U - 1U); // default
    WRITE_REG(TIM1->ARR, 60U - 1U); // default

    // update event is trigger output
    MODIFY_REG(TIM1->CR2, TIM_CR2_MMS, 0b010 << TIM_CR2_MMS_Pos);
}

void tim2_delay_init(void) {
    // Clock
    SET_BIT(RCC->APB1LENR, RCC_APB1LENR_TIM2EN);

    // Frequency settings
    WRITE_REG(TIM2->PSC, 240U-1U); // timer clock = 240MHz

    // timer turns off after one cycle
    SET_BIT(TIM2->CR1, TIM_CR1_OPM);

    // interrupts
    SET_BIT(TIM2->DIER, TIM_DIER_UIE); // update event
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void tim4_dac1_init(void) {
    // Clock
    SET_BIT(RCC->APB1LENR, RCC_APB1LENR_TIM4EN);

    // Frequency settings
    WRITE_REG(TIM4->PSC, (TIM_CLK/DAC_PRESC_FREQ) - 1U); // default
    WRITE_REG(TIM4->ARR, 120U - 1U); // default

    // update event is trigger output
    MODIFY_REG(TIM4->CR2, TIM_CR2_MMS, 0b010 << TIM_CR2_MMS_Pos);
}

void tim8_pwm_init(void) {
    if (READ_BIT(TIM8->CR1, TIM_CR1_CEN)) {
        error = TIMER_ERROR_TIM8_INIT_WHILE_RUNNING;
        return;
    }

    // Clock
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);

    // Select PWM mode
    MODIFY_REG(TIM8->CCMR1, TIM_CCMR1_OC1M, 0b0111 << TIM_CCMR1_OC1M_Pos);
    MODIFY_REG(TIM8->CCMR1, TIM_CCMR1_OC2M, 0b0111 << TIM_CCMR1_OC2M_Pos);
    MODIFY_REG(TIM8->CCMR2, TIM_CCMR2_OC3M, 0b0111 << TIM_CCMR2_OC3M_Pos);
    MODIFY_REG(TIM8->CCMR2, TIM_CCMR2_OC4M, 0b0111 << TIM_CCMR2_OC4M_Pos);

    // Preload updates
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC1PE);
    SET_BIT(TIM8->CCMR1, TIM_CCMR1_OC2PE);
    SET_BIT(TIM8->CCMR2, TIM_CCMR2_OC3PE);
    SET_BIT(TIM8->CCMR2, TIM_CCMR2_OC4PE);
    SET_BIT(TIM8->CR1, TIM_CR1_ARPE);

    // Default frequency settings
    WRITE_REG(TIM8->PSC, 1U - 1U);
    WRITE_REG(TIM8->ARR, 1000U - 1U);

    // Default duty cycle settings
    WRITE_REG(TIM8->CCR1, 1000U - 1U);
    WRITE_REG(TIM8->CCR2, 750U - 1U);
    WRITE_REG(TIM8->CCR3, 500U - 1U);
    WRITE_REG(TIM8->CCR4, 250U - 1U);

    // Clear Counter
    SET_BIT(TIM8->EGR, TIM_EGR_UG);
    WRITE_REG(TIM8->CNT, 0U);

    // Enable Capture/Compare
    SET_BIT(TIM8->CCER, TIM_CCER_CC1E);
    SET_BIT(TIM8->CCER, TIM_CCER_CC2E);
    SET_BIT(TIM8->CCER, TIM_CCER_CC3E);
    SET_BIT(TIM8->CCER, TIM_CCER_CC4E);

    // Main Output Enable
    SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);
}

/* -------------------------------------------------------------------------- */
/*                                 Interrupts                                 */
/* -------------------------------------------------------------------------- */
void TIM2_IRQHandler(void) { 
    if (READ_BIT(TIM2->SR, TIM_SR_UIF)) {
        CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
        delay_flag = 0;
    }
}