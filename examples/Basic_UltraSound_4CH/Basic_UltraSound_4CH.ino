#include "SensEdu.h"
#include "SineLUT.h"

/* -------------------------------------------------------------------------- */
/*                                  Settings                                  */
/* -------------------------------------------------------------------------- */

/* DAC */
// lut settings are in SineLUT.h
#define DAC_SINE_FREQ     	32000                           // 32kHz
#define DAC_SAMPLE_RATE     DAC_SINE_FREQ * sine_lut_size   // 64 samples per one sine cycle

DAC_Channel* dac_ch = DAC_CH1;
SensEdu_DAC_Settings dac_settings = {
    .dac_channel = dac_ch, 
    .sampling_freq = DAC_SAMPLE_RATE,
    .mem_address = (uint16_t*)sine_lut,
    .mem_size = sine_lut_size,
    .wave_mode = SENSEDU_DAC_MODE_BURST_WAVE,
    .burst_num = dac_cycle_num
};

/* ADC */
const uint16_t mic_data_size = 16*128*2; // must be multiple of 16 for 16bit
__attribute__((aligned(__SCB_DCACHE_LINE_SIZE))) uint16_t mic12_data[mic_data_size]; // cache aligned
__attribute__((aligned(__SCB_DCACHE_LINE_SIZE))) uint16_t mic34_data[mic_data_size]; // cache aligned


ADC_TypeDef* adc1 = ADC1;
ADC_TypeDef* adc2 = ADC2;
const uint8_t mic_num = 2;
uint8_t mic12_pins[mic_num] = {A5, A4};
uint8_t mic34_pins[mic_num] = {A1, A6};

SensEdu_ADC_Settings adc1_settings = {
    .adc = adc1,
    .pins = mic12_pins,
    .pin_num = mic_num,

    .conv_mode = SENSEDU_ADC_MODE_CONT_TIM_TRIGGERED,
    .sampling_freq = 250000,
    
    .dma_mode = SENSEDU_ADC_DMA_CONNECT,
    .mem_address = (uint16_t*)mic12_data,
    .mem_size = mic_data_size
};

SensEdu_ADC_Settings adc2_settings = {
    .adc = adc2,
    .pins = mic34_pins,
    .pin_num = mic_num,

    .conv_mode = SENSEDU_ADC_MODE_CONT_TIM_TRIGGERED,
    .sampling_freq = 250000,
    
    .dma_mode = SENSEDU_ADC_DMA_CONNECT,
    .mem_address = (uint16_t*)mic34_data,
    .mem_size = mic_data_size
};

/* errors */
uint32_t lib_error = 0;
uint8_t error_led = D86;

/* -------------------------------------------------------------------------- */
/*                                    Setup                                   */
/* -------------------------------------------------------------------------- */
void setup() {

    Serial.begin(115200);

    SensEdu_DAC_Init(&dac_settings);

    SensEdu_ADC_ShortA4toA9();
    SensEdu_ADC_Init(&adc1_settings);
    SensEdu_ADC_Init(&adc2_settings);
    SensEdu_ADC_Enable(adc1);
    SensEdu_ADC_Enable(adc2);

    pinMode(error_led, OUTPUT);
    digitalWrite(error_led, HIGH);

    lib_error = SensEdu_GetError();
    while (lib_error != 0) {
        handle_error();
    }
}

/* -------------------------------------------------------------------------- */
/*                                    Loop                                    */
/* -------------------------------------------------------------------------- */
void loop() {
    
    // Measurement is initiated by the signal from computing device
    static char serial_buf = 0;
    
    while (1) {
        while (Serial.available() == 0); // Wait for a signal
        serial_buf = Serial.read();

        if (serial_buf == 't') {
            // expected 't' symbol (trigger)
            break;
        }
    }

    // start dac->adc sequence
    SensEdu_DAC_Enable(dac_ch);
    while(!SensEdu_DAC_GetBurstCompleteFlag(dac_ch));
    SensEdu_DAC_ClearBurstCompleteFlag(dac_ch);
    SensEdu_ADC_Start(adc1);
    SensEdu_ADC_Start(adc2);
    
    // wait for the data and send it
    while(!SensEdu_ADC_GetTransferStatus(adc1));
    SensEdu_ADC_ClearTransferStatus(adc1);

    while(!SensEdu_ADC_GetTransferStatus(adc2));
    SensEdu_ADC_ClearTransferStatus(adc2);

    serial_send_array((const uint8_t *) & mic12_data, mic_data_size << 1);
    serial_send_array((const uint8_t *) & mic34_data, mic_data_size << 1);

    // check errors
    lib_error = SensEdu_GetError();
    while (lib_error != 0) {
        handle_error();
    }
}

/* -------------------------------------------------------------------------- */
/*                                  Functions                                 */
/* -------------------------------------------------------------------------- */
void handle_error() {
    // serial is taken by matlab, use LED as indication
    digitalWrite(error_led, LOW);
}

// send serial data in 32 byte chunks
void serial_send_array(const uint8_t* data, size_t size) {
    const size_t chunk_size = 32;
	for (uint32_t i = 0; i < size/chunk_size; i++) {
		Serial.write(data + chunk_size * i, chunk_size);
	}
}
