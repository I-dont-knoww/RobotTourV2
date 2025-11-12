#include "drivers/Motors.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <algorithm>
#include <cstdint>

Motors::Controller::Controller(uint in1Pin, uint in2Pin)
    : m_in1Channel{ pwm_gpio_to_channel(in1Pin) },
      m_in2Channel{ pwm_gpio_to_channel(in2Pin) },
      m_in1Slice{ pwm_gpio_to_slice_num(in1Pin) },
      m_in2Slice{ pwm_gpio_to_slice_num(in2Pin) } {
    gpio_set_function(in1Pin, GPIO_FUNC_PWM);
    gpio_set_function(in2Pin, GPIO_FUNC_PWM);

    auto pwmConfig = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwmConfig, static_cast<float>(clock_get_hz(clk_sys)) /
                                          Drivers::Motors::TARGET_CLK_FREQUENCY);
    pwm_config_set_wrap(&pwmConfig, Drivers::Motors::PWM_WRAP);
    pwm_config_set_phase_correct(&pwmConfig, true);

    pwm_init(m_in1Slice, &pwmConfig, true);
    pwm_init(m_in2Slice, &pwmConfig, true);

    spin(0);
}

void Motors::Controller::spin(int power) {
    m_power = std::clamp(power, -static_cast<int>(Drivers::Motors::MAX_POWER),
                         static_cast<int>(Drivers::Motors::MAX_POWER));

    if (m_power > 0) {
        pwm_set_chan_level(m_in1Slice, m_in1Channel, Drivers::Motors::MAX_POWER);
        pwm_set_chan_level(m_in2Slice, m_in2Channel, Drivers::Motors::MAX_POWER - m_power);
    } else if (m_power < 0) {
        pwm_set_chan_level(m_in1Slice, m_in1Channel, Drivers::Motors::MAX_POWER + m_power);
        pwm_set_chan_level(m_in2Slice, m_in2Channel, Drivers::Motors::MAX_POWER);
    } else {
        pwm_set_chan_level(m_in1Slice, m_in1Channel, Drivers::Motors::MAX_POWER);
        pwm_set_chan_level(m_in2Slice, m_in2Channel, Drivers::Motors::MAX_POWER);
    }
}

Motors::CurrentSensor::CurrentSensor(uint leftCurrentPin, uint rightCurrentPin) {
    adc_gpio_init(leftCurrentPin);
    adc_gpio_init(rightCurrentPin);

    m_bias.x = getBias(leftCurrentPin);
    m_bias.y = getBias(rightCurrentPin);
    setupRead(leftCurrentPin, rightCurrentPin);
}

float Motors::CurrentSensor::getBias(uint currentPin) {
    adc_init();
    adc_select_input(currentPin - 26u);
    adc_set_round_robin(0b00000);
    adc_fifo_setup(true, true, 1u, false, false);
    adc_set_clkdiv(0.0f);

    uint const dmaChannel = dma_claim_unused_channel(true);

    auto dmaConfig = dma_channel_get_default_config(dmaChannel);
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_16);
    channel_config_set_sniff_enable(&dmaConfig, true);
    channel_config_set_read_increment(&dmaConfig, false);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_dreq(&dmaConfig, DREQ_ADC);

    dma_sniffer_enable(dmaChannel, 0xf, true);
    dma_sniffer_set_data_accumulator(0u);

    dma_channel_configure(dmaChannel, &dmaConfig, &m_rawData[0], &adc_hw->fifo,
                          dma_encode_transfer_count(Drivers::Motors::SAMPLE_COUNT), true);

    adc_run(true);

    dma_channel_wait_for_finish_blocking(dmaChannel);

    dma_channel_unclaim(dmaChannel);
    dma_sniffer_disable();
    adc_run(false);
    adc_fifo_drain();

    float rawBias = static_cast<float>(dma_sniffer_get_data_accumulator()) /
                    static_cast<float>(Drivers::Motors::SAMPLE_COUNT);
    return rawBias * 3.3f / 4095.0f * 5.0f;
}

void Motors::CurrentSensor::setupRead(uint leftCurrentPin, uint rightCurrentPin) {
    adc_init();
    adc_select_input(leftCurrentPin - 26u);
    adc_set_round_robin((1u << (leftCurrentPin - 26u)) | (1u << (rightCurrentPin - 26u)));
    adc_fifo_setup(true, true, 1u, false, false);
    adc_set_clkdiv(0.0f);

    uint const dmaChannel1 = dma_claim_unused_channel(true);
    uint const dmaChannel2 = dma_claim_unused_channel(true);

    auto dmaConfig1 = dma_channel_get_default_config(dmaChannel1);
    channel_config_set_chain_to(&dmaConfig1, dmaChannel2);
    channel_config_set_transfer_data_size(&dmaConfig1, DMA_SIZE_16);
    channel_config_set_read_increment(&dmaConfig1, false);
    channel_config_set_write_increment(&dmaConfig1, false);
    channel_config_set_dreq(&dmaConfig1, DREQ_ADC);

    auto dmaConfig2 = dma_channel_get_default_config(dmaChannel2);
    channel_config_set_chain_to(&dmaConfig2, dmaChannel1);
    channel_config_set_transfer_data_size(&dmaConfig2, DMA_SIZE_16);
    channel_config_set_read_increment(&dmaConfig2, false);
    channel_config_set_write_increment(&dmaConfig2, false);
    channel_config_set_dreq(&dmaConfig2, DREQ_ADC);

    dma_channel_configure(dmaChannel1, &dmaConfig1, &m_rawData[0], &adc_hw->fifo,
                          dma_encode_transfer_count(1u), false);
    dma_channel_configure(dmaChannel2, &dmaConfig2, &m_rawData[1], &adc_hw->fifo,
                          dma_encode_transfer_count(1u), false);

    dma_channel_start(dmaChannel1);

    adc_run(true);
}

Vec2 Motors::CurrentSensor::data() const {
    return Vec2{ static_cast<float>(m_rawData[0]) * 3.3f / 4095.0f * 5.0f,
                 static_cast<float>(m_rawData[1]) * 3.3f / 4095.0f * 5.0f } -
           m_bias;
}

Motors::Motors(uint leftIn1Pin, uint leftIn2Pin, uint rightIn1Pin, uint rightIn2Pin,
               uint leftCurrentPin, uint rightCurrentPin)
    : m_left{ leftIn1Pin, leftIn2Pin },
      m_right{ rightIn1Pin, rightIn2Pin },
      m_currentSensor{ leftCurrentPin, rightCurrentPin } {}
