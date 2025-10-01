#include "drivers/Battery.hpp"

#include "Constants.hpp"

#include "hardware/adc.h"
#include "hardware/dma.h"

Battery::Battery(uint voltagePin) {
    adc_gpio_init(voltagePin);

    adc_init();
    adc_select_input(voltagePin - 26u);
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

    dma_channel_configure(dmaChannel, &dmaConfig, &m_temp, &adc_hw->fifo,
                          dma_encode_transfer_count(Drivers::Battery::SAMPLE_COUNT), true);

    adc_run(true);

    dma_channel_wait_for_finish_blocking(dmaChannel);
    dma_channel_unclaim(dmaChannel);
    adc_run(false);
    adc_fifo_drain();

    float const adcOutput = static_cast<float>(dma_sniffer_get_data_accumulator()) /
                            static_cast<float>(Drivers::Battery::SAMPLE_COUNT);
    m_voltage = (adcOutput * 3.3f / 4095.0f) / Drivers::Battery::DIVIDER_VALUE;
}
