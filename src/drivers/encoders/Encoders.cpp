#include "drivers/Encoders.hpp"

#include "Encoders.pio.h"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include "hardware/dma.h"
#include "hardware/pio.h"

Encoders::Encoders(PIO pio, uint csLeftPin, uint csRightPin, uint sckPin, uint misoPin) {
    uint const sm = pio_claim_unused_sm(pio, true);

    uint const dmaChannel1 = dma_claim_unused_channel(true);
    uint const dmaChannel2 = dma_claim_unused_channel(true);

    auto dmaConfig1 = dma_channel_get_default_config(dmaChannel1);
    channel_config_set_chain_to(&dmaConfig1, dmaChannel2);
    channel_config_set_read_increment(&dmaConfig1, false);
    channel_config_set_write_increment(&dmaConfig1, false);
    channel_config_set_dreq(&dmaConfig1, pio_get_dreq(pio, sm, false));

    auto dmaConfig2 = dma_channel_get_default_config(dmaChannel2);
    channel_config_set_chain_to(&dmaConfig2, dmaChannel1);
    channel_config_set_read_increment(&dmaConfig2, false);
    channel_config_set_write_increment(&dmaConfig2, false);
    channel_config_set_dreq(&dmaConfig2, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dmaChannel1, &dmaConfig1, &m_rawData[0], &pio->rxf[sm],
                          dma_encode_transfer_count(1u), false);
    dma_channel_configure(dmaChannel2, &dmaConfig2, &m_rawData[1], &pio->rxf[sm],
                          dma_encode_transfer_count(1u), false);

    dma_channel_start(dmaChannel1);

    uint offset = pio_add_program(pio, &encoders_program);
    encoders_program_init(pio, sm, offset, csLeftPin, csRightPin, sckPin, misoPin);
}

Vec2 Encoders::data() const {
    return { -static_cast<float>(m_rawData[0] >> 10u) / 16384.0f * 2.0f * Constants::PI,
             static_cast<float>(m_rawData[1] >> 10u) / 16384.0f * 2.0f * Constants::PI };
}
