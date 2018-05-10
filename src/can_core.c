#include <string.h>

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"

#include "can_core.h"
#include "xprintf.h"

const struct TXBn_REGS TXB[N_TXBUFFERS] = {
        {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
        {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
        {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};

const struct RXBn_REGS RXB[N_RXBUFFERS] = {
        {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
        {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};

static inline void
spi_start_cs() {
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

        return;
}

static inline void
spi_end_cs() {
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

        return;
}

static uint8_t
spi_send(uint8_t value) {
        uint8_t byte;

        LL_SPI_TransmitData8(SPI1, value);
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        while (LL_SPI_IsActiveFlag_BSY(SPI1));

        while (LL_SPI_IsActiveFlag_RXNE(SPI1))
                byte = LL_SPI_ReceiveData8(SPI1);

        return byte;
}

static void
can_set_reg(const enum REGISTER reg, const uint8_t value) {
        spi_start_cs();
        spi_send(INSTRUCTION_WRITE);
        spi_send(reg);
        spi_send(value);
        spi_end_cs();

        return;
}

static void
can_set_regs(const enum REGISTER reg, const uint8_t values[], const uint8_t n) {
        uint8_t i;

        spi_start_cs();
        spi_send(INSTRUCTION_WRITE);
        spi_send(reg);
        for (i = 0; i < n; i++) {
                spi_send(values[i]);
        }
        spi_end_cs();

        return;
}

static void
can_modify_reg(const enum REGISTER reg, const uint8_t mask,
               const uint8_t data) {
        spi_start_cs();
        spi_send(INSTRUCTION_BITMOD);
        spi_send(reg);
        spi_send(mask);
        spi_send(data);
        spi_end_cs();

        return;
}

static uint8_t
can_read_reg(const enum REGISTER reg) {
        uint8_t ret;

        spi_start_cs();
        spi_send(INSTRUCTION_READ);
        spi_send(reg);
        ret = spi_send(0x00);
        spi_end_cs();

        return ret;
}

static void
can_read_regs(const enum REGISTER reg, uint8_t values[], const uint8_t n) {
        uint8_t i;

        spi_start_cs();
        spi_send(INSTRUCTION_READ);
        spi_send(reg);
        // mcp2515 has auto-increment of address-pointer
        for (i = 0; i < n; i++) {
                values[i] = spi_send(0x00);
        }
        spi_end_cs();
}

static void
can_reset() {
        int i;
        uint8_t zeros[14] = {0};

        spi_start_cs();
        spi_send(INSTRUCTION_RESET);
        spi_end_cs();

        for (i = 0; i < 1000000; i++);

        can_set_regs(MCP_TXB0CTRL, zeros, 14);
        can_set_regs(MCP_TXB1CTRL, zeros, 14);
        can_set_regs(MCP_TXB2CTRL, zeros, 14);

        can_set_reg(MCP_RXB0CTRL, 0);
        can_set_reg(MCP_RXB1CTRL, 0);

        can_set_reg(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF |
                  CANINTF_ERRIF | CANINTF_MERRF);

        can_modify_reg(MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT,
                       RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT);
        can_modify_reg(MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_STDEXT);
        return;
}

static enum ERROR
can_set_mode(const enum CANCTRL_REQOP_MODE mode) {
        uint8_t modeMatch = 0;
        uint32_t timeout = 0;
        uint32_t spin = 0;
        uint8_t newmode = 0;

        can_modify_reg(MCP_CANCTRL, CANCTRL_REQOP, mode);

        for (timeout = 0; timeout < 10; timeout++) {
                for (spin = 0; spin < 1000000; spin++);

                newmode = can_read_reg(MCP_CANSTAT);
                newmode &= CANSTAT_OPMOD;

                modeMatch = newmode == mode;

                if (modeMatch) {
                        break;
                }
        }

        return modeMatch ? ERROR_OK : ERROR_FAIL;
}

static enum ERROR
can_set_bitrate(const enum CAN_SPEED canSpeed) {
        uint8_t set, cfg1, cfg2, cfg3;

        can_set_mode(CANCTRL_REQOP_CONFIG);
        set = 1;

        switch (canSpeed) {
        case (CAN_5KBPS):
                cfg1 = MCP_8MHz_5kBPS_CFG1;
                cfg2 = MCP_8MHz_5kBPS_CFG2;
                cfg3 = MCP_8MHz_5kBPS_CFG3;
                break;
        case (CAN_10KBPS):
                cfg1 = MCP_8MHz_10kBPS_CFG1;
                cfg2 = MCP_8MHz_10kBPS_CFG2;
                cfg3 = MCP_8MHz_10kBPS_CFG3;
                break;
        case (CAN_20KBPS):
                cfg1 = MCP_8MHz_20kBPS_CFG1;
                cfg2 = MCP_8MHz_20kBPS_CFG2;
                cfg3 = MCP_8MHz_20kBPS_CFG3;
                break;
        case (CAN_31K25BPS):
                cfg1 = MCP_8MHz_31k25BPS_CFG1;
                cfg2 = MCP_8MHz_31k25BPS_CFG2;
                cfg3 = MCP_8MHz_31k25BPS_CFG3;
                break;
        case (CAN_33KBPS):
                cfg1 = MCP_8MHz_33k3BPS_CFG1;
                cfg2 = MCP_8MHz_33k3BPS_CFG2;
                cfg3 = MCP_8MHz_33k3BPS_CFG3;
                break;
        case (CAN_40KBPS):
                cfg1 = MCP_8MHz_40kBPS_CFG1;
                cfg2 = MCP_8MHz_40kBPS_CFG2;
                cfg3 = MCP_8MHz_40kBPS_CFG3;
                break;
        case (CAN_50KBPS):
                cfg1 = MCP_8MHz_50kBPS_CFG1;
                cfg2 = MCP_8MHz_50kBPS_CFG2;
                cfg3 = MCP_8MHz_50kBPS_CFG3;
                break;
        case (CAN_80KBPS):
                cfg1 = MCP_8MHz_80kBPS_CFG1;
                cfg2 = MCP_8MHz_80kBPS_CFG2;
                cfg3 = MCP_8MHz_80kBPS_CFG3;
                break;
        case (CAN_100KBPS):
                cfg1 = MCP_8MHz_100kBPS_CFG1;
                cfg2 = MCP_8MHz_100kBPS_CFG2;
                cfg3 = MCP_8MHz_100kBPS_CFG3;
                break;
        case (CAN_125KBPS):
                cfg1 = MCP_8MHz_125kBPS_CFG1;
                cfg2 = MCP_8MHz_125kBPS_CFG2;
                cfg3 = MCP_8MHz_125kBPS_CFG3;
                break;
        case (CAN_200KBPS):
                cfg1 = MCP_8MHz_200kBPS_CFG1;
                cfg2 = MCP_8MHz_200kBPS_CFG2;
                cfg3 = MCP_8MHz_200kBPS_CFG3;
                break;
        case (CAN_250KBPS):
                cfg1 = MCP_8MHz_250kBPS_CFG1;
                cfg2 = MCP_8MHz_250kBPS_CFG2;
                cfg3 = MCP_8MHz_250kBPS_CFG3;
                break;
        case (CAN_500KBPS):
                cfg1 = MCP_8MHz_500kBPS_CFG1;
                cfg2 = MCP_8MHz_500kBPS_CFG2;
                cfg3 = MCP_8MHz_500kBPS_CFG3;
                break;
        case (CAN_1000KBPS):
                cfg1 = MCP_8MHz_1000kBPS_CFG1;
                cfg2 = MCP_8MHz_1000kBPS_CFG2;
                cfg3 = MCP_8MHz_1000kBPS_CFG3;
                break;
        default:
                set = 0;
                break;
        }

        if (set) {
                can_set_reg(MCP_CNF1, cfg1);
                can_set_reg(MCP_CNF2, cfg2);
                can_set_reg(MCP_CNF3, cfg3);
                return ERROR_OK;
        }
        else {
                return ERROR_FAIL;
        }
}

static void
can_prepare_id(uint8_t *buffer, const uint8_t ext, const uint32_t id) {
        uint16_t canid = (uint16_t)(id & 0x0FFFF);

        if (ext) {
                buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
                buffer[MCP_EID8] = (uint8_t) (canid >> 8);
                canid = (uint16_t)(id >> 16);
                buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
                buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
                buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
                buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
        } else {
                buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
                buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
                buffer[MCP_EID0] = 0;
                buffer[MCP_EID8] = 0;
        }

        return;
}

static enum ERROR
_can_send_msg(const enum TXBn txbn, const struct can_frame *frame) {
        const struct TXBn_REGS *txbuf = &TXB[txbn];
        uint8_t data[13];
        uint8_t ext = (frame->can_id & CAN_EFF_FLAG);
        uint8_t rtr = (frame->can_id & CAN_RTR_FLAG);
        uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));
        uint8_t req_attempts = REQS_ATTEMPTS;

        can_prepare_id(data, ext, id);
        data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;
        memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

        can_set_regs(txbuf->SIDH, data, 5 + frame->can_dlc);
        can_modify_reg(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

        //while (req_attempts--)
        //        if (can_read_reg(txbuf->CTRL) == 0x00)
        //                return ERROR_OK;
        //
        //xprintf("       Sent with ret code: %02X\n",can_read_reg(txbuf->CTRL));

        return ERROR_OK;
}

enum ERROR
can_send_msg(const struct can_frame *frame) {
        enum TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};
        uint8_t i;

        if (frame->can_dlc > CAN_MAX_DLEN) {
                return ERROR_FAILTX;
        }

        for (i = 0; i < N_TXBUFFERS; i++) {
                const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
                uint8_t ctrlval = can_read_reg(txbuf->CTRL);

                if ((ctrlval & TXB_TXREQ) == 0) {
                        return _can_send_msg(txBuffers[i], frame);
                }
        }
        return ERROR_FAILTX;
}

static uint8_t
can_get_status(void) {
        uint8_t byte;

        spi_start_cs();
        spi_send(INSTRUCTION_READ_STATUS);
        byte = spi_send(0x00);
        spi_end_cs();

        return byte;
}

static enum ERROR
_can_read_msg(const enum RXBn rxbn, struct can_frame *frame) {
        const struct RXBn_REGS *rxb = &RXB[rxbn];
        uint8_t tbufdata[5];
        uint32_t id;
        uint8_t dlc, ctrl;

        can_read_regs(rxb->SIDH, tbufdata, 5);
        id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

        if ((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK) {
                id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
                id = (id << 8) + tbufdata[MCP_EID8];
                id = (id << 8) + tbufdata[MCP_EID0];
                id |= CAN_EFF_FLAG;
        }

        dlc = (tbufdata[MCP_DLC] & DLC_MASK);
        if (dlc > CAN_MAX_DLEN) {
                return ERROR_FAIL;
        }

        ctrl = can_read_reg(rxb->CTRL);
        if (ctrl & RXBnCTRL_RTR) {
                id |= CAN_RTR_FLAG;
        }

        frame->can_id = id;
        frame->can_dlc = dlc;

        can_read_regs(rxb->DATA, frame->data, dlc);
        can_modify_reg(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

        return ERROR_OK;
}

enum ERROR
can_read_msg(struct can_frame *frame) {
        enum ERROR rc;
        uint8_t stat = can_get_status();

        if (stat & STAT_RX0IF) {
                rc = _can_read_msg(RXB0, frame);
        } else if (stat & STAT_RX1IF) {
                rc = _can_read_msg(RXB1, frame);
        } else {
                rc = ERROR_NOMSG;
        }

        return rc;
}

uint8_t
can_check_new_msg(void) {
        uint8_t res = can_get_status();

        if (res & STAT_RXIF_MASK) {
                return 1;
        } else {
                return 0;
        }
}

static enum ERROR
_can_set_filter(const enum RXF num, const uint8_t ext, const uint32_t can_id) {
        enum REGISTER reg;
        uint8_t tbufdata[4];

        switch (num) {
        case RXF0:
                reg = MCP_RXF0SIDH;
                break;
        case RXF1:
                reg = MCP_RXF1SIDH;
                break;
        case RXF2:
                reg = MCP_RXF2SIDH;
                break;
        case RXF3:
                reg = MCP_RXF3SIDH;
                break;
        case RXF4:
                reg = MCP_RXF4SIDH;
                break;
        case RXF5:
                reg = MCP_RXF5SIDH;
                break;
        default:
                return ERROR_FAIL;
        }

        can_prepare_id(tbufdata, ext, can_id);
        can_set_regs(reg, tbufdata, 4);

        return ERROR_OK;
}

static enum ERROR
_can_set_mask(const enum MASK mask, const uint8_t ext, const uint32_t data) {
        enum REGISTER reg;
        uint8_t tbufdata[4];

        switch (mask) {
        case MASK0:
                reg = MCP_RXM0SIDH;
                break;
        case MASK1:
                reg = MCP_RXM1SIDH;
                break;
        default:
                return ERROR_FAIL;
        }

        can_prepare_id(tbufdata, ext, data);
        can_set_regs(reg, tbufdata, 4);

        return ERROR_OK;
}

uint8_t
can_set_id(uint32_t can_id, uint32_t brdcst_id) {
        if (can_set_mode(CANCTRL_REQOP_CONFIG))
                return 1;

        _can_set_filter(RXF0, 0, brdcst_id);
        _can_set_filter(RXF1, 0, can_id);
        _can_set_filter(RXF2, 0, can_id);
        _can_set_filter(RXF3, 0, can_id);
        _can_set_filter(RXF4, 0, brdcst_id);
        _can_set_filter(RXF5, 0, can_id);

        _can_set_mask(MASK0, 0, 0xFFFF);
        _can_set_mask(MASK1, 0, 0xFFFF);

        if (can_set_mode(CANCTRL_REQOP_NORMAL) != ERROR_OK)
                return 1;

        return 0;
}

uint8_t
can_core_config(const enum CAN_SPEED canSpeed) {
        /*
         * Init GPIO
         */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        //SPI_MOSI
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_0);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
        //SPI_MISO
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_0);
        //SPI_SCK
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_0);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
        //SPI_CS
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
        /*
         * Init SPI
         */
        LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
        LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
        LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8);
        LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
        LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
        LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
        LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
        LL_SPI_Enable(SPI1);
        /*
         * Init CAN
         */
        can_reset();
        if (can_set_bitrate(canSpeed) != ERROR_OK)
                return 1;

        return 0;
}
