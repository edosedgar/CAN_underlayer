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

static inline void
startTransmit() {
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
        return;
}

static inline void
endTransmit() {
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
        return;
}

static inline void
sendSPI(uint8_t value) {
        LL_SPI_TransmitData8(SPI1, value);
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        return;
}

static inline uint8_t
recvSPI() {
        uint8_t byte;

        byte = LL_SPI_ReceiveData8(SPI1);
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        byte = LL_SPI_ReceiveData8(SPI1);
        return byte;
}

static void
setCANReg(const enum REGISTER reg, const uint8_t value) {
        startTransmit();
        sendSPI(INSTRUCTION_WRITE);
        sendSPI(reg);
        sendSPI(value);
        endTransmit();
}

static void
setCANRegs(const enum REGISTER reg, const uint8_t values[], const uint8_t n) {
        uint8_t i;

        startTransmit();
        sendSPI(INSTRUCTION_WRITE);
        sendSPI(reg);
        for (i = 0; i < n; i++) {
                sendSPI(values[i]);
        }
        endTransmit();
}

static void
modifyCANReg(const enum REGISTER reg, const uint8_t mask, const uint8_t data) {
        startTransmit();
        sendSPI(INSTRUCTION_BITMOD);
        sendSPI(reg);
        sendSPI(mask);
        sendSPI(data);
        endTransmit();
}

static uint8_t
readCANReg(const enum REGISTER reg) {
        uint8_t ret;

        startTransmit();
        sendSPI(INSTRUCTION_READ);
        recvSPI();
        sendSPI(reg);
        recvSPI();
        sendSPI(0x00);
        ret = recvSPI();
        endTransmit();

        return ret;
}

static void
readCANRegisters(const enum REGISTER reg, uint8_t values[], const uint8_t n) {
        uint8_t i;

        startTransmit();
        sendSPI(INSTRUCTION_READ);
        sendSPI(reg);
        // mcp2515 has auto-increment of address-pointer
        for (i = 0; i < n; i++) {
                LL_SPI_TransmitData8(SPI1, 0x00);
                values[i] = recvSPI();
        }
        endTransmit();
}

static void
can_reset() {
        int i;
        uint8_t zeros[14] = {0};

        startTransmit();
        sendSPI(INSTRUCTION_RESET);
        endTransmit();

        for (i = 0; i < 1000000; i++);

        setCANRegs(MCP_TXB0CTRL, zeros, 14);
        setCANRegs(MCP_TXB1CTRL, zeros, 14);
        setCANRegs(MCP_TXB2CTRL, zeros, 14);

        setCANReg(MCP_RXB0CTRL, 0);
        setCANReg(MCP_RXB1CTRL, 0);

        setCANReg(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF |
                  CANINTF_ERRIF | CANINTF_MERRF);

        modifyCANReg(MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT,
                     RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT);
        modifyCANReg(MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_STDEXT);
        return;
}

static enum ERROR
set_can_mode(const enum CANCTRL_REQOP_MODE mode) {
        modifyCANReg(MCP_CANCTRL, CANCTRL_REQOP, mode);

        uint8_t modeMatch = 0;
        uint32_t timeout = 0;
        uint32_t spin = 0;
        uint8_t newmode = 0;

        for (timeout = 0; timeout < 10; timeout++) {
                for (spin = 0; spin < 1000000; spin++);

                newmode = readCANReg(MCP_CANSTAT);
                newmode &= CANSTAT_OPMOD;

                modeMatch = newmode == mode;

                if (modeMatch) {
                        break;
                }
        }

        return modeMatch ? ERROR_OK : ERROR_FAIL;
}

static enum ERROR
set_can_bitrate(const enum CAN_SPEED canSpeed) {
        uint8_t set, cfg1, cfg2, cfg3;

        set_can_mode(CANCTRL_REQOP_CONFIG);
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
                setCANReg(MCP_CNF1, cfg1);
                setCANReg(MCP_CNF2, cfg2);
                setCANReg(MCP_CNF3, cfg3);
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
}

static enum ERROR
_can_send_msg(const enum TXBn txbn, const struct can_frame *frame) {
        const struct TXBn_REGS *txbuf = &TXB[txbn];

        uint8_t data[13];
        uint8_t ext = (frame->can_id & CAN_EFF_FLAG);
        uint8_t rtr = (frame->can_id & CAN_RTR_FLAG);
        uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

        can_prepare_id(data, ext, id);

        data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

        memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

        setCANRegs(txbuf->SIDH, data, 5 + frame->can_dlc);

        modifyCANReg(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

        return ERROR_OK;
}

enum ERROR
can_send_msg(const struct can_frame *frame) {
        uint8_t i;

        if (frame->can_dlc > CAN_MAX_DLEN) {
                return ERROR_FAILTX;
        }

        enum TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};


        for (i = 0; i < N_TXBUFFERS; i++) {
                const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
                uint8_t ctrlval = readCANReg(txbuf->CTRL);

                if ((ctrlval & TXB_TXREQ) == 0) {
                        return _can_send_msg(txBuffers[i], frame);
                }
        }
        return ERROR_FAILTX;
}

void
can_core_config(void) {
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
        set_can_bitrate(CAN_125KBPS);
        set_can_mode(CANCTRL_REQOP_NORMAL);

        struct can_frame canMsg1;
        canMsg1.can_id  = 0x123;
        canMsg1.can_dlc = 5;
        canMsg1.data[0] = 'H';
        canMsg1.data[1] = 'E';
        canMsg1.data[2] = 'L';
        canMsg1.data[3] = 'L';
        canMsg1.data[4] = 'L';
        canMsg1.data[5] = 'O';

        if (can_send_msg(&canMsg1) != ERROR_FAILTX)
                xprintf("Message has been sent\n");

        return;
}
