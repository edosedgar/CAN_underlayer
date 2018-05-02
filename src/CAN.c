#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "CAN.h"

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
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        byte = LL_SPI_ReceiveData8(SPI1);
        return byte;
}

void
setCANReg(const enum REGISTER reg, const uint8_t value) {
        startTransmit();
        sendSPI(INSTRUCTION_WRITE);
        sendSPI(reg);
        sendSPI(value);
        endTransmit();
}

void
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

void
modifyCANReg(const enum REGISTER reg, const uint8_t mask, const uint8_t data) {
        startTransmit();
        sendSPI(INSTRUCTION_BITMOD);
        sendSPI(reg);
        sendSPI(mask);
        sendSPI(data);
        endTransmit();
}

uint8_t
readCANReg(const enum REGISTER reg) {
        uint8_t ret;

        startTransmit();
        sendSPI(INSTRUCTION_READ);
        sendSPI(reg);
        ret = recvSPI();
        endTransmit();

        return ret;
}

void
readCANRegisters(const enum REGISTER reg, uint8_t values[], const uint8_t n) {
        uint8_t i;

        startTransmit();
        sendSPI(INSTRUCTION_READ);
        sendSPI(reg);
        // mcp2515 has auto-increment of address-pointer
        for (i = 0; i < n; i++) {
                values[i] = recvSPI();
        }
        endTransmit();
}

void
resetCAN() {
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

void
setBitrateCAN() {

        return;
}

void
config_CAN(void) {
        int i;
        uint8_t zeros[14] = {0};
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
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
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
        LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV256);
        LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
        LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
        LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
        LL_SPI_EnableNSSPulseMgt(SPI1);
        LL_SPI_Enable(SPI1);
        /*
         * Init CAN
         */
        resetCAN();
        setBitrateCAN();

        return;
}
