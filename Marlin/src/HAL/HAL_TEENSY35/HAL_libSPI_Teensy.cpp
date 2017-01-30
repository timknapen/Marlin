#include ../HAL.h


// Standard SPI functions
/** Initialise SPI bus */
void spiBegin(void) {
  SET_OUTPUT(SS_PIN);
  WRITE(SS_PIN, HIGH);
  SET_OUTPUT(SCK_PIN);
  SET_INPUT(MISO_PIN);
  SET_OUTPUT(MOSI_PIN);

  #if DISABLED(SOFTWARE_SPI)
    // set SS high - may be chip select for another SPI device
    #if SET_SPI_SS_HIGH
      WRITE(SS_PIN, HIGH);
    #endif  // SET_SPI_SS_HIGH
    // set a default rate
    spiInit(1);
  #endif  // SOFTWARE_SPI
}

/** Configure SPI for specified SPI speed */
void spiInit(uint8_t spiRate) {

}

/** Write single byte to SPI */
void spiSend(uint8_t b) {

}

/** Read single byte from SPI */
uint8_t spiRec(void) {

}

/** Read from SPI into buffer */
void spiRead(uint8_t* buf, uint16_t nbyte) {

}

/** Write token and then write from 512 byte buffer to SPI (for SD card) */
void spiSendBlock(uint8_t token, const uint8_t* buf) {

}
