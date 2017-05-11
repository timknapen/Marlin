#include <stdio.h>
#include <avr/io.h>
#include "MarlinConfig.h"
#include "TRAMS.h"

/**
 * @brief Send a byte via SPI and read the received
 * @param	data		to transmitted byte
 * @return	received byte
 */
uint8_t Trams::spi_readWriteByte(uint8_t data) {
  SPDR = data;
  while(!(SPSR & (1<<SPIF))); // polling the SPI Interrupt Flag
  return SPDR;	// return the received byte
}

/**
 * @brief Send a byte via SPI
 * @param		data		to be transmitted byte
 */
void Trams::spi_writeByte(uint8_t data) {
  SPDR = data;
  while(!(SPSR & (1<<SPIF)));
}

/**
 * @brief Initialize the SPI
 * SPI Master
 * 4Mhz(CPU_CLOCK / 4)
 * CPOL = 0
 * CPHA = 0
 * no interrupt
 */
void Trams::spi_init(void) {
	//Initialize the SPI interface
	//outputs
	DDR_SPI |= ((1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<SPI_CS));
	//inputs
	DDR_SPI &= ~(1<<SPI_MISO);

	//Initialize chip select pins
	SPI_CS_DDR	|= (1<<XAXIS_CS);
	SPI_CS_DDR	|= (1<<YAXIS_CS);
	SPI_CS_DDR	|= (1<<ZAXIS_CS);
	SPI_CS_DDR	|= (1<<E0AXIS_CS);

	//all cs high
	SPI_CS_PORT	|= (1<<XAXIS_CS);
	SPI_CS_PORT	|= (1<<YAXIS_CS);
	SPI_CS_PORT	|= (1<<ZAXIS_CS);
	SPI_CS_PORT	|= (1<<E0AXIS_CS);


	SPCR = ((1<<SPE )|              // SPI Enable
          (0<<SPIE)|              // SPI Interupt Enable
          (0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
          (1<<MSTR)|              // Master/Slave select
          (0<<SPR1)|(0<<SPR0)|    // SPI Clock Rate(fcpu/4 = 4Mhz)
          (0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
          (0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)
}

/**
 * @brief Reads four byte via SPI
 * @param		address		register address
 * @param		csPin		chip select
 * @return		status
 */
uint32_t Trams::spi_readRegister(uint8_t address, uint8_t slave) {
  uint8_t buf[4];
  uint32_t register_value = 0;

  SPI_CS_PORT &= ~(1 << slave); 	// enable slave, low activ

  // first read cycle to address the register
  spi_readWriteByte(address);
  spi_readWriteByte(0x00);
  spi_readWriteByte(0x00);
  spi_readWriteByte(0x00);
  spi_readWriteByte(0x00);

  PORTL |= (1 << slave);			// disable slave, low activ

  DELAY_3_NOP;

  SPI_CS_PORT &= ~(1 << slave); 	// select slave, low activ

  // second read cycle to get the register value
  spi_readWriteByte(address);
  buf[3] = spi_readWriteByte(0x00);
  buf[2] = spi_readWriteByte(0x00);
  buf[1] = spi_readWriteByte(0x00);
  buf[0] = spi_readWriteByte(0x00);

  SPI_CS_PORT |= (1 << slave);	// disable slave, low activ

  register_value |= buf[3];
  register_value = register_value << 8;
  register_value |= buf[2];
  register_value = register_value << 8;
  register_value |= buf[1];
  register_value = register_value << 8;
  register_value |= buf[0];

  return register_value;
}

/**
 * @brief Send five byte via SPI and received the status
 * @param		address		register address
 * @param		data		to transmitted data
 * @param		csPin		chip select
 * @return	status
 */
uint8_t Trams::spi_writeRegister(uint8_t address, uint32_t data, uint8_t slave) {
  uint8_t buf[4];
  uint8_t status = 0;

  buf[0] = data & 0xFF;
  buf[1] = (data & 0xFF00) >> 8;
  buf[2] = (data & 0xFF0000) >> 16;
  buf[3] = (data & 0xFF000000) >> 24;

  SPI_CS_PORT &= ~(1 << slave); 	// enable slave, low activ

  // address register
  spi_writeByte(address | READ_ACCESS);
  // send new register value
  spi_writeByte(buf[3]);
  spi_writeByte(buf[2]);
  spi_writeByte(buf[1]);
  spi_writeByte(buf[0]);

  SPI_CS_PORT |= (1 << slave);	// disable slave, low activ

  return status;
}


/**
 * @brief Reads the status from the MAMC
 * @param		csPin		chip select
 * @return	status
 */
uint8_t Trams::spi_readStatus(uint8_t slave) {
  uint8_t status;

  SPI_CS_PORT &= ~(1 << slave); 	// enable slave, low activ

  // send adress and read the status from FPGA
  status = spi_readWriteByte(GCONF);	// addressing any register int the tmc5130
  // send data, msb first
  spi_readWriteByte(0x00);
  spi_readWriteByte(0x00);
  spi_readWriteByte(0x00);
  spi_readWriteByte(0x00);

  SPI_CS_PORT |= (1 << slave);	// disable slave, low activ

  return status;
}

/**
 * @brief Initialize the Trinamic Drivers(TMC5130)
 * @param	csPin				chip select for spi (XAXIS,YAXIS,ZAXIS,E0AXIS)
 * @param	irun				Motor run current (0..31)
 * @param	ihold				Standstill current (0..31)
 * @param	stepper_direction	inverse/ not inverse
 * @return	none
 */
void Trams::TMC5130_init(uint8_t csPin, uint8_t irun, uint8_t ihold, uint8_t stepper_direction) {
	uint32_t value;
	value = SET_IHOLD(ihold) | SET_IRUN(irun) | SET_IHOLDDELAY(7);
	spi_writeRegister(IHOLD_IRUN, value, csPin);		//IHOLD and IRUN current
	spi_writeRegister(RAMPMODE, 0x0, csPin);			//select position mode
	spi_writeRegister(V_1, 0x0, csPin);					//Disables A1 and D1 in position mode, amax and vmax only
	spi_writeRegister(D_1, 0x10, csPin);				//D1 not zero
	spi_writeRegister(AMAX, 0xFFFF, csPin);				//Acceleration
	spi_writeRegister(VMAX, 0xFFFF, csPin);				//Velocity
	spi_writeRegister(CHOPCONF, 0x140101D5, csPin);		//Chopper Configuration
	spi_writeRegister(GCONF, 0x1084 | stepper_direction, csPin);	//General Configuration

	// initialize enable pin for given axis
	// set as output
	// default disable, low activ
	switch(csPin){
		case XAXIS_CS:	DRV_EN_X_DDR	|= (1<<DRV_EN_X);
						TMC5130_disableDriver(X_AXIS);
						break;
		case YAXIS_CS:	DRV_EN_Y_DDR	|= (1<<DRV_EN_Y);
						TMC5130_disableDriver(Y_AXIS);
						break;
		case ZAXIS_CS:	DRV_EN_Z_DDR	|= (1<<DRV_EN_Z);
						TMC5130_disableDriver(Z_AXIS);
						break;
		case E0AXIS_CS:	DRV_EN_E0_DDR	|= (1<<DRV_EN_E0);
						TMC5130_disableDriver(E_AXIS);
						break;
	}
}

/**
 * @brief Activates the driver stage for the given axis
 * @param	axis		axis
 * @return	none
 */
void Trams::TMC5130_enableDriver(uint8_t axis) {

	// low activ
	switch(axis){
		case X_AXIS:	DRV_EN_X_PORT	&= ~(1<<DRV_EN_X);
						break;
		case Y_AXIS:	DRV_EN_Y_PORT	&= ~(1<<DRV_EN_Y);
						break;
		case Z_AXIS:	DRV_EN_Z_PORT	&= ~(1<<DRV_EN_Z);
						break;
		case E_AXIS:	DRV_EN_E0_PORT	&= ~(1<<DRV_EN_E0);
						break;
		default:
					break;
	}

	return;
}


/**
 * @brief Deactivates the driver stage for the given axis
 * @param	axis		axis
 * @return	none
 */
void Trams::TMC5130_disableDriver(uint8_t axis) {

	// low activ
	switch(axis){
		case X_AXIS:	DRV_EN_X_PORT	|= (1<<DRV_EN_X);
						break;
		case Y_AXIS:	DRV_EN_Y_PORT	|= (1<<DRV_EN_Y);
						break;
		case Z_AXIS:	DRV_EN_Z_PORT	|= (1<<DRV_EN_Z);
						break;
		case E_AXIS:	DRV_EN_E0_PORT	|= (1<<DRV_EN_E0);
						break;
		default:
					break;
	}

	return;
}

/**
 * @brief Performs a homing for the given axis
 * @param	axis		axis to home
 * @return	none
 */
void Trams::TMC5130_homing(int axis) {
	unsigned int sw_register;
	uint8_t motor_direction;
	uint32_t stallguardthreshold;
	int homing_retract;
	int homing_speed;
	int axis_to_home;
	bool sg_active = false;
	uint32_t stall_speed;

	float STEPS_PER_UNIT_TMP[]=DEFAULT_AXIS_STEPS_PER_UNIT; // configuraton.h
	float HOMING_FEEDRATE_TMP[]={ HOMING_FEEDRATE_XY, HOMING_FEEDRATE_XY, HOMING_FEEDRATE_Z };

	switch(axis) {
		case X_AXIS:
			axis_to_home = XAXIS_CS;
			homing_retract = X_HOME_BUMP_MM * STEPS_PER_UNIT_TMP[0]; // configuraton_adv.h
			homing_speed = HOMING_FEEDRATE_TMP[0];
			sw_register = SWITCH_POSITION_X | SWITCH_POLARITY_X; // TMC_TRAMS_CONFIGURATION

			#ifdef STALLGUARD_X
				sg_active = true;
				stallguardthreshold = STALLGUARDTHRESHOLD_X;
				motor_direction = STEPPER_DIRECTION_X;
			#endif

			break;
		case Y_AXIS:
			axis_to_home = YAXIS_CS;
			homing_retract = Y_HOME_BUMP_MM * STEPS_PER_UNIT_TMP[1];
			homing_speed = HOMING_FEEDRATE_TMP[1];
			sw_register = SWITCH_POSITION_Y | SWITCH_POLARITY_Y;

			#ifdef STALLGUARD_Y
				sg_active = true;
				stallguardthreshold = STALLGUARDTHRESHOLD_Y;
				motor_direction = STEPPER_DIRECTION_Y;
			#endif

			break;
		case Z_AXIS:
			axis_to_home = ZAXIS_CS;
			homing_retract = Z_HOME_BUMP_MM * STEPS_PER_UNIT_TMP[2];
			homing_speed = HOMING_FEEDRATE_TMP[2];
			sw_register = SWITCH_POSITION_Z | SWITCH_POLARITY_Z;

			#ifdef STALLGUARD_Z
				sg_active = true;
				stallguardthreshold = STALLGUARDTHRESHOLD_Z;
				motor_direction = STEPPER_DIRECTION_Z;
			#endif

			break;
		default:
			return;
	}


	//Retract axis before homing so it doesn't crash into the printing bed
	if(axis == Z_AXIS) {
		spi_writeRegister(RAMPMODE, VELOCITY_MODE_POS, axis_to_home);	//VELOCITY MODE positive Direction
		spi_writeRegister(VMAX, homing_speed, axis_to_home);			//Homing Speed in VMAX
		_delay_ms(3000);
	}

	// Homing Procedure:
	// Enable Trinamic Drivers to start homing movement


	if(sg_active == true) {
		spi_writeRegister(SW_MODE, 0x00, axis_to_home);	//SWITCH REGISTER

		stall_speed = 16777216 / homing_speed;
		stall_speed = stall_speed / 16;  // match homing speed to actual microstep speed (at 1/16 microstep)
		stall_speed = stall_speed * 1.10; // Activate stallGuard sligthly below desired homing velocity (provide 10% tolerance)

		spi_writeRegister(GCONF, 0x1080 | motor_direction, axis_to_home);	//stealthchop off for stallguard homing
		spi_writeRegister(COOLCONF, ((stallguardthreshold & 0x7F)<<16),axis_to_home);//sgt <-- Entry the value determined for SGT: lower value=higher sensitivity (lower force for stall detection)
		spi_writeRegister(TCOOLTHRS, stall_speed  ,axis_to_home);//TCOOLTHRS
		spi_writeRegister(SW_MODE, 0x400, axis_to_home);	//SWITCH REGISTER
		spi_writeRegister(AMAX, 100, axis_to_home);	//AMAX for stallGuard homing shall be significantly lower than AMAX for printing

		// Set velocity mode in direction to the endstop
		spi_writeRegister(RAMPMODE, VELOCITY_MODE_NEG, axis_to_home);	//VELOCITY MODE, direction to the endstop
		spi_writeRegister(VMAX, homing_speed, axis_to_home);	//Homing Speed in VMAX

		_delay_ms(20);

		//While motor is still moving (vzero != 1)
		while((spi_readRegister(RAMP_STAT, axis_to_home) & VZERO) != VZERO);

		// Endstop reached. Reset and retract
		spi_writeRegister(RAMPMODE, HOLD_MODE, axis_to_home);		//HOLD Mode
		spi_writeRegister(XACTUAL, 0x0, axis_to_home);				//XACTUAL = 0
		spi_writeRegister(XTARGET, 0x0, axis_to_home);				//XTARGET = 0
		spi_writeRegister(SW_MODE, 0x0, axis_to_home);				//SWITCH REGISTER
		spi_writeRegister(RAMPMODE, POSITIONING_MODE, axis_to_home);//Position MODE
		spi_writeRegister(VMAX, homing_speed, axis_to_home);	//Homing Speed in VMAX
		spi_writeRegister(DMAX, 0xFFFF, axis_to_home);				//DMAX
		spi_writeRegister(XTARGET, homing_retract, axis_to_home);	//XTARGET = homing_retract

		_delay_ms(20);

		//While motor is still moving (vzero != 1)
		while((spi_readRegister(RAMP_STAT, axis_to_home) & VZERO) != VZERO);

		// Endstop reached. Reset and retract
		spi_writeRegister(SW_MODE, 0x0, axis_to_home);	//SWITCH REGISTER
		spi_writeRegister(RAMPMODE, 0x3, axis_to_home);	//HOLD Mode
		spi_writeRegister(GCONF, 0x1080 | motor_direction, axis_to_home);//Turn on stealthchop again
		spi_writeRegister(XACTUAL, 0x0, axis_to_home);	//XACTUAL = 0
		spi_writeRegister(XTARGET, 0x0, axis_to_home);	//XTARGET = 0
		spi_writeRegister(RAMPMODE, 0x0, axis_to_home);	//Position MODE
		_delay_ms(200);
	} else {
		TMC5130_enableDriver(axis);

		// Set velocity mode in direction to the endstop
		spi_writeRegister(RAMPMODE, VELOCITY_MODE_NEG, axis_to_home);	//VELOCITY MODE negative Direction
		spi_writeRegister(VMAX, homing_speed, axis_to_home);			//Homing Speed in VMAX

		//Config switch register of TMC5130
		spi_writeRegister(SW_MODE, sw_register, axis_to_home);		//SWITCH REGISTER

		//While motor is still moving (vzero != 1)
		while((spi_readRegister(RAMP_STAT, axis_to_home) & VZERO) != VZERO);

		// Endstop reached. Reset and retract
		spi_writeRegister(RAMPMODE, HOLD_MODE, axis_to_home);		//HOLD Mode
		spi_writeRegister(XACTUAL, 0x0, axis_to_home);				//XACTUAL = 0
		spi_writeRegister(XTARGET, 0x0, axis_to_home);				//XTARGET = 0
		spi_writeRegister(SW_MODE, 0x0, axis_to_home);				//SWITCH REGISTER
		spi_writeRegister(RAMPMODE, POSITIONING_MODE, axis_to_home);//Position MODE
		spi_writeRegister(VMAX, homing_speed, axis_to_home);		//Homing Speed in VMAX
		spi_writeRegister(DMAX, 0xFFFF, axis_to_home);				//DMAX
		spi_writeRegister(XTARGET, homing_retract, axis_to_home);	//XTARGET = homing_retract

		_delay_ms(200);

		//While motor is still moving (vzero != 1)
		while((spi_readRegister(RAMP_STAT, axis_to_home) & VZERO) != VZERO);

		//Retract finished
		spi_writeRegister(SW_MODE, sw_register, axis_to_home);		//SWITCH REGISTER
		spi_writeRegister(RAMPMODE, HOLD_MODE, axis_to_home);		//HOLD Mode
		spi_writeRegister(XACTUAL, 0x0, axis_to_home);				//XACTUAL = 0
		spi_writeRegister(XTARGET, 0x0, axis_to_home);				//XTARGET = 0
		spi_writeRegister(RAMPMODE, POSITIONING_MODE, axis_to_home);//Position MODE
	}
}

// Additional motion execution queue for use with Trinamic TMC5130
#define MOTION_BUFFER_SIZE 16
// Conversion of units between TMC5130 and Arduino
#define TMC5130clockFrequency (double)16000000
#define TMC5130_a_divisor (long)128
#define TMC5130_t_factor (double)1.048576

static Trams::motion_block_t motion_buffer[MOTION_BUFFER_SIZE],  // A ring buffer for motion movements
                      motion_buffer_block_old;
static volatile unsigned char motion_buffer_head = 0,           // Index of the next block to be pushed
                              motion_buffer_tail = 0;
static volatile bool motion_buffer_full = false;
static unsigned char out_bits;        // The next stepping-bits to be output
volatile long pos[NUM_AXIS] = { 0 };
static volatile long timerClk, nextTimerClk;
static volatile double TEMPtimerClk;

/**
 * @brief Called when the current block is no longer needed. Discards the block and makes the memory
 * available for new blocks.)
 * @return  none
 */
FORCE_INLINE void Trams::discard_current_motion_block(void) {
  motion_buffer_tail = (motion_buffer_tail + 1) & (MOTION_BUFFER_SIZE - 1);
  motion_buffer_full = false;
}


/**
 * @brief  Gets the current block. Returns NULL if buffer empty
 * @return pointer of the current block, null if empty
 */
FORCE_INLINE Trams::motion_block_t *Trams::get_current_motion_block(void) {
  if ((motion_buffer_head == motion_buffer_tail) && (motion_buffer_full == false)) {
    return(NULL);
  }
  else
    return &motion_buffer[motion_buffer_tail];
}


/**
 * @brief  Returns true if the buffer has a queued block, false otherwise
 * @return  false if queue emty, else true
 */
FORCE_INLINE bool Trams::motion_blocks_queued(void) {
  return ((motion_buffer_head != motion_buffer_tail) || motion_buffer_full);
}


/**
 * @brief  Returns a pointer to the next free position in the motion buffer, if there is one
 * @return  pointer of next free block
 */
FORCE_INLINE Trams::motion_block_t *Trams::get_next_free_motion_block(void) {
  if(motion_buffer_full)
    return NULL;
  else
    return &motion_buffer[motion_buffer_head];
}


/**
 * @brief  Increases the head pointer of the buffer. Don't use without proving
 * before that the queue is not full
 * @return  none
 */
FORCE_INLINE void Trams::append_motion_block(void) {
  CRITICAL_SECTION_START;
  motion_buffer_head += 1;
  if(motion_buffer_head == MOTION_BUFFER_SIZE)
    motion_buffer_head = 0;
  if(motion_buffer_head == motion_buffer_tail)
    motion_buffer_full = true;
  CRITICAL_SECTION_END;
}

/**
 * @brief  Returns the number of queued blocks
 * @return  number of queued blocks
 */
unsigned char Trams::blocks_in_motion_queue() {
  char temp = motion_buffer_head - motion_buffer_tail;
  if(temp < 0)
    temp += MOTION_BUFFER_SIZE;
  return temp;
}

/**
 * @brief Pre-calculates the parameters for the current block, in order to save time. Hence, in the interrupt function
 * there is no processing, only sending the data via SPI
 * @return none
 */
void Trams::calculate(void) {
  unsigned long scale_axis; // scale factor
  Trams::motion_block_t *current_motion_block = get_next_free_motion_block();

  // block in motion_block available?
  if(current_motion_block == NULL) return;

  // If there is no current block, attempt to pop one from the buffer
  current_block = planner.get_current_block();
  // Anything in the buffer?
  if (current_block == NULL) return;
  SBI(current_block->flag, BLOCK_BIT_BUSY); //Block being used

  // Set directions
  out_bits = current_block->direction_bits;

  // Calculate ramp parameters of the current block
  // If there is any movement in the x-axis
  if(current_block->steps[X_AXIS] != 0) {
    //accel[X_AXIS] = current_block->steps[X_AXIS] * current_block->acceleration_steps_per_s2 / current_block->step_event_count
    scale_axis = (((unsigned long)current_block->steps[X_AXIS])<<16) / current_block->step_event_count;
    current_motion_block->accel[X_AXIS] = (scale_axis>>7) * current_block->acceleration_steps_per_s2;
    current_motion_block->accel[X_AXIS] = (current_motion_block->accel[X_AXIS]>>16);


    // Keep acceleration high to be able to brake
    if(current_motion_block->accel[X_AXIS] == 0)
      current_motion_block->accel[X_AXIS] = 1000;

    //initial_speed[X_AXIS] = current_block->steps[X_AXIS] * current_block->initial_rate / current_block->step_event_count;
    current_motion_block->initial_speed[X_AXIS] = scale_axis * current_block->initial_rate;
    current_motion_block->initial_speed[X_AXIS] = (current_motion_block->initial_speed[X_AXIS] >> 16);

    //nominal_speed[X_AXIS] = current_block->steps[X_AXIS] * current_block->nominal_rate / current_block->step_event_count;
    current_motion_block->nominal_speed[X_AXIS] = scale_axis * current_block->nominal_rate;
    current_motion_block->nominal_speed[X_AXIS] = (current_motion_block->nominal_speed[X_AXIS]>>16);

    if(current_motion_block->nominal_speed[X_AXIS] < current_motion_block->initial_speed[X_AXIS])
      current_motion_block->initial_speed[X_AXIS] = current_motion_block->nominal_speed[X_AXIS];

    //final_speed[X_AXIS] = current_block->steps[X_AXIS] * current_block->final_rate / current_block->step_event_count;
    current_motion_block->final_speed[X_AXIS] = scale_axis * current_block->final_rate;
    current_motion_block->final_speed[X_AXIS] = (current_motion_block->final_speed[X_AXIS]>>16);

    if(current_motion_block->final_speed[X_AXIS] < 10)
      current_motion_block->final_speed[X_AXIS] = 10;

    //speed_x = - speed_x;
    if((out_bits & (1<<X_AXIS))!=0) {
      pos[X_AXIS] -= current_block->steps[X_AXIS];
      current_motion_block->pos[X_AXIS] = pos[X_AXIS];
    } else {
      pos[X_AXIS] += current_block->steps[X_AXIS];
      current_motion_block->pos[X_AXIS] = pos[X_AXIS];
    }

    //store the calculate values in motion_buffer_block_old
    motion_buffer_block_old.accel[X_AXIS] = current_motion_block->accel[X_AXIS];
    motion_buffer_block_old.initial_speed[X_AXIS] = current_motion_block->initial_speed[X_AXIS];
    motion_buffer_block_old.nominal_speed[X_AXIS] = current_motion_block->nominal_speed[X_AXIS];
    motion_buffer_block_old.final_speed[X_AXIS] = current_motion_block->final_speed[X_AXIS];
    motion_buffer_block_old.pos[X_AXIS] = current_motion_block->pos[X_AXIS];
  } else {
    //If there is no movement in the x-axis use the last calculate values
    current_motion_block->accel[X_AXIS] = motion_buffer_block_old.accel[X_AXIS];
    current_motion_block->initial_speed[X_AXIS] = motion_buffer_block_old.initial_speed[X_AXIS];
    current_motion_block->nominal_speed[X_AXIS] = motion_buffer_block_old.nominal_speed[X_AXIS];
    current_motion_block->final_speed[X_AXIS] = motion_buffer_block_old.final_speed[X_AXIS];
    current_motion_block->pos[X_AXIS] = motion_buffer_block_old.pos[X_AXIS];
  }


  // If there is any movement in the y-axis
  if(current_block->steps[Y_AXIS] != 0) {
    //accel[Y_AXIS] = current_block->steps[Y_AXIS] * current_block->acceleration_steps_per_s2 / current_block->step_event_count;
    scale_axis = (((unsigned long)current_block->steps[Y_AXIS])<<16) / current_block->step_event_count;
    current_motion_block->accel[Y_AXIS] = (scale_axis>>7) * current_block->acceleration_steps_per_s2;
    current_motion_block->accel[Y_AXIS] = (current_motion_block->accel[Y_AXIS]>>16);


    // Keep acceleration high to be able to brake
    if(current_motion_block->accel[Y_AXIS] == 0)
      current_motion_block->accel[Y_AXIS] = 1000;

    //initial_speed[Y_AXIS] = current_block->steps[Y_AXIS] * current_block->initial_rate / current_block->step_event_count;
    current_motion_block->initial_speed[Y_AXIS] = scale_axis * current_block->initial_rate;
    current_motion_block->initial_speed[Y_AXIS] = (current_motion_block->initial_speed[Y_AXIS]>>16);

    //nominal_speed[Y_AXIS] = current_block->steps[Y_AXIS] * current_block->nominal_rate / current_block->step_event_count;
    current_motion_block->nominal_speed[Y_AXIS] = scale_axis * current_block->nominal_rate;
    current_motion_block->nominal_speed[Y_AXIS] = (current_motion_block->nominal_speed[Y_AXIS]>>16);

    if(current_motion_block->nominal_speed[Y_AXIS] < current_motion_block->initial_speed[Y_AXIS])
      current_motion_block->initial_speed[Y_AXIS] = current_motion_block->nominal_speed[Y_AXIS];

    //final_speed[Y_AXIS] = current_block->steps[Y_AXIS] * current_block->final_rate / current_block->step_event_count;
    current_motion_block->final_speed[Y_AXIS] = scale_axis * current_block->final_rate;
    current_motion_block->final_speed[Y_AXIS] = (current_motion_block->final_speed[Y_AXIS]>>16);

    if(current_motion_block->final_speed[Y_AXIS] < 10)
      current_motion_block->final_speed[Y_AXIS] = 10;

    //speed_y = - speed_y;
    if((out_bits & (1<<Y_AXIS))!=0) {
      pos[Y_AXIS] -= current_block->steps[Y_AXIS];
      current_motion_block->pos[Y_AXIS] = pos[Y_AXIS];
    } else {
      pos[Y_AXIS] += current_block->steps[Y_AXIS];
      current_motion_block->pos[Y_AXIS] = pos[Y_AXIS];
    }

    //store the calculate values in motion_buffer_block_old
    motion_buffer_block_old.accel[Y_AXIS] = current_motion_block->accel[Y_AXIS];
    motion_buffer_block_old.initial_speed[Y_AXIS] = current_motion_block->initial_speed[Y_AXIS];
    motion_buffer_block_old.nominal_speed[Y_AXIS] = current_motion_block->nominal_speed[Y_AXIS];
    motion_buffer_block_old.final_speed[Y_AXIS] = current_motion_block->final_speed[Y_AXIS];
    motion_buffer_block_old.pos[Y_AXIS] = current_motion_block->pos[Y_AXIS];
  } else {
    //If there is no movement in the y-axis use the last calculate values
    current_motion_block->accel[Y_AXIS] = motion_buffer_block_old.accel[Y_AXIS];
    current_motion_block->initial_speed[Y_AXIS] = motion_buffer_block_old.initial_speed[Y_AXIS];
    current_motion_block->nominal_speed[Y_AXIS] = motion_buffer_block_old.nominal_speed[Y_AXIS];
    current_motion_block->final_speed[Y_AXIS] = motion_buffer_block_old.final_speed[Y_AXIS];
    current_motion_block->pos[Y_AXIS] = motion_buffer_block_old.pos[Y_AXIS];
  }

  // If there is any movement in the z-axis
  if(current_block->steps[Z_AXIS] != 0) {
    //accel[Z_AXIS] = current_block->steps[Z_AXIS] * current_block->acceleration_steps_per_s2 / current_block->step_event_count;
    scale_axis = (((unsigned long)current_block->steps[Z_AXIS])<<16) / current_block->step_event_count;
    current_motion_block->accel[Z_AXIS] = (scale_axis>>7) * current_block->acceleration_steps_per_s2;
    current_motion_block->accel[Z_AXIS] = (current_motion_block->accel[Z_AXIS]>>16);


    // Keep acceleration high to be able to brake
    if(current_motion_block->accel[Z_AXIS] == 0)
      current_motion_block->accel[Z_AXIS] = 1000;

    //initial_speed[Z_AXIS] = current_block->steps[Z_AXIS] * current_block->initial_rate / current_block->step_event_count;
    current_motion_block->initial_speed[Z_AXIS] = scale_axis * current_block->initial_rate;
    current_motion_block->initial_speed[Z_AXIS] = (current_motion_block->initial_speed[Z_AXIS]>>16);

    //nominal_speed[Z_AXIS] = current_block->steps[Z_AXIS] * current_block->nominal_rate / current_block->step_event_count;
    current_motion_block->nominal_speed[Z_AXIS] = scale_axis * current_block->nominal_rate;
    current_motion_block->nominal_speed[Z_AXIS] = (current_motion_block->nominal_speed[Z_AXIS]>>16);

    if(current_motion_block->nominal_speed[Z_AXIS] < current_motion_block->initial_speed[Z_AXIS])
      current_motion_block->initial_speed[Z_AXIS] = current_motion_block->nominal_speed[Z_AXIS];

    //final_speed[Z_AXIS] = current_block->steps[Z_AXIS] * current_block->final_rate / current_block->step_event_count;
    current_motion_block->final_speed[Z_AXIS] = scale_axis * current_block->final_rate;
    current_motion_block->final_speed[Z_AXIS] = (current_motion_block->final_speed[Z_AXIS]>>16);

    if(current_motion_block->final_speed[Z_AXIS] < 10)
      current_motion_block->final_speed[Z_AXIS] = 10;
    if(current_motion_block->final_speed[Z_AXIS] > 800)
      current_motion_block->final_speed[Z_AXIS] = 800;

    //speed_z = - speed_z;
    if((out_bits & (1<<Z_AXIS))!=0) {
      pos[Z_AXIS] -= current_block->steps[Z_AXIS];
      current_motion_block->pos[Z_AXIS] = pos[Z_AXIS];
    } else {
      pos[Z_AXIS] += current_block->steps[Z_AXIS];
      current_motion_block->pos[Z_AXIS] = pos[Z_AXIS];
    }

    current_motion_block->pos_change_z = true;
  } else {
    current_motion_block->pos_change_z = false;
  }

  // If there is any movement in the e-axis
  if(current_block->steps[E_AXIS] != 0) {
    //accel[E_AXIS] = current_block->steps[E_AXIS] * current_block->acceleration_steps_per_s2 / current_block->step_event_count;
    scale_axis = (((unsigned long)current_block->steps[E_AXIS])<<16) / current_block->step_event_count;
    current_motion_block->accel[E_AXIS] = (scale_axis>>7) * current_block->acceleration_steps_per_s2;
    current_motion_block->accel[E_AXIS] = (current_motion_block->accel[E_AXIS]>>16);

    // Keep acceleration high to be able to brake
    if(current_motion_block->accel[E_AXIS] == 0)
      current_motion_block->accel[E_AXIS] = 1000;

    //initial_speed[E_AXIS] = current_block->steps[E_AXIS] * current_block->initial_rate / current_block->step_event_count;
    current_motion_block->initial_speed[E_AXIS] = scale_axis * current_block->initial_rate;
    current_motion_block->initial_speed[E_AXIS] = (current_motion_block->initial_speed[E_AXIS]>>16);

    //nominal_speed[E_AXIS] = current_block->steps[E_AXIS] * current_block->nominal_rate / current_block->step_event_count;
    current_motion_block->nominal_speed[E_AXIS] = scale_axis * current_block->nominal_rate;
    current_motion_block->nominal_speed[E_AXIS] = (current_motion_block->nominal_speed[E_AXIS]>>16);

    if(current_motion_block->nominal_speed[E_AXIS] < current_motion_block->initial_speed[E_AXIS])
      current_motion_block->initial_speed[E_AXIS] = current_motion_block->nominal_speed[E_AXIS];

    //final_speed[E_AXIS] = current_block->steps[E_AXIS] * current_block->final_rate / current_block->step_event_count;
    current_motion_block->final_speed[E_AXIS] = scale_axis * current_block->final_rate;
    current_motion_block->final_speed[E_AXIS] = (current_motion_block->final_speed[E_AXIS]>>16);

    if(current_motion_block->final_speed[E_AXIS] < 10)
      current_motion_block->final_speed[E_AXIS] = 10;
    if(current_motion_block->final_speed[E_AXIS] > 800)
      current_motion_block->final_speed[E_AXIS] = 800;

    //speed_e = - speed_e;
    if((out_bits & (1<<E_AXIS))!=0) {
      pos[E_AXIS] -= current_block->steps[E_AXIS];
      current_motion_block->pos[E_AXIS] = pos[E_AXIS];
    } else {
      pos[E_AXIS] += current_block->steps[E_AXIS];
      current_motion_block->pos[E_AXIS] = pos[E_AXIS];
    }

    //store the calculate values in motion_buffer_block_old
    motion_buffer_block_old.accel[E_AXIS] = current_motion_block->accel[E_AXIS];
    motion_buffer_block_old.initial_speed[E_AXIS] = current_motion_block->initial_speed[E_AXIS];
    motion_buffer_block_old.nominal_speed[E_AXIS] = current_motion_block->nominal_speed[E_AXIS];
    motion_buffer_block_old.final_speed[E_AXIS] = current_motion_block->final_speed[E_AXIS];
    motion_buffer_block_old.pos[E_AXIS] = current_motion_block->pos[E_AXIS];
  } else {
    //If there is no movement in the e-axis use the last calculate values
    current_motion_block->accel[E_AXIS] = motion_buffer_block_old.accel[E_AXIS];
    current_motion_block->initial_speed[E_AXIS] = motion_buffer_block_old.initial_speed[E_AXIS];
    current_motion_block->nominal_speed[E_AXIS] = motion_buffer_block_old.nominal_speed[E_AXIS];
    current_motion_block->final_speed[E_AXIS] = motion_buffer_block_old.final_speed[E_AXIS];
    current_motion_block->pos[E_AXIS] = motion_buffer_block_old.pos[E_AXIS];
  }

  // Calculate duration of the movement
  // I - Acceleration phase
  double temp;
  //timerClk = (current_block->nominal_rate - current_block->initial_rate) / current_block->acceleration_steps_per_s2;
  nextTimerClk = current_block->nominal_rate - current_block->initial_rate;
  temp = (double) nextTimerClk /  current_block->acceleration_steps_per_s2;
  nextTimerClk = temp * 2000000;

  // II - Plateau / Constant speed phase (if applies)
  if (current_block->decelerate_after > current_block->accelerate_until) {
    //timer += phase2duration;
    //timerClk = timerClk + (2000 *1000 * (current_block->decelerate_after - current_block->accelerate_until) / current_block->nominal_rate);
    TEMPtimerClk = current_block->decelerate_after - current_block->accelerate_until;
    TEMPtimerClk = TEMPtimerClk / current_block->nominal_rate;
    TEMPtimerClk = TEMPtimerClk * 2000000;
    nextTimerClk = nextTimerClk + TEMPtimerClk;
  }

  // III - Deceleration phase
  //timerClk = 2000 * 1000 * (current_block->nominal_rate - current_block->final_rate) / current_block->acceleration_steps_per_s2;
  TEMPtimerClk = current_block->nominal_rate - current_block->final_rate;
  TEMPtimerClk = TEMPtimerClk / current_block->acceleration_steps_per_s2;
  TEMPtimerClk = TEMPtimerClk * 2000000;
  TEMPtimerClk = nextTimerClk + TEMPtimerClk;

  // Adjust value for the internal units of the chip (we changed units of the acceleration, but speed has also internal units)
  nextTimerClk = TEMPtimerClk * TMC5130_t_factor;

  current_motion_block->nextTimerClk = nextTimerClk;
  motion_buffer_block_old.nextTimerClk = current_motion_block->nextTimerClk;

  // Calculations finished: Let the interrupt send the information
  current_motion_block->calcready = true;
  motion_buffer_block_old.calcready = current_motion_block->calcready;
  append_motion_block();
  current_block = NULL;
  planner.discard_current_block();  
}

/**
 * @brief "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
 * It has been divided into st_calculate and this function. In this part we send the information of the current
 * block, once the calculations of st_calculate are ready (current_block->ready) and set the timer for the next
 * wake-up
 * @return none
 */
void Trams::isr() {
  unsigned long temp_timer;
  motion_block_t *current_motion_block;

  if (timerClk == 0) {
    current_motion_block =  get_current_motion_block();
    if ((current_motion_block->calcready == true) && (current_motion_block != NULL)) {
      // Send accel and nominal speed SPI datagrams
      spi_writeRegister(VMAX, current_motion_block->nominal_speed[X_AXIS],  TRAMS_XAXIS);  //Velocity of X
      spi_writeRegister(AMAX, current_motion_block->accel[X_AXIS],          TRAMS_XAXIS);  //ACC of X
      spi_writeRegister(DMAX, current_motion_block->accel[X_AXIS],          TRAMS_XAXIS);  //DEC of X
      spi_writeRegister(VMAX, current_motion_block->nominal_speed[Y_AXIS],  TRAMS_YAXIS);  //Velocity of Y
      spi_writeRegister(AMAX, current_motion_block->accel[Y_AXIS],          TRAMS_YAXIS);  //ACC of Y
      spi_writeRegister(DMAX, current_motion_block->accel[Y_AXIS],          TRAMS_YAXIS);  //DEC of Y

      // only send if there is any movement in the z-axis
      if (current_motion_block->pos_change_z == true) {
        spi_writeRegister(VMAX, current_motion_block->nominal_speed[Z_AXIS],  TRAMS_ZAXIS); //Velocity of ZAXIS
        spi_writeRegister(AMAX, current_motion_block->accel[Z_AXIS],          TRAMS_ZAXIS); //ACC of ZAXIS
        spi_writeRegister(DMAX, current_motion_block->accel[Z_AXIS],          TRAMS_ZAXIS); //DEC of ZAXIS
      }

      //check if new data receive
      //st_check_UART_rx();

      spi_writeRegister(VMAX, current_motion_block->nominal_speed[E_AXIS],  TRAMS_E0AXIS);  //Velocity of E0AXIS
      spi_writeRegister(AMAX, current_motion_block->accel[E_AXIS],          TRAMS_E0AXIS);  //ACC of E0AXIS
      spi_writeRegister(DMAX, current_motion_block->accel[E_AXIS],          TRAMS_E0AXIS);  //DEC of E0AXIS

      // Send initial and final speeed SPI datagrams
      spi_writeRegister(VSTART, current_motion_block->initial_speed[X_AXIS],  TRAMS_XAXIS); //Initial vel of X
      spi_writeRegister(VSTOP, current_motion_block->final_speed[X_AXIS],     TRAMS_XAXIS); //Final vel of X
      spi_writeRegister(VSTART, current_motion_block->initial_speed[Y_AXIS],  TRAMS_YAXIS); //Initial vel of Y
      spi_writeRegister(VSTOP, current_motion_block->final_speed[Y_AXIS],     TRAMS_YAXIS); //Final vel of Y

      // only send if there is any movement in the z-axis
      if (current_motion_block->pos_change_z == true) {
        spi_writeRegister(VSTART, current_motion_block->initial_speed[Z_AXIS],  TRAMS_ZAXIS);//Initial vel of Z
        spi_writeRegister(VSTOP, current_motion_block->final_speed[Z_AXIS],     TRAMS_ZAXIS);//Final vel of Z
      }

      //check if new data receive
      //st_check_UART_rx();

      spi_writeRegister(VSTART, current_motion_block->initial_speed[E_AXIS],  TRAMS_E0AXIS); //Initial vel of E0
      spi_writeRegister(VSTOP, current_motion_block->final_speed[E_AXIS],     TRAMS_E0AXIS);    //Final vel of E0

      // Send target positions, movement starts immediately
      // only send if there is any movement in the z-axis
      if (current_motion_block->pos_change_z == true)
        spi_writeRegister(XTARGET, current_motion_block->pos[Z_AXIS], TRAMS_ZAXIS);   // target

      spi_writeRegister(XTARGET, current_motion_block->pos[X_AXIS],   TRAMS_XAXIS);   // target
      spi_writeRegister(XTARGET, current_motion_block->pos[Y_AXIS],   TRAMS_YAXIS);   // target
      spi_writeRegister(XTARGET, current_motion_block->pos[E_AXIS],   TRAMS_E0AXIS);  // target
      
      // Let's wake up back when this movement is over
      timerClk = current_motion_block->nextTimerClk;

      // Discard the current_block and let st_calculate start with the next one
      current_motion_block->calcready = false;
      discard_current_motion_block();
    } else {
      timerClk = 2000; // Check in a short time, if a new block is calculated
    }
  }

  // Update 32-bit software-extended timer
  if(timerClk >= 65535) {
    timerClk = timerClk - 65535;
    TCNT1 = 1;
  } else {
    temp_timer = TCNT1;
    if(timerClk > (temp_timer + 100))
      TCNT1 = 65535 - timerClk + temp_timer;
    else
      TCNT1 = 65436;

    timerClk = 0;
  }
}

void Trams::init() {
  TMC5130_init( TRAMS_XAXIS,  X_CURRENT_RUN,  X_CURRENT_HOLD,  STEPPER_DIRECTION_X);
  TMC5130_init( TRAMS_YAXIS,  Y_CURRENT_RUN,  Y_CURRENT_HOLD,  STEPPER_DIRECTION_Y);
  TMC5130_init( TRAMS_ZAXIS,  Z_CURRENT_RUN,  Z_CURRENT_HOLD,  STEPPER_DIRECTION_Z);
  TMC5130_init(TRAMS_E0AXIS, E0_CURRENT_RUN, E0_CURRENT_HOLD,  STEPPER_DIRECTION_E0);
  Stepper::init();
}

void Trams::set_position(const long &a, const long &b, const long &c, const long &e) {

  synchronize(); // Bad to set stepper counts in the middle of a move

  CRITICAL_SECTION_START;

  if(pos[X_AXIS] != a) {
    count_position[X_AXIS] = pos[X_AXIS] = a;
    motion_buffer_block_old.pos[X_AXIS] = a;
    // Update position in the driver
    spi_writeRegister(RAMPMODE, HOLD_MODE, TRAMS_XAXIS);  //HOLD Mode
    spi_writeRegister(XTARGET, a, TRAMS_XAXIS);
    spi_writeRegister(XACTUAL, a, TRAMS_XAXIS);
    spi_writeRegister(RAMPMODE, POSITIONING_MODE, TRAMS_XAXIS); //Position MODE
  }
  if(pos[Y_AXIS] != b) {
    count_position[Y_AXIS] = pos[Y_AXIS] = b;
    motion_buffer_block_old.pos[Y_AXIS] = b;
    // Update position in the driver
    spi_writeRegister(RAMPMODE, HOLD_MODE, TRAMS_YAXIS);  //HOLD Mode
    spi_writeRegister(XTARGET, b, TRAMS_YAXIS);
    spi_writeRegister(XACTUAL, b, TRAMS_YAXIS);
    spi_writeRegister(RAMPMODE, POSITIONING_MODE, TRAMS_YAXIS); //Position MODE
  }
  if(pos[Z_AXIS] != c) {
    count_position[Z_AXIS] = pos[Z_AXIS] = c;
    motion_buffer_block_old.pos[Z_AXIS] = c;
    // Update position in the driver
    spi_writeRegister(RAMPMODE, HOLD_MODE, TRAMS_ZAXIS);  //HOLD Mode
    spi_writeRegister(XTARGET, c, TRAMS_ZAXIS);
    spi_writeRegister(XACTUAL, c, TRAMS_ZAXIS);
    spi_writeRegister(RAMPMODE, POSITIONING_MODE, TRAMS_ZAXIS); //Position MODE
  }
  if(pos[E_AXIS] != e) {
    count_position[E_AXIS] = pos[E_AXIS] = e;
    motion_buffer_block_old.pos[E_AXIS] = e;
    // Update position in the driver
    spi_writeRegister(RAMPMODE, HOLD_MODE, TRAMS_E0AXIS); //HOLD Mode
    spi_writeRegister(XTARGET, e, TRAMS_E0AXIS);
    spi_writeRegister(XACTUAL, e, TRAMS_E0AXIS);
    spi_writeRegister(RAMPMODE, POSITIONING_MODE, TRAMS_E0AXIS);  //Position MODE
  }

  CRITICAL_SECTION_END;
}

void Trams::set_position(const AxisEnum &axis, const long &v) {
  CRITICAL_SECTION_START;
  count_position[axis] = v;
  if(pos[axis] != v) {
    count_position[axis] = pos[axis] = v;
    motion_buffer_block_old.pos[axis] = v;
    //Bit in PORTL which function as Slave Select PIN for an axis
    uint8_t axis_bit;
    switch(axis) {
      case X_AXIS: axis_bit = TRAMS_XAXIS; break;
      case Y_AXIS: axis_bit = TRAMS_YAXIS; break;
      case Z_AXIS: axis_bit = TRAMS_ZAXIS; break;
      case E_AXIS: axis_bit = TRAMS_E0AXIS; break;
    }
    // Update position in the driver
    spi_writeRegister(RAMPMODE, HOLD_MODE, axis_bit); //HOLD Mode
    spi_writeRegister(XTARGET, v, axis_bit);
    spi_writeRegister(XACTUAL, v, axis_bit);
    spi_writeRegister(RAMPMODE, POSITIONING_MODE, axis_bit);  //Position MODE
  }
  CRITICAL_SECTION_END;
}

void Trams::set_e_position(const long &e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  if(pos[E_AXIS] != e) {
    count_position[E_AXIS] = pos[E_AXIS] = e;
    motion_buffer_block_old.pos[E_AXIS] = e;
    // Update position in the driver
    spi_writeRegister(RAMPMODE, HOLD_MODE, TRAMS_E0AXIS); //HOLD Mode
    spi_writeRegister(XTARGET, e, TRAMS_E0AXIS);
    spi_writeRegister(XACTUAL, e, TRAMS_E0AXIS);
    spi_writeRegister(RAMPMODE, POSITIONING_MODE, TRAMS_E0AXIS);  //Position MODE
  }
  CRITICAL_SECTION_END;
}

void Trams::set_directions() {

  #define SET_STEP_DIR(AXIS) \
    if (motor_direction(AXIS ##_AXIS)) { \
      AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = -1; \
    } \
    else { \
      AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = 1; \
    }

  #if HAS_X_DIR
    SET_STEP_DIR(X); // A
  #endif
  #if HAS_Y_DIR
    SET_STEP_DIR(Y); // B
  #endif
  #if HAS_Z_DIR
    SET_STEP_DIR(Z); // C
  #endif

  #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    if (motor_direction(E_AXIS)) {
      //REV_E_DIR();
      count_direction[E_AXIS] = -1;
    }
    else {
      //NORM_E_DIR();
      count_direction[E_AXIS] = 1;
    }
  #endif // !ADVANCE && !LIN_ADVANCE
}

