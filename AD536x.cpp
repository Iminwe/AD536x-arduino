/*
  File Name:      AD536x.cpp
  Description:    AD536x family DAC control library for Arduino
                  Analog devices AD5360, AD5361, AD5362, AD5363
  Author:         Itahisa Hernández Fumero
  Created:        Based on https://github.com/JQIamo/AD536x-arduino
  Last Modified:  2023-06-28
  License:        GNU-GPL
*/


#include "AD536x.h"

// Constructor
// some parameters related to the particular hardware implementation

//The physical pin used to enable the SPI device
//#define SPI_DEVICE 4

//The clock frequency for the SPI interface
//#define AD536x_CLOCK_DIVIDER_WR SPI_CLOCK_DIV2
//#define AD536x_CLOCK_DIVIDER_RD SPI_CLOCK_DIV4 //that (Assuming and Arduino clocked at 80MHz will set the clock of the SPI to 20 MHz
                         //AD536x can operate to up to 50 MHz for write operations and 20MHz for read operations.

// constructor...
AD536x::AD536x(int cs, int clr, int ldac, int reset) {

  _sync   = cs;
  _clr    = clr;
  _ldac   = ldac;
  _reset  = reset;
    
  // Default to 5V reference... can change with setGlobalVref[bank]
  _vref[0] = 5.0;
  _vref[1] = 5.0;
  
  // make pins output, and initialize to startup state
  pinMode(_sync,  OUTPUT);
  pinMode(_clr,   OUTPUT);
  pinMode(_ldac,  OUTPUT);
  pinMode(_reset, OUTPUT);
  
  digitalWrite(_sync,   HIGH);
  digitalWrite(_ldac,   HIGH);
  digitalWrite(_clr,    HIGH);
  digitalWrite(_reset,  HIGH);
  
  AD536x::reset();
/*	
  SPI.begin();
  
  // want to do this better long-term; AD536x can handle 50MHz clock though.
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);*/

}

// Public Methods
/*********************************************/

/**************************
    DAC functions
***************************/
void AD536x::writeDAC(AD536x_bank_t bank, AD536x_ch_t ch, unsigned int data){
  AD536x::write(DAC, bank, ch, data);
  AD536x::IOUpdate();	
}

void AD536x::writeDACHold(AD536x_bank_t bank, AD536x_ch_t ch, unsigned int data){
  AD536x::write(DAC, bank, ch, data);
}

unsigned int AD536x::getDAC(AD536x_bank_t bank, AD536x_ch_t ch){
  return _dac[bank][ch];
}

void AD536x::setVoltage(AD536x_bank_t bank, AD536x_ch_t ch, double voltage){
  unsigned int data = AD536x::voltageToDAC(bank, ch, voltage);
  AD536x::writeDAC(bank, ch, data);
}

void AD536x::setVoltageHold(AD536x_bank_t bank, AD536x_ch_t ch, double voltage){
  unsigned int data = AD536x::voltageToDAC(bank, ch, voltage);
  AD536x::writeDACHold(bank, ch, data);
}

/**************************
    Offset functions
***************************/
void AD536x::writeOffset(AD536x_bank_t bank, AD536x_ch_t ch, unsigned int data){
  AD536x::write(OFFSET, bank, ch, data);
  AD536x::IOUpdate();
}

unsigned int AD536x::getOffset(AD536x_bank_t bank, AD536x_ch_t ch){
  return _offset[bank][ch];
}

/**************************
    Gain functions
***************************/
void AD536x::writeGain(AD536x_bank_t bank, AD536x_ch_t ch, unsigned int data){
  AD536x::write(GAIN, bank, ch, data);
  AD536x::IOUpdate();
}

unsigned int AD536x::getGain(AD536x_bank_t bank, AD536x_ch_t ch){
  return _gain[bank][ch];
}

/**************************
    Misc functions
***************************/
void AD536x::IOUpdate(){
  digitalWrite(_ldac, LOW);
  // delay(1);
  digitalWrite(_ldac, HIGH);
}

void AD536x::reset(){
  digitalWrite(_reset, LOW);
  //delay(1);
  digitalWrite(_reset, HIGH);
  
  // reset DAC, OFFSET, GAIN to default values
  for (int c = 0; c < AD536x_MAX_CHANNELS; c++){
    _dac[0][c] = AD536x_DEFAULT_DAC;
    _dac[1][c] = AD536x_DEFAULT_DAC;
    
    _offset[0][c] = AD536x_DEFAULT_OFFSET;
    _offset[1][c] = AD536x_DEFAULT_OFFSET;
    
    _gain[0][c] = AD536x_DEFAULT_GAIN;
    _gain[1][c] = AD536x_DEFAULT_GAIN;
    
    _globalOffset[0] = AD536x_DEFAULT_GLOBALOFFSET;
    _globalOffset[1] = AD536x_DEFAULT_GLOBALOFFSET;
    
    // resets max/min boundaries.
    _max[0][c] = AD536x_DEFAULT_MAX;
    _min[0][c] = AD536x_DEFAULT_MIN;
    
    _max[1][c] = AD536x_DEFAULT_MAX;
    _min[1][c] = AD536x_DEFAULT_MIN;
  }
}

void AD536x::assertClear(int state){
  switch (state){
    case 1:
      digitalWrite(_clr, HIGH);
      break;
    case 0:
      digitalWrite(_clr, LOW);
      break;
    default:
      break;
  }
}

void AD536x::writeGlobalOffset(AD536x_bank_t bank, unsigned int data){
  
  unsigned long cmd = 0;
  data = data & 0x3FFF; 	// 14-bit mask
  switch (bank) {
    case BANK0:
      cmd = (cmd | AD536x_WRITE_OFS0 | data);
      _globalOffset[0] = data;
      break;
    case BANK1:
      cmd = (cmd | AD536x_WRITE_OFS1 | data);
      _globalOffset[1] = data;
      break;
    default:
      // bad bank; return early
      return;
  }
      
  // Make sure you assert clear while adjusting range to avoid glitches
  // see datasheet
  
  // AD536x::assertClear(0);
  AD536x::writeCommand(cmd);
  //AD536x::assertClear(1);

}

unsigned int AD536x::getGlobalOffset(AD536x_bank_t bank){
  return _globalOffset[bank];
}

void AD536x::setGlobalVref(AD536x_bank_t bank, double voltage){
  switch (bank) {
    case BANK0:
      _vref[0] = voltage;
      break;
    case BANK1:
      _vref[1] = voltage;
      break;
    default:
      break;
  }
}

double AD536x::getGlobalVref(AD536x_bank_t bank){
  return _vref[bank];
}

void AD536x::writeCommand(unsigned long cmd){

  digitalWrite(_sync, LOW);
  
  // write MSBFIRST
  
  SPI.transfer((cmd >> 16) & 0xFF);
  SPI.transfer((cmd >> 8) & 0xFF);
  SPI.transfer(cmd & 0xFF);
  
  digitalWrite(_sync, HIGH);
}

/**************************
    Advanced functions
***************************/
void AD536x::getReg (AD536x_reg_readback_t reg, AD536x_ch_t ch){
  unsigned long cmd = 0;

  switch (reg) {
    case X1A:
      cmd = cmd | AD536x_READ_REG | AD536x_READ_X1A(ch);
      break;
    case X1B:
      cmd = cmd | AD536x_READ_REG | AD536x_READ_X1B(ch);
      break;
    case C:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_C(ch));
      break;
    case M:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_M(ch));
      break;
    case CR:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_CR);
      break;
    case OFS0:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_OFS0);
      break;
    case OFS1:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_OFS1);
      break;
    case AB0:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_AB_0);
      break;
    case AB1:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_AB_1);
      break;
    case GPIO:
      cmd = (cmd | AD536x_READ_REG | AD536x_READ_GPIO);
      break;
    default:
      return;
  }

  // Serial.println(cmd);
  AD536x::writeCommand(cmd);
  AD536x::IOUpdate();
  
}

void AD536x::selectX1AX1B (AD536x_reg_readback_t reg){

  unsigned long cmd = 0;

  switch (reg) {
    case X1A:
      cmd = cmd | AD536x_WR_CR | AD536x_X1A;
      break;
    case X1B:
      cmd = cmd | AD536x_WR_CR | AD536x_X1B;
      break;
    default:
      return;
  }

  AD536x::writeCommand(cmd);
  // AD536x::IOUpdate();

}

void AD536x::selectX2AX2B (AD536x_reg_use_t reg){

  unsigned long cmd1 = 0;
  unsigned long cmd2 = 0;
  unsigned long X2A_use =   0 & 0xFF; //8 bits mask
  unsigned long X2B_use =  255 & 0xFF; //8 bits mask

  switch (reg) {
    case X2A:
      cmd1 = cmd1 | AD536x_WRITE_AB_SELECT_0 | X2A_use;
      cmd2 = cmd2 | AD536x_WRITE_AB_SELECT_1 | X2A_use;
      break;
    case X2B:
      cmd1 = cmd1 | AD536x_WRITE_AB_SELECT_0 | X2B_use;
      cmd2 = cmd2 | AD536x_WRITE_AB_SELECT_1 | X2B_use;
      break;
    default:
      return;
  }

  AD536x::writeCommand(cmd1);
  AD536x::writeCommand(cmd2);
  // AD536x::IOUpdate();

}

void AD536x::blockWriteAB (AD536x_reg_use_t reg){

  unsigned long cmd = 0;
  unsigned long X2A_use =   0 & 0xFF; //8 bits mask
  unsigned long X2B_use = 255 & 0xFF; //8 bits mask

  switch (reg) {
    case X2A:
      cmd = cmd | AD536x_BLOCK_WRITE_AB_SELECT | X2A_use;
      break;
    case X2B:
      cmd = cmd | AD536x_BLOCK_WRITE_AB_SELECT | X2B_use;
      break;
    default:
      return;
  }
  
  AD536x::writeCommand(cmd);
  // AD536x::IOUpdate();

}

// Private Methods
/*********************************************/

void AD536x::write(AD536x_reg_t reg, AD536x_bank_t bank, AD536x_ch_t ch, unsigned int data){
  data = data & AD536x_DATA_MASK; 	// bitmask ensure data has proper resolution
  
  unsigned int payload;
  
  // if 14 bit DAC, coerce to right form
  #ifdef AD536x_14BIT
    payload = (data << 2) & 0xFFFF;
  #else
    payload = data;
  #endif
  
  // pointer for where to store channel data 
  // for reference.
  // see: http://stackoverflow.com/questions/21488179/c-how-to-declare-pointer-to-2d-array
  unsigned int  (*localData)[2][AD536x_MAX_CHANNELS]; 	
                
  unsigned long cmd = 0;		// var for building command.
    
  // for validation purposes...
  // don't want to validate for entire bank writes.
  // (in the sense that it's too costly)		
  #ifdef AD536x_VALIDATE
    bool singleCh = false;
  #endif
  
  // add register header M1, M0	
  switch (reg) {
    case DAC:
      cmd = cmd | AD536x_WRITE_DAC;
      localData = &_dac;
      break;
    case OFFSET:
      cmd = cmd | AD536x_WRITE_OFFSET;
      localData = &_offset;
      break;
    case GAIN:
      cmd = cmd | AD536x_WRITE_GAIN;
      localData = &_gain;
      break;
    default:
      // bad register; return early.
      // might want to notify user...???
      return;
  }
  
  // check to make sure channel in range, if not, return early.
  if (ch > AD536x_MAX_CHANNELS && ch != CHALL){
    return;
  }
  
  if (ch == CHALL){
    // if writing all channels, figure out which bank to address
    // and update local reference data.
    switch (bank){
      case BANK0:
        cmd = cmd | AD536x_ALL_BANK0;
        for (int c = 0; c < AD536x_MAX_CHANNELS; c++){
          (*localData)[0][c] = data;
        }
        break;
      
      case BANK1:
        cmd = cmd | AD536x_ALL_BANK1;
        for (int c = 0; c < AD536x_MAX_CHANNELS; c++){
          (*localData)[1][c] = data;
        }
        break;
      
      case BANKALL:
        // all banks, all channels
        // address bits are zero, so do nothing to cmd.
          for (int c = 0; c < AD536x_MAX_CHANNELS; c++){
          (*localData)[0][c] = data;
          (*localData)[1][c] = data;
        }
        break;
        
      default:
        // not valid address, so return early
        // note, no way to (natively) address, eg, 
        // BANKALL, CH2
        return;
    }
  } else {
    
    // else, write particular bank/channel
    switch (bank){
      case BANK0:
        cmd = cmd | AD536x_BANK0 | ((unsigned long)ch << 16);
        (*localData)[0][ch] = data;
        
        #ifdef AD536x_VALIDATE
          singleCh = true;
        #endif
        
        break;
      
      case BANK1:
        cmd = cmd | AD536x_BANK1 | ((unsigned long) ch << 16);
        (*localData)[1][ch] = data;
        
        #ifdef AD536x_VALIDATE
          singleCh = true;
        #endif
        
        break;
      
      default:
        // not valid address
        // return early
        return;
    }
  }
  
  #ifdef AD536x_VALIDATE
    // if writing a single channel, validate your data.
    if(singleCh){
      int valid = AD536x::validateData(bank, ch, data);
      
      // if data out of range, return early
      // not sure how to best notify user of this.
      if (valid != 1){
        return;
      }
    }
  #endif
  // update command with data packet, and write to dac.
  cmd = cmd | payload;
  AD536x::writeCommand(cmd);
}

unsigned int AD536x::voltageToDAC(AD536x_bank_t bank, AD536x_ch_t ch, double voltage){
    
  if (AD536x_RESOLUTION == 16){  
    /* 	
      Transfer function:	
      VOUT = 4*VREF*(DAC_CODE - (4*OFFSET_CODE))/2^16)
      DAC_CODE = data*(M+1)/2^16 + (C - 2^15)
    */
    double m;
    // double m = (double)(_gain[bank][ch] - AD536x_DEFAULT_GAIN);
    if ((_gain[bank][ch]) != AD536x_DEFAULT_GAIN){
      m = (double)(_gain[bank][ch] - AD536x_DEFAULT_GAIN);
    } else {
      m = (double)(AD536x_DEFAULT_GAIN);
    }
    double mm = (m + 1)/pow(2,16);

    double c;
    // double c = (double) (_offset[bank][ch] - AD536x_DEFAULT_OFFSET);
    if ((_offset[bank][ch]) != AD536x_DEFAULT_OFFSET){
      c = (double) (_offset[bank][ch] - AD536x_DEFAULT_OFFSET);
    } else {
      c = (double) (AD536x_DEFAULT_OFFSET);
    }
    double cc = c - pow(2,15);

    double d = (voltage*pow(2,16)/(4*_vref[bank])) + 4*(double)_globalOffset[bank]; //DAC_CODE

    unsigned int data = (unsigned int)((d - cc)/mm); //INPUT_CODE in decimal number
    
    if (d >= AD536x_DEFAULT_MAX){
      data = AD536x_DEFAULT_MAX;
    }else if(d <= AD536x_DEFAULT_MIN){
      data = AD536x_DEFAULT_MIN;
    }

    return data;
  } else if (AD536x_RESOLUTION == 14){
    /* 	
      Transfer function:	
      VOUT = 4*VREF*(DAC_CODE - OFFSET_CODE)/2^14
      DAC_CODE = data*(M+1)/2^14 + (C - 2^13)
    */
    double m;
    // double m = (double)(_gain[bank][ch] - AD536x_DEFAULT_GAIN);
    if ((_gain[bank][ch]) != AD536x_DEFAULT_GAIN){
      m = (double)(_gain[bank][ch] - AD536x_DEFAULT_GAIN);
    } else {
      m = (double)(AD536x_DEFAULT_GAIN);
    }
    double mm = (m + 1)/pow(2,14);

    double c;
    // double c = (double) (_offset[bank][ch] - AD536x_DEFAULT_OFFSET);
    if ((_offset[bank][ch]) != AD536x_DEFAULT_OFFSET){
      c = (double) (_offset[bank][ch] - AD536x_DEFAULT_OFFSET);
    } else {
      c = (double) (AD536x_DEFAULT_OFFSET);
    }
    double cc = c - pow(2,13);

    double d = (voltage*pow(2,14)/(4*_vref[bank])) + (double)_globalOffset[bank]; //DAC_CODE

    unsigned int data = (unsigned int)((d - cc)/mm); //INPUT_CODE in decimal number
    
    if (d >= AD536x_DEFAULT_MAX){
      data = AD536x_DEFAULT_MAX;
    }else if(d <= AD536x_DEFAULT_MIN){
      data = AD536x_DEFAULT_MIN;
    }

    return data;
  }

}

// ToDo
// doesn't handle 14-bit transfer function!!!!!
double AD536x::dacToVoltage(AD536x_bank_t bank, AD536x_ch_t ch, unsigned int data){
  //double dd = (double) data;
  // better way to do this...???
  double mm = ((double)_gain[bank][ch] + 1)/pow(2, 16);
  double dd = ((double)data * mm) + (double) _offset[bank][ch] - 0x8000;
  
  double vout = 4*_vref[bank]*dd/pow(2, 16) - ((double)_globalOffset[bank]/pow(2, 16));
  
  return vout;
}

int AD536x::validateData(AD536x_bank_t bank, AD536x_ch_t ch, unsigned int data){
  unsigned int max = _max[bank][ch];
  unsigned int min = _min[bank][ch];
  
  if (data <= max && data >= min){
    return 1;
  } else {
    return 0;
  }
}