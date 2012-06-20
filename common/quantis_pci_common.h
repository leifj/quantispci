/*
 * Quantis PCI driver
 *
 * Copyright (c) 2004-2010 id Quantique SA, Carouge/Geneva, Switzerland
 * All rights reserved.
 *
 * ----------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY.
 *
 * ----------------------------------------------------------------------------
 *
 * Alternatively, this software may be distributed under the terms of the
 * terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * ----------------------------------------------------------------------------
 *
 * For history of changes, ChangeLog.txt
 */

#ifndef QUANTIS_PCI_COMMON_H
#define QUANTIS_PCI_COMMON_H


#ifdef __linux__
# include <linux/types.h>
#endif /* __linux__ */


#ifdef __sun
# include <stdint.h>

/* Some specific include for some functions (memcpy for instance)
 * used in the interface are needed when compiling in kernel mode.
 */
# ifdef _KERNEL
#   include <sys/systm.h>
#   include <sys/cmn_err.h>
#   include <sys/ddi.h>
#   include <sys/sunddi.h>
# endif /* _KERNEL */
#endif /* __sun */

/**
 * This type is very specific to types describing the Quantis modules.
 * Only the bits 0-3 of the integer are used and they
 * represent Quantis modules (they can be at most 4 modules per card)
 */
typedef uint32_t quantis_module_mask;

/**
 * Type describing the values used to select a register of a Quantis board.
 * A register is alway described by its offset (expressed in bytes) from the
 * base of the registers. Since a Quantis register contains always unsigned
 * value of 32 bits, a value of this type should always be divisible 4.
 */
typedef uint32_t quantis_register;

/**
 * This is the type of the value contained in a Quantis register
 * (which is always an unsigned 32 bits value).
 */
typedef uint32_t quantis_register_value;


/**
 * Quantis PCI Registers
 *
 * The Quantis PCI device has a memory register page 16 words long,
 * consisting of 10 registers. These registers are split into the
 * following functional groups:
 *  -> Core Control group:
 *      - Enable Register `CC_ER'
 *      - Disable Register `CC_DR' 
 *      - Status Register `CC_SR'.
 *  -> Core Version group:
 *      - Status Register `CV_SR', which is used to get the board version.
 *  -> FIFO Status group:
 *      - FIFO Flush Register `FS_CA', 
 *      - FIFO Status Register `FS_RR'
 *      - FIFO Data Read Register `FD_RR'.
 *  -> Module Group:
 *      - Enable Register `MX_ER'
 *      - Disable Register `MX_DR'
 *      - Status Register `MX_SR'
 *     used to enable and disable card modules,
 *  -> Interrupt group
 *      - Enable Register `IR_ER'
 *      - Disable Register `IR_DR'
 *      - Status Register `IR_SR'
 *     used to enable or disable interrupt for specific conditions
 *
 * All the other registers are "reserved" and should not be used.
 **/
#define CC_ER   0     /* Core enable */
#define CC_DR   4     /* Core disable */
#define CC_SR   12    /* Core status */
#define CV_SR   28    /* Core version  */
#define FS_CA   32    /* FIFO flush */
#define FS_RR   40    /* FIFO status */
#define FD_RR   44    /* FIFO data */
#define MX_ER   48    /* Module enable */
#define MX_DR   52    /* Module disable */
#define MX_SR   60    /* Module status */
#define IR_ER   64    /* Interrupts enable */
#define IR_DR   68    /* Interrupts disable */
#define IR_SR   76    /* Interrupts status */

/**
 * Module's register are composed of four 1-byte for each of the (possible)
 * module.
 * 
 *  31     24 23     16 15      8 7       0   bits
 * +---------+---------+---------+---------+
 * | Module3 | Module2 | Module1 | Module0 |
 * +---------+---------+---------+---------+
 */ 
#define MX_SD   0     /* Software shutdown */
#define MX_SEN  1     /* Software enable */
#define MX_TM   2     /* Test mode enable */
#define MX_HEN  6     /* Hardware enable */
#define MX_HST  7     /* Hardware status */


/**
 * The FIFO of the random number generator is 4096 bytes big. It can be either
 * empty, 1/4 full, half-full, 3/4 full or completely full.
 */
#define QUANTIS_FIFO_SIZE     4096

#define QUANTIS_FIFO_EMPTY    1
#define QUANTIS_FIFO_FL1      (1 << 1)
#define QUANTIS_FIFO_FL2      (1 << 2)
#define QUANTIS_FIFO_FL3      (1 << 3)
#define QUANTIS_FIFO_FULL     (1 << 6)
#define QUANTIS_FIFO_ERROR    (1 << 7)


/** Size in bytes of all hardware registers */
#define QUANTIS_REG_LENGTH (16 * 4)

/**
 * Size of the internal buffer used to read random data.
 * This corresponds to the maximal length that can be read by quantis_rng_read
 * at once.
 * @see quantis_rng_read
 */
#define QUANTIS_DEVICE_BUFFER_SIZE  (16 * QUANTIS_FIFO_SIZE)

/**
 * Structure representing a Quantis PCI device. This is an opaque type
 * that must be defined in quantis_pci_MYOS.c
 */
typedef struct quantis_pci_device quantis_pci_device;


/**
 * Get the value of a register. This function is OS-specific and must be 
 * implemented in quantis_pci_MYOS.c
 * @param qdev 
 * @param reg
 * @return the value of a register
 */
extern quantis_register_value quantis_reg_get(quantis_pci_device* qdev, quantis_register reg);


/**
 * Set the value of a register. This function is OS-specific and must be
 * implemented in quantis_pci_MYOS.c
 * @param qdev
 * @param reg
 * @param value
 */
extern void quantis_reg_set(quantis_pci_device* qdev, quantis_register reg, quantis_register_value value);


/* Value to detect the different module in a mask */
#define MODULE0 (1 << 0)
#define MODULE1 (1 << 1)
#define MODULE2 (1 << 2)
#define MODULE3 (1 << 3)

/**
 * 
 * @param mask
 * @param type
 * @return
 */
static inline quantis_register_value mask2reg(quantis_module_mask mask, int type)
{
  quantis_register_value reg = (mask & MODULE0 ? 1 << 0  : 0)
                             | (mask & MODULE1 ? 1 << 8  : 0)
                             | (mask & MODULE2 ? 1 << 16 : 0)
                             | (mask & MODULE3 ? 1 << 24 : 0);
  return reg << type;
}


/**
 *
 * @param reg
 * @param type
 * @return
 */
static inline quantis_module_mask reg2mask(quantis_register reg, int type)
{
  reg >>= type;
  return (reg & (1 << 0)  ? MODULE0 : 0)
    |    (reg & (1 << 8)  ? MODULE1 : 0)
    |    (reg & (1 << 16) ? MODULE2 : 0)
    |    (reg & (1 << 24) ? MODULE3 : 0);
}


/**
 * Returns the FIFO status
 * @param qdev
 * @return
 */
static inline quantis_register_value quantis_get_fifo_status(quantis_pci_device* qdev)
{
  return quantis_reg_get(qdev, FS_RR);
}


/**
 * Flush the FIFO.
 * @param qdev
 */
static inline void quantis_flush_fifo(quantis_pci_device* qdev)
{
  quantis_reg_set(qdev, FS_CA, 0);
}


/**
 *
 * @param qdev
 * @return 1 if any module is not correctly working, 0 otherwise
 */
static inline int quantis_rng_error(quantis_pci_device* qdev)
{
  quantis_register_value reg = quantis_reg_get(qdev, MX_SR);
  quantis_module_mask test   = reg2mask(reg, MX_TM);
  quantis_module_mask status = reg2mask(reg, MX_HST);
  quantis_module_mask enable = reg2mask(reg, MX_SEN);

  if (test)
  {
    return 0;
  }
  else
  {
    return (enable & status) == 0;
  }
}


/**
 * Returns the number of bytes of random data available in the FIFO.
 * @param qdev
 * @return the number of bytes of random data available in the FIFO.
 */
static inline int quantis_rng_fifo_bytes(quantis_pci_device* qdev)
{
  quantis_register_value status = quantis_get_fifo_status(qdev);

  /* */
#ifdef __linux__
  static int once = 1;

  if (once)
  {
      schedule();
      once = 0;
  }
#endif

  if (status & QUANTIS_FIFO_FULL)
  {
    return QUANTIS_FIFO_SIZE;
  }
  else if (status & QUANTIS_FIFO_FL3)
  {
    return (QUANTIS_FIFO_SIZE / 4 * 3);
  }
  else if (status & QUANTIS_FIFO_FL2)
  {
    return (QUANTIS_FIFO_SIZE / 2);
  }
  else if (status & QUANTIS_FIFO_FL1)
  {
    return (QUANTIS_FIFO_SIZE / 4);
  }
  else
  {
    return 0;
  }
}


/**
 * Wait until the FIFO has some random data available. This 
 * 
 * @return the number of bytes of random data available in the FIFO or a 
 * negative value on timeout.
 */
static inline int quantis_rng_wait_fifo(quantis_pci_device* qdev)
{
  unsigned int timeout = 10000;
  unsigned int available_bytes = 0;

  while(timeout > 0)
  {
    available_bytes = quantis_rng_fifo_bytes(qdev);
    if (available_bytes > 0)
    {
      return available_bytes;
    }
    timeout--;

    // Add sleep or use the interrupt of the cards
    // Active sleep is bad, while passive sleep (like relinquish
    // the CPU for a tick) is too much time, so there isn't a solution.

    // TODO
    //#warning "TODO: add sleep" ?
    //#include <unistd.h>
    //usleep(ms*1000); //convert to microseconds
    //usleep(100); // avoid 100% CPU usage

    // In kernel
    //#include <linux/delay.h>
    //void udelay(unsigned long usecs);
    //void mdelay(unsigned long msecs);
    //The functions introduce delays of an integer number of microseconds and milliseconds. The former should be used to wait for no longer than one millisecond; the latter should be used with extreme care because these delays are both busy-loops.

  }

  /* Wait timed out*/
  return -1;
}


/**
 *
 */
static inline void quantis_rng_reset(quantis_pci_device* qdev)
{
  quantis_reg_set(qdev, CC_ER, 1);
  quantis_reg_set(qdev, CC_DR, 1);
}


/**
 * Returns the core version
 */
static inline quantis_register_value quantis_rng_version(quantis_pci_device* qdev)
{
  return quantis_reg_get(qdev, CV_SR);
}




/**
 * Enable module(s) speficified by mask
 * @param qdev
 * @param mask
 */
static inline void quantis_rng_enable_modules(quantis_pci_device* qdev, quantis_module_mask mask)
{
  /* enable modules */
  quantis_reg_set(qdev, MX_ER, mask2reg(mask, MX_SEN));

  /* Flush FIFO */
  quantis_flush_fifo(qdev);
}


/**
 * Disable module(s) speficified by mask
 * @param qdev
 * @param mask
 */
static inline void quantis_rng_disable_modules(quantis_pci_device* qdev, quantis_module_mask mask)
{
  /* Disable modules */
  quantis_reg_set(qdev, MX_DR, mask2reg(mask, MX_SEN));

  /* Flush FIFO */
  quantis_flush_fifo(qdev);
}


/**
 *
 * @param qdev
 * @return
 */
static inline quantis_module_mask quantis_rng_modules_status(quantis_pci_device* qdev)
{
  return reg2mask(quantis_reg_get(qdev, MX_SR), MX_SEN)
      &  reg2mask(quantis_reg_get(qdev, MX_SR), MX_HST);
}


/**
 * Return the modules presents on the device as a mask of 4 bits.
 * @param qdev
 * @return
 */
static inline quantis_module_mask quantis_rng_modules_mask(quantis_pci_device* qdev)
{
  return reg2mask(quantis_reg_get(qdev, MX_SR), MX_HEN);
}


/**
 * Fills buffer with random bytes.
 * @param qdev
 * @param buffer a pointer to a buffer which size is at least
 * QUANTIS_DEVICE_BUFFER_SIZE.
 * @param length the requested number of bytes to fill into the buffer.
 * @return the number of random bytes filled into the buffer or a negative
 * value on error.
 * @warning if (length > QUANTIS_DEVICE_BUFFER_SIZE), only
 * QUANTIS_DEVICE_BUFFER_SIZE will be filled!
 */
static inline int quantis_rng_read(quantis_pci_device* qdev,
                                   unsigned char* buffer,
                                   int length)
{
  int read_bytes = 0;

  /* Verify at least one module is enabled and everything works correctly */
  if (quantis_rng_error(qdev))
  {
    /* Module status error (is at least one module enabled? */
    return -1;
  }

  /* Read random bytes */
  while (read_bytes < length)
  {
    /* Get the number of bytes available on the FIFO */
    int available_bytes = quantis_rng_wait_fifo(qdev);
    if (available_bytes < 0)
    {
      /* Timeout while waiting random bytes */
      return -2;
    }
    else if (available_bytes > (length - read_bytes))
    {
      available_bytes = (length - read_bytes);
    }

    while (available_bytes > 0)
    {
      int random_data_length = 4;
      //       /* this can potentially cause endianness problems */
      quantis_register_value random_data = quantis_reg_get(qdev, FD_RR);
      quantis_register_value fifo_status = quantis_get_fifo_status(qdev);

      if (fifo_status & QUANTIS_FIFO_ERROR)
      {
        /* The FIFO has overflown, reset it */
        quantis_flush_fifo(qdev);
        available_bytes = 0;
        break;
      }

      /* Checks we don't read too much data */
      if (random_data_length > available_bytes)
      {
        random_data_length = available_bytes;
      }

      /* Avoid buffer overflow */
      if ((read_bytes + random_data_length) > QUANTIS_DEVICE_BUFFER_SIZE)
      {
        return read_bytes;
      }

      /* copy random data to the buffer */
      memcpy(buffer + read_bytes, &random_data, random_data_length);
      available_bytes -= random_data_length;
      read_bytes += random_data_length;
    } // while (available_bytes > 0)
  } // while (read_bytes < length)

  return read_bytes;
}

/**
 * Only quantis_pci_common's functions are allowed to directly access the 
 * registers. We thus undefine "local" defines...
 */
#undef CC_ER
#undef CC_DR
#undef CC_SR
#undef CV_SR
#undef FS_CA
#undef FS_RR
#undef FD_RR
#undef MX_ER
#undef MX_DR
#undef MX_SR

#undef MX_SD
#undef MX_SEN
#undef MX_TM
#undef MX_HEN
#undef MX_HST


#endif
