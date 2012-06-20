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


#include <sys/types.h>
#include <sys/file.h>
#include <sys/errno.h>
#include <sys/conf.h>
#include <sys/modctl.h>
#include <sys/open.h>
#include <sys/stat.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>

#include "quantis_pci.h"
#include "quantis_pci_common.h"

/********************************* Constants *********************************/


/* Keep enough character for UTF-8 msg */
#define MAX_MSG_LEN 512

/*
 * Macros to display info, warning, and error messages from the drivers.
 * The message should be completely defined and ready to be printed (without
 * any variable substitutions to do.
 *
 * A single line message may be produced by constructing the message with
 * sprintf(9F) or vsprintf(9F) before calling cmn_err().
 *
 * The prototype of sprintf is included in "sys/ddi.h", do not include
 * "stdio.h"
 */

/* Information */
#define QUANTIS_INFO(msg)  cmn_err(CE_CONT,(msg))

/*
 * A problem with the quantis card, is just a note to the sys admin,
 * but not a big system problem
 */
#define QUANTIS_WARN(msg)  cmn_err(CE_NOTE,(msg))

/* An error with the Quantis card should not panic the system ! */
#define QUANTIS_ERROR(msg) cmn_err(CE_WARN,(msg))


/********************************* Prototypes *********************************
 *
 * These are the prototypes of our module functions.
 *
 */
static int quantis_getinfo(dev_info_t *, ddi_info_cmd_t, void *, void **);
static int quantis_attach(dev_info_t *, ddi_attach_cmd_t);
static int quantis_detach(dev_info_t *, ddi_detach_cmd_t);

static int quantis_open(dev_t *, int, int, cred_t *);
static int quantis_close(dev_t, int, int, cred_t *);
static int quantis_read(dev_t, struct uio *, cred_t *);
static int quantis_ioctl(dev_t, int, intptr_t, int, cred_t *, int *);

/**
 * Most device drivers maintain state information with each instance
 * of the device they control. We store the soft state of our devices
 * in a `quantis_soft_state_t' structure, which for now contains a
 * pointer to the mapped I/O registers of the quantis card, a pointer
 * to the device information structure and a mutex to lock accesses
 * to the device.
 */
struct quantis_pci_device
{
  dev_info_t *dip;
  kmutex_t mutex;

  unsigned char *regs;
  ddi_acc_handle_t regs_handle;

  unsigned char buffer[QUANTIS_DEVICE_BUFFER_SIZE];
  unsigned int  cnt;
};

/** Synonym for the opaque type defined in common file. */
typedef quantis_pci_device quantis_soft_state_t;

#define QUANTIS_DEBUG2 LOG_DEBUG2
#define QUANTIS_DEBUG0 LOG_DEBUG0

/**
 * Quantis PCI registers structure. The Quantis Card has 3 PCI
 * base address registers (bar). The second one contains
 * the address to be used.
 *
 * This structure is mapped using `ddi_regs_map_setup' at attach time.
 */
#define QUANTIS_REG_IDX 2



/**
 * Device driver soft state
 *
 * This global variable is modified only during the load and unload
 * of module (functions "_init", "_fini", "_info") where no race
 * condition can occur (when used elsewhere the value is read-only).
 * No mutex is then mandatory to manage the variable.
 */ 
static void* quantis_soft_state_p;

/**
 * Global cards count, which is protected by a global mutex.
 */
static kmutex_t quantis_mutex;
static int card_count;

/** Character device entry points **/
static struct cb_ops quantis_cb_ops =
{
  quantis_open,     /* open */
  quantis_close,    /* close */
  nodev,            /* strategy */
  nodev,            /* print */
  nodev,            /* dump: not used*/
  quantis_read,     /* read */
  nodev,            /* write */
  quantis_ioctl,    /* ioctl */
  nodev,            /* devmap: not used */
  nodev,            /* mmap */
  nodev,            /* segmap */
  nochpoll,         /* chpoll */
  ddi_prop_op,      /* prop_op */
  0,                /* aread ??*/

  /*
   * The `cb_flag' member contains the following flags: the `D_MP'
   * flag indicates that the driver is safe for multi-threading (this
   * flag must be set). If the driver properly handles 64-bit offsets, it
   * should set the `D_64BIT' flag (this specifies that the driver will
   * use the `uio_loffset' field of the `uio' structure. If the
   * driver supports the `devmap' entry point, it should set the
   * `D_DEVMAP' flag.
   */
  D_NEW | D_MP | D_64BIT,  /* flags */
};


static struct dev_ops quantis_ops =
{
  DEVO_REV,
  0,                  /* devo_refcnt */
  quantis_getinfo,    /* devo_getinfo */
  nulldev,            /* devo_identify */
  nulldev,            /* devo_probe */
  quantis_attach,     /* devo_attach */
  quantis_detach,     /* devo_detach */
  nodev,              /* devo_reset */
  &quantis_cb_ops,    /* devo_cb_ops */
  NULL,               /* devo_bus_ops */
  NULL,               /* devo_power */
};


/* Modlinkage and modldrv driver entry points */

static struct modldrv module_loader_info =
{
  &mod_driverops,
  QUANTIS_PCI_DRIVER_NAME,
  &quantis_ops,
};

static struct modlinkage modlinkage =
{
  MODREV_1,
  &module_loader_info,
  NULL,
};


/***************************** Global Functions  *****************************
 *
 * functions "_init", "_info", "_fini" are used by the dynamic loader to load
 * and unload the driver.
 */

/**
 * Initializes a loadable module. It calls `mod_install' to install the
 * `modlinkage' and `modldrv' structures.
 */
int _init(void)
{
  int error;
  error = ddi_soft_state_init(&quantis_soft_state_p,
                              sizeof(quantis_soft_state_t),
                              0);
  if (error != 0)
  {
    QUANTIS_ERROR("Could not initialize the soft state tree.\n");
    return error;
  }

  error = mod_install(&modlinkage);
  if (error != 0)
  {
    QUANTIS_ERROR("Could not install the modlinkage structure.\n");
    ddi_soft_state_fini(&quantis_soft_state_p);
    return error;
  }

  mutex_init(&quantis_mutex, NULL, MUTEX_DRIVER, NULL);
  card_count = 0;

  LOG_DEBUG0("Initialized the quantis driver\n");

  return error;
}

/**
 * Returns information about the loadable module by calling `mod_info'.
 */
int _info(struct modinfo *modinfop)
{
  LOG_DEBUG0("_info\n");

  return mod_info(&modlinkage, modinfop);
}

/**
 * Prepares a loadable module for unloading. If the module can be unloaded,
 * it calls `mod_remove'. When `_fini' is successful, no method in the driver
 * will be called before `_init' is called again.
 *
 * We release the soft state hanging structure at unloading time.
 */
int _fini(void)
{
  int error;

  error = mod_remove(&modlinkage);
  if (error != 0)
  {
    QUANTIS_ERROR("Could not remove the modlinkage structure\n");
    return error;
  }
  mutex_destroy(&quantis_mutex);
  ddi_soft_state_fini(&quantis_soft_state_p);
  return 0;
}


/* Implements functions to load and write Quantis registers */
quantis_register_value quantis_reg_get(quantis_pci_device* qdev,
                                       quantis_register reg)
{
  char msg[MAX_MSG_LEN];
  LOG_DEBUG1("In quantis_reg_get with reg=%d\n", reg);
  if (reg % 4 !=  0)
  {
    snprintf(msg,
             MAX_MSG_LEN,
             "Offset (%d) in the registers array is not divisible by 4. This could crash the driver.\n",
             reg);
    QUANTIS_WARN(msg);
  }
  return (quantis_register_value)ddi_get32(qdev->regs_handle,
                                           (quantis_register_value *)(qdev->regs + reg));
}

void quantis_reg_set(quantis_pci_device* qdev,
                     quantis_register reg,
                     quantis_register_value value)
{
  char msg[MAX_MSG_LEN];
  LOG_DEBUG2("In quantis_reg_set with reg=%d and value=%d\n", reg, value);
  if (reg % 4 !=  0)
  {
    snprintf(msg,
             MAX_MSG_LEN,
             "Offset (%d) in the registers array is not divisible by 4. This could crash the driver.\n",
             reg);
    QUANTIS_WARN(msg);
  }
  ddi_put32(qdev->regs_handle,
            (quantis_register_value *)(qdev->regs + reg),
            value);
}

/***************************** Local Functions  *****************************/

/**
 *
 */
static int quantis_getinfo(dev_info_t *dip,
                           ddi_info_cmd_t infocmd,
                           void *arg, void **result)
{
  int error;
  int instance = getminor((dev_t)arg);
  quantis_soft_state_t *soft_state;

  switch (infocmd)
  {
    case DDI_INFO_DEVT2DEVINFO:
      soft_state = (quantis_soft_state_t *)ddi_get_soft_state(quantis_soft_state_p, instance);
      if (soft_state == NULL)
      {
        *result = NULL;
        error = DDI_FAILURE;
      }
      else
      {
        mutex_enter(&soft_state->mutex);
        *result = soft_state->dip;
        mutex_exit(&soft_state->mutex);
        error = DDI_SUCCESS;
      }
      break;

    case DDI_INFO_DEVT2INSTANCE:
      *result = (void*)(unsigned long)instance;
      error = DDI_SUCCESS;
      break;

    default:
      *result = NULL;
      error = DDI_FAILURE;
  }

  return error;
}


/**
 * At attach time, we allocate the soft state structure for the current
 * instance of the device.
 */
static int quantis_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
  int instance;
  quantis_soft_state_t *soft_state;
  ddi_device_acc_attr_t dev_acc_attr; /* Hold the device access attributes. */
  int nregs;
  off_t regsize;
  char msg[MAX_MSG_LEN];

  LOG_DEBUG0("attach\n");

  switch (cmd)
  {
    case DDI_ATTACH:
      instance = ddi_get_instance(dip);
      snprintf(msg,
               MAX_MSG_LEN,
               "Attaching the Quantis device %d.\n",
               instance);
      LOG_DEBUG0(msg);

      /*
       * PCI devices are self-identifying devices, so we check that we
       * indeed have a Quantis QRNG card by checking that we have one
       * register page with the correct size.
       */
      if (ddi_dev_nregs(dip, &nregs) != DDI_SUCCESS)
      {
        snprintf(msg,
                 MAX_MSG_LEN,
                 "Could not get the number of register for the Quantis device %d.\n",
                 instance);
        QUANTIS_ERROR(msg);
        return DDI_FAILURE;
      }

      if (nregs < 4)
      {
        snprintf(msg,
                 MAX_MSG_LEN,
                 "The Quantis device %d has %d PCI base registers, but should have at least 4.\n",
                 instance,
                 nregs);
        QUANTIS_ERROR(msg);
        return DDI_FAILURE;
      }

      if (ddi_dev_regsize(dip, QUANTIS_REG_IDX, &regsize) != DDI_SUCCESS)
      {
        snprintf(msg,
                 MAX_MSG_LEN,
                 "Could not get the register size for the Quantis device %d.\n",
                 instance);
        QUANTIS_ERROR(msg);
        return DDI_FAILURE;
      }

      if (regsize < (int)QUANTIS_REG_LENGTH)
      {
        snprintf(msg,
                 MAX_MSG_LEN,
                 "The size of the Quantice device (%d) registers file is %d bytes long, "
                 "but should be at least %u bytes long.\n",
                 instance,
                 (int)regsize,
                 (unsigned int)QUANTIS_REG_LENGTH);
        QUANTIS_ERROR(msg);
        return DDI_FAILURE;
      }

      LOG_DEBUG0("After test of the validity of the card, before soft state alloc.\n");
      if (ddi_soft_state_zalloc(quantis_soft_state_p, instance) != DDI_SUCCESS)
      {
        snprintf(msg,
                 MAX_MSG_LEN,
                 "Could not allocate soft state structure for the Quantis device %d.\n",
                 instance);
        QUANTIS_ERROR(msg);
        return DDI_FAILURE;
      }

      soft_state = (quantis_soft_state_t *)ddi_get_soft_state(quantis_soft_state_p,
                                                              instance);
      soft_state->dip = dip;
      ddi_set_driver_private(dip, (caddr_t)soft_state);
      soft_state->cnt = 0;

      /*
       * Initialize the mutex in the soft state. We have no interrupt,
       * so we can set `arg' to `NULL'
       */
      mutex_init(&soft_state->mutex, NULL, MUTEX_DRIVER, NULL);

      if (ddi_create_minor_node(dip,
                                ddi_get_name(dip),
                                S_IFCHR,
                                instance,
                                DDI_PSEUDO,
                                0) == DDI_FAILURE)
      {
        snprintf(msg,
                 MAX_MSG_LEN,
                 "Could not create minor node for the Quantis device %d.\n",
                 instance);
        QUANTIS_ERROR(msg);
        mutex_destroy(&soft_state->mutex);
        ddi_soft_state_free(quantis_soft_state_p, instance);
        return DDI_FAILURE;
      }
      LOG_DEBUG1("ddi_get_name %s\n", ddi_get_name(dip));

      dev_acc_attr.devacc_attr_version = DDI_DEVICE_ATTR_V0;
      dev_acc_attr.devacc_attr_endian_flags = DDI_STRUCTURE_LE_ACC;
      dev_acc_attr.devacc_attr_dataorder = DDI_STRICTORDER_ACC;

      if (ddi_regs_map_setup(dip,
                             QUANTIS_REG_IDX,
                             (caddr_t *)&soft_state->regs,
                             0,
                             QUANTIS_REG_LENGTH,
                             &dev_acc_attr,
                             &soft_state->regs_handle) != DDI_SUCCESS)
      {
        snprintf(msg,
                 MAX_MSG_LEN,
                 "Could not map the registers space of the Quantis device %d.\n",
                 instance);
        QUANTIS_ERROR(msg);
        mutex_destroy(&soft_state->mutex);
        ddi_soft_state_free(quantis_soft_state_p, instance);
        return DDI_FAILURE;
      }

      mutex_enter(&quantis_mutex);
      card_count++;
      mutex_exit(&quantis_mutex);

      LOG_DEBUG0("Just before mutex\n");
      mutex_enter(&soft_state->mutex);
      LOG_DEBUG0("Just before rng_reset.\n");
      quantis_rng_reset(soft_state);
      LOG_DEBUG0("Just before enable_modules.\n");
      quantis_rng_enable_modules(soft_state, quantis_rng_modules_mask(soft_state));
      LOG_DEBUG0("Just before release mutex.\n");
      mutex_exit(&soft_state->mutex);

      snprintf(msg,
               MAX_MSG_LEN,
               "Successfully attached the Quantis device %d. Currently, %d Quantis cards are available.\n",
               instance,
               card_count);
      QUANTIS_INFO(msg);

#     ifdef DEBUG
        ddi_report_dev(dip);
#     endif

      return DDI_SUCCESS;

    case DDI_SUSPEND:
    case DDI_PM_SUSPEND:
      return DDI_SUCCESS;

    default:
      return DDI_FAILURE;
  }
}

/**
 * On cleanup, remove the minor node, unmade the register space, destroy the
 * mutex and free the soft state structure.
 */
static int quantis_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
  int instance;
  quantis_soft_state_t *soft_state;
  char msg[MAX_MSG_LEN];

  switch (cmd)
  {
    case DDI_DETACH:
      instance = ddi_get_instance(dip);
      snprintf(msg,
               MAX_MSG_LEN,
               "Detaching the Quantis device %d.\n",
               instance);
      QUANTIS_INFO(msg);
      soft_state = (quantis_soft_state_t*)ddi_get_soft_state(quantis_soft_state_p,
                                                             instance);
      ddi_remove_minor_node(dip, NULL);
      ddi_regs_map_free(&soft_state->regs_handle);
      mutex_destroy(&soft_state->mutex);
      ddi_soft_state_free(quantis_soft_state_p, instance);

      mutex_enter(&quantis_mutex);
      card_count--;
      mutex_exit(&quantis_mutex);

      return DDI_SUCCESS;

    case DDI_SUSPEND:
    case DDI_PM_SUSPEND:
      LOG_DEBUG1("Suspending dev %d\n", instance);
      return DDI_SUCCESS;

    default:
      return DDI_FAILURE;
  }
}
  
/**
 * We have nothing to initialize on file opening, so we just check that the
 * device really exists, and that it is opened as a character device.
 */
static int quantis_open(dev_t* dev, int flag, int otyp, cred_t* cred_p)
{
  quantis_soft_state_t* soft_state;

  LOG_DEBUG1("open dev %d\n", getminor(*dev));

  soft_state = (quantis_soft_state_t*)ddi_get_soft_state(quantis_soft_state_p,
                                                         getminor(*dev));
  if (soft_state == NULL)
  {
    return ENXIO;
  }

  if (otyp != OTYP_CHR)
  {
    return EINVAL;
  }

  return 0;
}

/**
 * In the same way, we have no resources to get rid off when a file is closed,
 * so just return a success.
 */
static int quantis_close(dev_t dev, int flag, int otyp, cred_t* cred_p)
{
  LOG_DEBUG1("close dev %d\n", getminor(dev));

  return 0;
}

/**
 * Reads data from the QRNG into the soft state buffer, and then transfer this
 * buffer to userland using `uiomove', which is the same `uio' abstraction as
 * in FreeBSD. 64-bit memory is managed by `uiomove'.
 */
static int quantis_read(dev_t dev, struct uio *uio, cred_t *credp)
{
  quantis_soft_state_t* soft_state;
  int ret;
  int toread, len;

  LOG_DEBUG2("read %d bytes from dev %d\n", uio->uio_resid, getminor(dev));

  soft_state = (quantis_soft_state_t*)ddi_get_soft_state(quantis_soft_state_p,
                                                         getminor(dev));
  mutex_enter(&soft_state->mutex);

  toread = min(uio->uio_resid, (int)sizeof(soft_state->buffer));
  len = quantis_rng_read(soft_state, soft_state->buffer, toread);
  LOG_DEBUG2("got %d bytes, max %d\n", len, toread);
  if (len < toread)
  {
    ret = ENXIO;
  }
  else
  {
    ret = uiomove(soft_state->buffer, len, UIO_READ, uio);
  }
  mutex_exit(&soft_state->mutex);
  return ret;
}

/**
 * File ioctl entry point
 *
 * We have some additional work to do in the ioctl call, as we have to support
 * both 32 bit and 64 bit userland programs. Data structures from userland are
 * converted using the `ddi_model_convert_from' function.
 */
static int quantis_copyin_uint(intptr_t arg, int flags, unsigned int* dst)
{
  uint32_t tmp32;

  switch (ddi_model_convert_from(flags & FMODELS))
  {
    case DDI_MODEL_ILP32:
      if (ddi_copyin((void*)arg, &tmp32, sizeof(tmp32), flags) < 0)
      {
        return -1;
      }
      *dst = tmp32;
      return 0;

    case DDI_MODEL_NONE:
      if (ddi_copyin((void*)arg, dst, sizeof(*dst), flags) < 0)
      {
        return -1;
      }
      return 0;

    default:
      return -1;
  }
}

static int quantis_copyout_uint(intptr_t arg, int flags, unsigned int src)
{
  uint32_t src32;

  switch (ddi_model_convert_from(flags & FMODELS))
  {
    case DDI_MODEL_ILP32:
      src32 = src;
      if (ddi_copyout(&src32,  (void *)arg, sizeof(src32), flags) < 0)
      {
        return -1;
      }
      return 0;

    case DDI_MODEL_NONE:
      if (ddi_copyout(&src,  (void *)arg, sizeof(src), flags) < 0)
      {
        return -1;
      }
      return 0;

    default:
      return -1;
  }
}

static int quantis_ioctl(dev_t dev,
                         int cmd,
                         intptr_t arg,
                         int flags,
                         cred_t* credp,
                         int* rvalp)
{
  quantis_soft_state_t* soft_state;
  int ret;

  LOG_DEBUG2("ioctl on dev %d, cmd %x\n", getminor(dev), cmd);

  soft_state = (quantis_soft_state_t*)ddi_get_soft_state(quantis_soft_state_p,
                                                         getminor(dev));
  mutex_enter(&soft_state->mutex);

  switch (cmd)
  {
    case QUANTIS_IOCTL_GET_DRIVER_VERSION:
    {
      uint32_t version = QUANTIS_PCI_DRIVER_VERSION;
      if (quantis_copyout_uint(arg, flags, version) < 0)
      {
        ret = EFAULT;
      }
      else
      {
        ret = 0;
      }
      break;
    }

    case QUANTIS_IOCTL_GET_CARD_COUNT:
    {
      mutex_enter(&quantis_mutex);
      if (quantis_copyout_uint(arg, flags, card_count) < 0)
      {
        ret = EFAULT;
      }
      else
      {
        ret = 0;
      }
      mutex_exit(&quantis_mutex);
      break;
    }

    case QUANTIS_IOCTL_GET_MODULES_MASK:
    {
      uint32_t mask = quantis_rng_modules_mask(soft_state);
      if (quantis_copyout_uint(arg, flags, mask) < 0)
      {
        ret = EFAULT;
      }
      else
      {
        ret = 0;
      }
      break;
    }

    case QUANTIS_IOCTL_GET_BOARD_VERSION:
    {
      uint32_t version = quantis_rng_version(soft_state);
      if (quantis_copyout_uint(arg, flags, version) < 0)
      {
        ret = EFAULT;
      }
      else
      {
        ret = 0;
      }
      break;
    }

    case QUANTIS_IOCTL_RESET_BOARD:
    {
      quantis_rng_reset(soft_state);
      ret = 0;
      break;
    }

    case QUANTIS_IOCTL_ENABLE_MODULE:
    {
      unsigned int modules;
      if (quantis_copyin_uint(arg, flags, &modules) < 0)
      {
        ret = EFAULT;
      }
      else
      {
        quantis_rng_enable_modules(soft_state, modules);
        ret = 0;
      }
      break;
    }

    case QUANTIS_IOCTL_DISABLE_MODULE:
    {
      unsigned int modules;
      if (quantis_copyin_uint(arg, flags, &modules) < 0)
      {
        ret = EFAULT;
      }
      else
      {
        quantis_rng_disable_modules(soft_state, modules);
        ret = 0;
      }
      break;
    }

    case QUANTIS_IOCTL_GET_MODULES_STATUS:
    {
      uint32_t status = quantis_rng_modules_status(soft_state);
      if (quantis_copyout_uint(arg, flags, status) < 0)
      {
        ret = EFAULT;
      }
      else
      {
        ret = 0;
      }
      break;
    }

    case QUANTIS_IOCTL_SET_DEBUG_LEVEL:
    default:
    {
      ret = EINVAL;
      break;
    }
  } /* switch */

  mutex_exit(&soft_state->mutex);
  return ret;
}
