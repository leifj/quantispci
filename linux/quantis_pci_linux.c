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

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/version.h>

#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <asm/io.h>


#include "../common/quantis_pci.h"
#include "../common/quantis_pci_common.h"

/****************************** Module parameters *****************************
 *
 * module_param(foo, int, 0000)
 * The first param is the parameters name
 * The second param is it's data type
 * The final argument is the permissions bits,
 * for exposing parameters in sysfs (if non-zero) at a later stage.
 * see http://tldp.org/LDP/lkmpg/2.6/html/x323.html
 */

// static int subdir = 0;

// module_param (subdir, int, 0);
// MODULE_PARM_DESC(subdir, "Force device(s) to be created on a /dev subdirectory (0-1)");

// #if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14))
// #define pm_message_t u32
// #endif


/* Allotted device number */
dev_t quantis_dev_number;

/* Tie with the device model */
struct class* quantis_class;


/** Logging routines **/
#define QUANTIS_INFO(fmt, args...) \
  printk(KERN_INFO "%s: " fmt "\n" , QUANTIS_PCI_DRIVER_SHORTNAME , ## args)

#define QUANTIS_WARNING(fmt, args...) \
  printk(KERN_WARNING "%s: WARNING " fmt "\n" , QUANTIS_PCI_DRIVER_SHORTNAME , ## args)

#define QUANTIS_ERROR(fmt, args...) \
  printk(KERN_ERR "%s: ERROR " fmt "\n" , QUANTIS_PCI_DRIVER_SHORTNAME , ## args)


/**
 * Quantis PCI per-device structure
 */
struct quantis_pci_device
{
  unsigned char pci_dev_irq;                         /* irq of this device */
  u_int32_t* regs;                                   /* BUS address in CPU space */
  struct semaphore mutex;                            /* device's mutex */
  unsigned char buffer[QUANTIS_DEVICE_BUFFER_SIZE];  /* random data's buffer */
  u_int32_t device_number;                           /* internal number of this device */
  struct cdev cdev;                                  /* char device structure */
};

/** Holds all device's information */
quantis_pci_device* quantis_devices[QUANTIS_PCI_MAX_CARDS];

/** Number of Quantis PCI devices */
int quantis_pci_cards_count = 0;

/** Global mutex */
struct semaphore quantis_mutex;


/****************************** Shared functions *****************************
 *
 * Drivers are not allowed to directly access the hardware, but have to use 
 * the shared quantis functions (defined in quantis_pci_common.h).
 * 
 * quantis_reg_get and quantis_reg_set must be defined here since are
 * os-specific.
 */

quantis_register_value quantis_reg_get(quantis_pci_device* qdev, quantis_register reg)
{
  return qdev->regs[reg / 4];
}

void quantis_reg_set(quantis_pci_device* qdev, quantis_register reg, quantis_register_value value)
{
  qdev->regs[reg / 4] = value;
}


/********************* Declarations of internal functions ********************
 *
 */

static int quantis_open(struct inode* inode, struct file* file);

static int quantis_close(struct inode* inode, struct file* file);

static ssize_t quantis_read(struct file* file,
                            char* buffer,
                            size_t length,
                            loff_t* ppos);

static int quantis_ioctl(struct inode* inode,
                         struct file* file,
                         unsigned int cmd,
                         unsigned long arg);

static int __devinit quantis_pci_probe(struct pci_dev* pdev,
                                       const struct pci_device_id* ent);

static void __devexit quantis_pci_remove(struct pci_dev* pdev);

static int quantis_pci_proc_read(char* buf,
                                 char** start,
                                 off_t offset,
                                 int count,
                                 int* eof,
                                 void* data);

#ifdef CONFIG_PM /* Power-management related functions */
static int quantis_pci_suspend(struct pci_dev *pdev,
                               pm_message_t state);

static int quantis_pci_resume(struct pci_dev *pdev);
#endif /* CONFIG_PM */

static int __init quantis_module_init(void);

static void __exit quantis_module_exit(void);


/************************* Kernel's modules properties ************************
 *
 */

module_init(quantis_module_init);
module_exit(quantis_module_exit);

MODULE_DEVICE_TABLE(pci, quantis_pci_ids_table);

MODULE_LICENSE(QUANTIS_PCI_DRIVER_LICENSE);
MODULE_AUTHOR(QUANTIS_PCI_DRIVER_AUTHOR);
MODULE_DESCRIPTION(QUANTIS_PCI_DRIVER_NAME);



/***************************** Internal structures ****************************
 *
 */

/**
 * File operations structure.
 */
static struct file_operations quantis_fops =
{
  .owner    =   THIS_MODULE,      /* Owner */
  .open     =   quantis_open,     /* Open method */
  .release  =   quantis_close,    /* Release method */
  .read     =   quantis_read,     /* Read method */
  /*.write    =   quantis_write,*/    /* Write method */
  /*.llseek   =   quantis_llseek,*/   /* Seek method */
  .ioctl    =   quantis_ioctl,    /* Ioctl method */
};


/**
 * The set of PCI cards that this driver supports.
 */
struct pci_device_id quantis_pci_ids_table[] __devinitdata =
{
  { PCI_DEVICE(VENDOR_ID_HESSO, DEVICE_ID_QUANTIS_PCI) },
  /*{ PCI_DEVICE(VENDOR_ID_???, DEVICE_ID_????) },*/
  {0},
};


/**
 *
 */
static struct pci_driver quantis_pci_driver =
{
  .name      = QUANTIS_PCI_DRIVER_SHORTNAME,
  .id_table  = quantis_pci_ids_table,
  .probe     = quantis_pci_probe,
  .remove    = __devexit_p(quantis_pci_remove),
#ifdef CONFIG_PM
  .suspend   = quantis_pci_suspend,
  .resume    = quantis_pci_resume,
#endif
};


/**
 *
 */
int quantis_open(struct inode* inode, struct file* file)
{
  int card_number = MINOR(inode->i_rdev);

  /* Consistency checks */
  if(card_number > (quantis_pci_cards_count - 1))
  {
    return -ENODEV;
  }

  if (!try_module_get(THIS_MODULE))
  {
    return -EBUSY;
  }

  /* Fill private data */
  file->private_data = quantis_devices[card_number];

  return 0;
}


/**
 *
 */
int quantis_close(struct inode* inode, struct file* file)
{
  module_put(THIS_MODULE);

  return 0;
}


/**
 *
 */
ssize_t quantis_read(struct file* file,
                     char* buffer,
                     size_t length,
                     loff_t* ppos)
{
  int bytes_read = 0;
  quantis_pci_device* device;

  /* Verify if this is a write access. */
  if(!access_ok(VERIFY_WRITE, buffer, length))
  {
    QUANTIS_ERROR("Write access denied for buffer %p, length 0x%08x",
                  buffer,
                  (unsigned int)length);
    return -EFAULT;
  }

  device = file->private_data;
  if (down_interruptible(&(device->mutex)))
  {
    QUANTIS_ERROR("down_interruptible ERROR");
    return -ERESTARTSYS;
  }

  bytes_read = quantis_rng_read(device,
                                device->buffer,
                                length);
  if (bytes_read < 0)
  {
    bytes_read = -EIO;
  }
  else
  {
    if (__copy_to_user(buffer, device->buffer, bytes_read))
    {
      bytes_read = -EFAULT;
    }
  }

  up(&(device->mutex));

  return bytes_read;
}


/**
 *
 */
int quantis_ioctl(struct inode* inode,
                  struct file* file,
                  unsigned int cmd,
                  unsigned long arg)
{
  int status = 0;
  int card_number = MINOR(inode->i_rdev);
  quantis_pci_device* device = quantis_devices[card_number];

  if(down_interruptible(&device->mutex))
  {
    return -ERESTARTSYS;
  }

  switch (cmd)
  {
    case QUANTIS_IOCTL_GET_DRIVER_VERSION:
    {
      status = put_user((u_int32_t)QUANTIS_PCI_DRIVER_VERSION, (u_int32_t*)arg);
      break;
    }

    case QUANTIS_IOCTL_GET_CARD_COUNT:
    {
      status = put_user((u_int32_t)quantis_pci_cards_count, (u_int32_t*)arg);
      break;
    }

    case QUANTIS_IOCTL_GET_BOARD_VERSION:
    {
      u_int32_t version = quantis_rng_version(device);
      status = put_user(version, (u_int32_t *)arg);
      break;
    }

    case QUANTIS_IOCTL_RESET_BOARD:
    {
      quantis_rng_reset(device);
      break;
    }

    case QUANTIS_IOCTL_GET_MODULES_MASK:
    {
      u_int32_t mask = quantis_rng_modules_mask(device);
      status = put_user(mask, (u_int32_t*)arg);
      break;
    }

    case QUANTIS_IOCTL_ENABLE_MODULE:
    {
      u_int32_t modules;
      get_user(modules, (u_int32_t*)arg);
      quantis_rng_enable_modules(device, modules);
      break;
    }

    case QUANTIS_IOCTL_DISABLE_MODULE:
    {
      u_int32_t modules;
      get_user(modules, (u_int32_t*)arg);
      quantis_rng_disable_modules(device, modules);
      break;
    }

    case QUANTIS_IOCTL_GET_MODULES_STATUS:
    {
      u_int32_t modules_status = quantis_rng_modules_status(device);
      status = put_user(modules_status, (u_int32_t*)arg);
      break;
    }

    default:
    {
      QUANTIS_ERROR("IOCTL '%d' is not a valid command", cmd);
      status = -ENOTTY;
    }
  } // switch (cmd)

  up(&device->mutex);

  return status;
}


#ifdef CONFIG_PM
int quantis_pci_suspend(struct pci_dev* pdev, pm_message_t state)
{
  return 0;
}

int quantis_pci_resume(struct pci_dev* pdev)
{
  return 0;
}
#endif /* CONFIG_PM */


int __devinit quantis_pci_probe(struct pci_dev* pdev,
                                const struct pci_device_id* ent)
{
  int status;
  quantis_pci_device* device;

  down(&quantis_mutex);

  /* Consistency check */
  if (quantis_pci_cards_count >= QUANTIS_PCI_MAX_CARDS)
  {
    QUANTIS_ERROR("Not supporting more than %d cards", QUANTIS_PCI_MAX_CARDS);
    up(&quantis_mutex);
    return -ENXIO;
  }

  /* Allocate memory for the per-device structure */
  quantis_devices[quantis_pci_cards_count] = kmalloc(sizeof(quantis_pci_device), GFP_KERNEL);
  if (!quantis_devices[quantis_pci_cards_count])
  {
    QUANTIS_ERROR("Unable to allocate memory for the device");
    up(&quantis_mutex);
    return -1;
  }

  /* Local variable for easier access */
  device = quantis_devices[quantis_pci_cards_count];
  quantis_pci_cards_count++;

  up(&quantis_mutex);

  /* Initialize PCI device */
  status = pci_enable_device(pdev);
  if (status != 0)
  {
    QUANTIS_ERROR("Can't enable PCI device");
    return status;
  }

  /* Reserved PCI I/O and memory resources */
  status = pci_request_regions(pdev, QUANTIS_PCI_DEVICE_NAME);
  if (status != 0)
  {
    QUANTIS_ERROR("Can't reserve PCI I/O and memory resources");
    return status;
  }

  /* map bus memory into CPU space */
  down(&quantis_mutex);
  device->regs = (u_int32_t*)ioremap_nocache(pci_resource_start(pdev, 1), QUANTIS_REG_LENGTH);

  /* Sets devices info */
  device->device_number = quantis_pci_cards_count - 1;
  device->pci_dev_irq = pdev->irq;

  sema_init(&device->mutex, 1);
  up(&quantis_mutex);

  /* Reset Quantis */
  down(&device->mutex);
  quantis_rng_reset(device);
  up(&device->mutex);

  /* Connect the file operations with the cdev */
  cdev_init(&device->cdev, &quantis_fops);
  device->cdev.owner = THIS_MODULE;

  /* Connect the major/minor number to the cdev */
  status = cdev_add(&device->cdev,
                    MKDEV(MAJOR(quantis_dev_number), device->device_number),
                    1);
  if (status < 0)
  {
    QUANTIS_ERROR("Can't connect major/mino number to cdev");
    return status;
  }

  /* Send uevents to udev, so it'll create /dev nodes */
  // TODO Create udev rule to override default permission
  device_create(quantis_class,
                NULL,
                MKDEV(MAJOR(quantis_dev_number), device->device_number),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
                NULL,
#endif
                "%s%d", QUANTIS_PCI_DEVICE_NAME, device->device_number);

  /* Some debug information */
  QUANTIS_INFO("Found card #%d", device->device_number);

  QUANTIS_INFO("   core version 0x%08x", quantis_rng_version(device));

  QUANTIS_INFO("   device registered at /dev/%s%d",
               QUANTIS_PCI_DEVICE_NAME,
               device->device_number);

  return 0;
}


/**
 *
 */
void __devexit quantis_pci_remove(struct pci_dev* pdev)
{
  int i;
  quantis_pci_device* device = NULL;

  down(&quantis_mutex);

  /* Search device to remove */
  for(i = 0; i < QUANTIS_PCI_MAX_CARDS; i++)
  {
    if((quantis_devices[i]) && (quantis_devices[i]->pci_dev_irq == pdev->irq))
    {
      device = quantis_devices[i];
    }
  }

  if (device == NULL)
  {
    QUANTIS_ERROR("Unable to remove unknown device");
    return;
  }

  /* unmap bus memory from CPU space */
  iounmap(device->regs);

  /* Release reserved PCI I/O and memory resources */
  pci_release_regions(pdev);

  /* Disable PCI device after use */
  pci_disable_device(pdev);

  /* remove the cdev */
  cdev_del(&device->cdev);

  /* release I/O region */
  device_destroy(quantis_class, MKDEV(MAJOR(quantis_dev_number), device->device_number));

  /* Release per-device structure */
  kfree(device);

  quantis_pci_cards_count--;

  up(&quantis_mutex);

  QUANTIS_INFO("Driver manages %d card(s)", quantis_pci_cards_count);
}


/**
 *
 */
int quantis_pci_proc_read(char* buf,
                          char** start,
                          off_t offset,
                          int count,
                          int* eof,
                          void* data)
{
  int len = 0;
  int cards_count = 0;
  int i;
  int j;

  down(&quantis_mutex);
  cards_count = quantis_pci_cards_count;
  up(&quantis_mutex);

  /* Driver info */
  len += snprintf(buf + len, count - len,
                  QUANTIS_PCI_DRIVER_NAME"\n");

  len += snprintf(buf + len, count - len,
                  "driver name         : %s\n",
                  QUANTIS_PCI_DRIVER_SHORTNAME);

  len += snprintf(buf + len, count - len,
                  "driver version      : %d.%d\n",
                  QUANTIS_PCI_DRIVER_VERSION / 10,
                  QUANTIS_PCI_DRIVER_VERSION % 10);

  len += snprintf(buf + len, count - len,
                  "device major number : %d\n",
                  MAJOR(quantis_dev_number));

  len += snprintf(buf + len, count - len,
                  "max cards supported : %d\n",
                  QUANTIS_PCI_MAX_CARDS);

  len += snprintf(buf + len, count - len,
                  "card(s) found       : %d\n",
                  cards_count);

  /* Display info about each card */
  for(i = 0; i < cards_count; i++)
  {
    quantis_pci_device* device = quantis_devices[i];;

    len += snprintf(buf + len, count - len,
                    "\ncard #%d details:\n", i);

    if (!device)
    {
      len += snprintf(buf + len, count - len,
                      "   No information available for this card\n");
      continue;
    }

    len += snprintf(buf + len, count - len,
                    "   core version: 0x%08x\n",
                    quantis_rng_version(device));

    // TODO number of modules and which has been enabled/disabled
    for (j = 0; j < QUANTIS_PCI_MAX_MODULES; j++)
    {
      char* str_mask = NULL;
      char* str_status = NULL;
      if (quantis_rng_modules_mask(device) & (1 << j))
      {
        str_mask = "found";
        if (quantis_rng_modules_status(device) & (1 << j))
        {
          str_status = "(enabled)";
        }
        else
        {
          str_status = "(disabled)";
        }
      }
      else
      {
        str_mask = "not found";
        str_status = "";
      }
      len += snprintf(buf + len, count - len,
                      "   module %d: %s %s\n",
                      j, str_mask, str_status);
    }
  }

  *eof = 1;

  return len;
}


/**
 *
 */
static int __init quantis_module_init(void)
{
  int status = 0;
  QUANTIS_INFO("Initializing %s version %d.%d",
               QUANTIS_PCI_DRIVER_NAME,
               QUANTIS_PCI_DRIVER_VERSION / 10,
               QUANTIS_PCI_DRIVER_VERSION % 10);
  QUANTIS_INFO("  driver build %s %s", __DATE__, __TIME__);
  QUANTIS_INFO("  support enabled up to %d PCI/PCIe card(s)", QUANTIS_PCI_MAX_CARDS);

  sema_init(&quantis_mutex, 1);

  /* Request dynamic allocation of a device major number */
  // TODO: use register_chrdev_region if quantis_dev_number is set!!!!
  status = alloc_chrdev_region(&quantis_dev_number,
                               0,
                               QUANTIS_PCI_MAX_CARDS,
                               QUANTIS_PCI_DEVICE_NAME);
  if (status < 0)
  {
    QUANTIS_ERROR("Can't register quantis device");
    return status;
  }

  /* Populate sysfs entries */
  quantis_class = class_create(THIS_MODULE, QUANTIS_PCI_DEVICE_NAME);

  /* Register the PCI driver */
  status = pci_register_driver(&quantis_pci_driver);
  if (status < 0)
  {
    QUANTIS_ERROR("PCI init module failed");
    goto cleanup_1;
  }

  /* Creates a file in the /proc directory */
  if(!create_proc_read_entry(QUANTIS_PCI_DEVICE_NAME,
                             0,
                             NULL,
                             quantis_pci_proc_read,
                             NULL))
  {
    QUANTIS_ERROR("PCI init module failed");
    status = -EBUSY;
    goto cleanup_2;
  }

  /* Info */
  QUANTIS_INFO("Driver loaded. Found %d card(s)", quantis_pci_cards_count);

  return 0;

  /* Cleanup functions */
cleanup_2:
  pci_unregister_driver(&quantis_pci_driver);

cleanup_1:
  class_destroy(quantis_class);
  return status;
}


/**
 *
 */
static void __exit quantis_module_exit(void)
{
  QUANTIS_INFO("Unloading driver...");

  /* Removes file in /proc directory */
  remove_proc_entry(QUANTIS_PCI_DEVICE_NAME, 0);

  /* Unregister the pci driver */
  pci_unregister_driver(&quantis_pci_driver);

  /* Destroys the struct quantis_class structure */
  class_destroy(quantis_class);

  /* Releases the magior number */
  unregister_chrdev_region(MAJOR(quantis_dev_number), QUANTIS_PCI_MAX_CARDS);

  QUANTIS_INFO("Driver unloaded");
}
