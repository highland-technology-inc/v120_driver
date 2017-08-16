/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Paul Bailey
 * Company: Highland Technology, Inc.
 * Date: 29 June 2015
 */
#ifndef V120_STRUCT_H
#define V120_STRUCT_H

/*
 * TODO: This will ultimately be something like:
 *
 *     #include <uapi/linux/v120_uapi.h>
 *
 * v120_uapi.h would be located there in the kernel source, and user
 * programs could access it with:
 *
 *     #include <linux/v120_uapi.h>
 */
#include "v120_uapi.h"

#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irqreturn.h>
#include <linux/mutex.h>

#ifndef DEPRECATED
# define DEPRECATED "[Deprecated]:"
#endif

/* These defines to be phased out when ready to integrate DMA code. */
#define DMA_PSEUDO              1
#define HEAVY_DEBUG             0
#define USE_DMA64               0
#define V120_DIRECT_IO          0

/* If I could start all over again, this would NOT be named: */
#define DRV_NAME                "v120"

#define V120_NBARS              3
#define V120_NCRATES            16

/*
 * TODO: remove this. It made sense while we were adding/removing
 * chardev definitions, but it looks like we're settled on only two.
 */
enum V120_CHARDEV_ENUM {
        V120_C_IDX = 0,
        V120_V_IDX,
        V120_Q_IDX,
        /* Number of char devs for each minor number */
        V120_NCHARDEVS
};

struct v120_dev_t;
struct class;
struct device;
struct pid;
struct file;

/**
 * struct v120_bar_t - saved base address register info
 * @start:      Start physical address
 * @end:        End physical address
 * @len:        span of start and end
 * @mapbase:    ioremap() of .start field
 */
struct v120_bar_t {
        resource_size_t start;
        resource_size_t end;
        resource_size_t len;
        void __iomem    *mapbase;
};

/**
 * struct v120_chardev_t - Definitions for a V120's char device.
 * @c_class:    struct class for this char device
 * @c_device:   struct device for this char device (TODO: is this still used?)
 * @c_cdev:     kernel-level char device for this char device.
 *              Embedded as typical.
 * @c_devno:    Device number of this char device.
 * @c_bar:      Pointer to the BAR struct in parent v120_dev_t that is
 *              for this char device.  (VME char device is BAR1 while the
 *              others are BAR0.)
 * @c_owner:    Pointer to the parent struct v120_dev_t
 *
 * There are three of these for each crate - one for the control region,
 * one for the DMA engine, and one for the VME pages.
 *
 * This is acquired from the 'private_data' field of the struct file *
 * argument to the char device file_operations functions.
 */
struct v120_chardev_t {
        struct class *c_class;
        struct device *c_device;
        struct cdev c_cdev;
        dev_t c_devno;
        struct v120_bar_t *c_bar;
        struct v120_dev_t *c_owner;
};

enum { V120_PCI_CONFIG_SIZE = 0x1000u, };
/**
 * struct v120_dev_t - private data for a V120's PCI device.
 * @p_flags:        Bitmask of effective %V120_FLAG_* status
 * @p_serial:       Module serial number, read at probe and available to
 *                  user as a device attribute.
 * @p_dips:         Raw value of the module's DIPS register, read at
 *                  probe and available to user as a device attribute.
 * @p_firmware_rev: Module's firmware revision, read at probe and
 *                  available to user as a device attribute.
 * @p_hardware_rev: Module's hardware revision, read at probe and
 *                  available to user as a device attribute.
 * @p_dash:         Module's dash number, read at probe and available to
 *                  user as a device attribute.
 * @p_pci_dev:      Pointer back to the struct pci_device allocated for
 *                  this device.
 * @p_chardev:      Array of char devices for this module.  There is one
 *                  for the DMA engine, one for the control register map,
 *                  and one for the VME pages.
 * @p_bar:          Array of embedded structs v120_bar_t.  As of firmware
 *                  revision C there are two BARs being used by the V120.
 * @p_minor:        Minor number of this module.  The minor number is the
 *                  same as the crate number.  If a user has two modules
 *                  connected with the same crate number, then only one
 *                  module will probe successfully.  It is meaningless to
 *                  assign the same crate number to two different crates
 *                  connected to the same PC.
 * @p_cfgregs:      Pointer to the module's configuration registers in
 *                  BAR0 (where struct v120_config_regs_t begins for this
 *                  module -- not the page descriptors below it or the
 *                  PCI configuration registers).
 * @p_dmaregs:      Pointer to the module's DMA engine control registers.
 * @p_pdregs:       Pointer to the V120's lowest page descriptor register
 * @p_qwq:          Wait-queue for IRQ device reads and writes.
 * @p_dwq:          Wait-queue for DMA interrupts.
 * @p_irq_pending:  Atomic (!?) flag for IRQs.
 * @p_n_dma_int:    Count of DMA interrupts - can be reset via sysfs
 * @p_nirq:         Count of non-DMA interrupts
 * @p_config:       Saved PCI config space registers
 * @p_dma:          Struct handling DMA double-buffering
 *
 * There is one of these for each V120 module connected to the PC.  It is
 * allocated at probe time.
 */
struct v120_dev_t {
        unsigned int p_flags;
        unsigned long p_serial;
        unsigned long p_dips;
        unsigned long p_firmware_rev;
        unsigned long p_hardware_rev;
        unsigned long p_dash;
        struct pci_dev *p_pci_dev;
        struct v120_chardev_t p_chardev[V120_NCHARDEVS];
        struct v120_bar_t p_bar[V120_NBARS];
        unsigned int p_minor;
        struct v120_config_regs_t __iomem *p_cfgregs;
        void __iomem *p_dmaregs;
        u64 __iomem *p_pdregs;
        wait_queue_head_t p_qwq;
        wait_queue_head_t p_dwq;
        int p_irq_pending;
        u64 p_n_dma_int;
        u64 p_nirq;
        u8 p_config[V120_PCI_CONFIG_SIZE];
        /*
         * XXX: DMA buf stays allocated until remove() time, hogs 1MB+ of
         * DMA bus memory.
         */
        struct v120_dma_buf_t {
                u32 flag;
                void *buf;
                dma_addr_t addr;
                struct mutex mutex;
        } p_dma;
};

/*
 * Get V120 chardev handle from the 'struct file *' parameter to fops
 * functions.
 */
static inline struct v120_chardev_t *to_v120_chardev(struct file *filp)
{
        return (struct v120_chardev_t *)(filp->private_data);
}

static inline struct v120_dev_t *to_v120_dev(struct file *filp)
{
        return to_v120_chardev(filp)->c_owner;
}

/*
 * TODO: These three functions, struct v120_dev_t, and everything needed
 * to define it, must be visible to kernel code.  In kernel source tree
 * these parts of this header would likely be in include/media/v120.h,
 * while the rest of this header would be privately kept in
 * drivers/media/v120/v120.h.
 *
 * While a separate module, here is a good enough place for it.
 */
extern struct v120_dev_t *v120_minor_to_dev(int minor);
extern void __iomem *v120_base_ptr(int minor);
extern void __iomem *v120_ctl_ptr(int minor);

/* Hepler to macros below */
#define V120_HAS_FLAG_(hdev_, Flag_) \
        (((hdev_)->p_flags & (Flag_)) != 0)
#define V120_SET_FLAG(hdev_, Flagmask_) \
        do { (hdev_)->p_flags |= (unsigned int)(Flagmask_); } while(0)
#define V120_CLEAR_FLAG(hdev_, Flagmask_) \
        do {(hdev_)->p_flags &= ~((unsigned int)(Flagmask_)); } while(0)

#define V120_FLAG_COND(hdev_, Flagmask_, Cond_) do {    \
        if (Cond_)                                      \
                V120_SET_FLAG(hdev_, Flagmask_);        \
        else                                            \
                V120_CLEAR_FLAG(hdev_, Flagmask_);      \
} while (0)


/*
 * p_flag Macros.
 * Use setter/getter macros directly beneath, for less error-prone
 * access to device flag info.
 */
#define V120_FLAG_HAVE_MSI      (1UL << 0)
#define V120_FLAG_SEP_VME       (1UL << 1) /* VME should be in BAR 1 */
#define V120_FLAG_HAVE_BAR1     (1UL << 2) /* BAR 1 is mapped */
#define V120_FLAG_HAVE_DMA      (1UL << 3)
#define V120_FLAG_HAVE_IRQ_CDEV (1UL << 4)
#define V120_FLAG_DAC           (1UL << 5)

#define V120_HAS_BAR1(hdev_) \
        V120_HAS_FLAG_(hdev_, V120_FLAG_HAVE_BAR1)
#define V120_HAS_SEP_VME(hdev_) \
        V120_HAS_FLAG_(hdev_, V120_FLAG_SEP_VME)
#define V120_HAS_MSI(hdev_) \
        V120_HAS_FLAG_(hdev_, V120_FLAG_HAVE_MSI)
#define V120_HAS_DMA(hdev_) \
        V120_HAS_FLAG_(hdev_, V120_FLAG_HAVE_DMA)
#define V120_HAS_IRQ_CDEV(hdev_) \
        V120_HAS_FLAG_(hdev_, V120_FLAG_HAVE_IRQ_CDEV)

/*
 * Offset from the control region's (not VME region's!) BAR map base to
 * the DMA registers.
 */
/* TODO: Very subject to change */
#define CREG_DMA_OFFS           (0x0U)

/*
 * Offset from the control region's (not VME region's!) BAR map base to
 * the CONFIG registers.
 */
#define CREG_CFG_OFFS           (0x10000U)

/*
 * Offset of the page descriptors from the control region's BAR map
 * base
 */
#define CREG_PD_OFFS            (0x0U)

/* Get the minor number from the DIPS register value 'x' */
#define V120_DIPS_MINOR(x) (((unsigned int)(x) & 0xFU) >> 0)

/* v120_dma.c */
extern irqreturn_t v120_dma_irq_handler(struct v120_dev_t *v120, int irq);
extern void v120_driver_dma_close(struct v120_dev_t *hdev,
                                  struct file *file);
extern long v120_driver_dma_ioctl(struct file *file, unsigned int cmd,
                                  unsigned long arg);
extern void v120_driver_dma_init(struct v120_dev_t *v120,
                                 unsigned int minor);
extern void v120_driver_dma_exit(struct v120_dev_t *v120);
extern void v120_fill_dma_status(struct v120_dev_t *v120,
                                 struct v120_dma_status_t *status);

/* So we don't have to keep de-referencing things in code */
#define v120_printk(v, ty, format, arg...) \
        dev_##ty(&(v)->p_pci_dev->dev, "crate %u: " format, \
                 (v)->p_minor, ## arg)

#define v120_debug(v, format, arg...) \
        v120_printk(v, dbg, format, ## arg)
#define v120_dbg(...) v120_debug(__VA_ARGS__)
#define v120_info(v, format, arg...) \
        v120_printk(v, info, format, ## arg)
#define v120_warning(v, format, arg...) \
        v120_printk(v, warn, format, ## arg)
#define v120_warn(...) v120_warning(__VA_ARGS__)
#define v120_crit(v, format, arg...) \
        v120_printk(v, crit, format, ## arg)
#define v120_err(v, format, arg...) \
        v120_printk(v, err, format, ## arg)
#define v120_notice(v, format, arg...) \
        v120_printk(v, notice, format, ## arg)

#define DR_BUG  "[Driver bug]: "

#endif /* V120_STRUCT_H */
