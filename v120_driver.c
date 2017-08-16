/*
 * v120_driver.c -- Driver for V120 crate controller.
 *
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

#include "v120_struct.h"

#include <linux/mm_types.h>
#include <asm/pgtable.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/poll.h>

#undef DEVICE_ATTR_RO
# define DEVICE_ATTR_RO(name_) \
        DEVICE_ATTR(name_, S_IRUGO, name_##_show, NULL)

#undef DEVICE_ATTR_RW
# define DEVICE_ATTR_RW(name_) \
        DEVICE_ATTR(name_, (S_IWUSR | S_IRUGO), name_##_show, name_##_store)


/* *********************************************************************
 *              Module local variables and parameters
 **********************************************************************/

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Paul Bailey <pbailey@highlandtechnology.com>");
MODULE_VERSION(DRV_VERSION);
MODULE_DESCRIPTION("HTI V120/V124 VME/VXI Crate Controler Driver");

static const struct pci_device_id v120_pci_ids[] = {
        { PCI_DEVICE(0x1C32, 22120), },
        { 0, }
};
MODULE_DEVICE_TABLE(pci, v120_pci_ids);

/*
 * (Growing) table of character device names, major numbers, & other
 * things yt can go right here.
 *
 * Some of these entries are for reference, and remain constant.
 * Others are for setting, but only at "__init" time.  Either way,
 * this table is for information that may remain in kernel memory
 * for the entire time that this module is loaded.  It applies to
 * any V120/V124 that may be connected.
 *
 * The macros below it are for prettier access to the table.
 */

#define V120_ITBL_ENTRY(I_, s_, b_)     \
        [V120_##I_##_IDX] = {           \
                .name  = "v120_" s_,    \
                .major = 0,             \
                .barno = (b_)           \
        }

static struct {
        unsigned int        major;
        const char          *name;
        const unsigned int  barno;
} v120_chardev_info_table[V120_NCHARDEVS] = {
        V120_ITBL_ENTRY(C, "c", 0),
        V120_ITBL_ENTRY(V, "v", 1),
        V120_ITBL_ENTRY(Q, "q", 0),
};

/*
 * Chardev_Enum_ is one of the 'V120_NCHARDEVS' enumerated char device
 * types for the crate.
 */
#define V120_MAJOR(Chardev_Enum_) \
        (v120_chardev_info_table[Chardev_Enum_].major)
#define V120_NAME(Chardev_Enum_) \
        (v120_chardev_info_table[Chardev_Enum_].name)
#define V120_BARNO(Chardev_Enum_) \
        (v120_chardev_info_table[Chardev_Enum_].barno)
#define V120_SET_MAJOR(Chardev_Enum_, maj_) \
        do { \
                v120_chardev_info_table[Chardev_Enum_].major = (maj_); \
        } while (0)

/*
 * Get the mapbase of a BAR number
 */
#define V120_BASE(v120_, Barno_) \
        ((v120_)->p_bar[Barno_].mapbase)

/*
 * Get the mapbase of the BAR of a given chardev enumeration for a
 * struct v120_dev_t.
 */
#define V120_CDEV_BASE(v120_, Chardev_Enum_) \
        V120_BASE(v120_, V120_BARNO(Chardev_Enum_))

/* Turn a long address file offset into a page number */
#define V120_PAGEOF(addr_)       ((addr_) >> PAGE_SHIFT)

/* Turn an offset in PAGE_SIZE numbers into a long address file offset */
#define V120_ADDRESSOF(pg_offs_) ((pg_offs_) << PAGE_SHIFT)

static inline int v120_irq_ready(struct v120_dev_t *v120)
{
        /* TODO: Check version or something */
        return 1;
}


/* *********************************************************************
 *        Save/restore config for hotplug
 **********************************************************************/

static void
v120_save_config(struct v120_dev_t *v120)
{
        int offs;
        for (offs = 0; offs < V120_PCI_CONFIG_SIZE; offs++) {
                pci_read_config_byte(v120->p_pci_dev, offs,
                                      &v120->p_config[offs]);
        }
}

static void
v120_restore_config(struct v120_dev_t *v120)
{
        int offs;
        for (offs = 0; offs < V120_PCI_CONFIG_SIZE; offs++) {
                pci_write_config_byte(v120->p_pci_dev, offs,
                                       v120->p_config[offs]);
        }
}


/* *********************************************************************
 *        Chardev code shared by control region and VME region
 **********************************************************************/

static inline unsigned long v120_vsize(struct vm_area_struct *vma)
{
        return vma->vm_end - vma->vm_start;
}

/* Helper wrapper for io_remap_pfn_range() call */
static int v120_remap_pfn_range(struct vm_area_struct *vma,
                                unsigned long phys)
{
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
        vma->vm_flags |= VM_IO;

        return io_remap_pfn_range(vma, vma->vm_start, V120_PAGEOF(phys),
                                  v120_vsize(vma), vma->vm_page_prot);
}

/* mmap the BAR region for this char dev */
static int v120_cv_mmap(struct file *filp, struct vm_area_struct *vma)
{
        int result;
        struct v120_chardev_t *vc;
        unsigned long off;
        unsigned long phys;
        unsigned long vsize;
        unsigned long psize;
        struct v120_bar_t *bar;

        vc = to_v120_chardev(filp);
        bar   = vc->c_bar;

        off   = V120_ADDRESSOF(vma->vm_pgoff);
        phys  = bar->start + off;
        vsize = v120_vsize(vma);

        psize = bar->end - bar->start + 1 - off;

        if (vsize > psize) {
                v120_debug(vc->c_owner,
                           "%s: virtual size=%lu is too big.\n",
                           __FUNCTION__, vsize);
                return -EINVAL;
        }

        result = v120_remap_pfn_range(vma, phys);
        if (result) {
                v120_debug(vc->c_owner,
                           "error in %s: io_remap_pfn_range(addr=0x%lX len=%lu) = %d\n",
                           __FUNCTION__, phys, vsize, result);
                return -EAGAIN;
        }
        return 0;
}

static int v120_cv_open(struct inode *inode, struct file *file)
{
        struct v120_chardev_t *vc;
        vc = container_of(inode->i_cdev, struct v120_chardev_t, c_cdev);
        file->private_data = vc;
        /* TODO: If DMA for this, call DMA file open function */
        return 0;
}

/*
 * Prevent count and pos from exceeding bounds.  If return -1, then
 * count cannot be adjusted from going out of bounds.
 */
static ssize_t v120_validate_count(loff_t pos, size_t count,
                                   struct v120_chardev_t *vc)
{
        size_t len = (size_t)vc->c_bar->len;
        if (pos >= len)
                return -1;

        if (count > len)
                count = len;

        if (count + pos > len)
                count = len - pos;

        return count;
}

#define NOTIFY_DEPRECATED(vc) do { \
        v120_debug((vc)->c_owner, DEPRECATED "%s\n", __FUNCTION__); \
} while (0)

/*
 * This method ignores consideration for things like alignment and
 * uses the kernel's memcpy methods, which may vary depending on
 * platform and architecture.
 */
static ssize_t v120_cv_read(struct file *file, char __user *ubuf,
                            size_t count, loff_t *pos)
{
        void __iomem *src;
        struct v120_chardev_t *vc;
        int ret;
        loff_t tpos = *pos;

        vc = to_v120_chardev(file);
        NOTIFY_DEPRECATED(vc);
        if ((count = v120_validate_count(tpos, count, vc)) < 0)
                return -EIO;
        else if (count == 0)
                return 0;
        ret = count;

        src = vc->c_bar->mapbase;
        if (src == NULL)
                return -EIO;
        src += tpos;

        while ((ssize_t)count > 0) {
                u32 tbuf[32];
                size_t tlen = sizeof(tbuf);
                if (tlen > count)
                        tlen = count;
                /*
                 * FIXME: Unless the architecture does not use the
                 * generic memcpy_fromio(), this call only works when
                 * PCI_IOBASE == (void __iomem *)0.
                 */
                memcpy_fromio(tbuf, src, tlen);
                copy_to_user(ubuf, tbuf, tlen);
                src += tlen;
                count -= tlen;
                ubuf += tlen;
        }
        *pos = tpos + ret;
        return ret;
}

/* Comment above v120_cv_read applies here as well */
static ssize_t v120_cv_write(struct file *file, const char __user *ubuf,
                             size_t count, loff_t *pos)
{
        struct v120_chardev_t *vc;
        void __iomem *dst;
        loff_t tpos = *pos;
        int ret;

        vc = to_v120_chardev(file);
        NOTIFY_DEPRECATED(vc);
        if ((count = v120_validate_count(tpos, count, vc)) < 0)
                return -EIO;
        else if (count == 0)
                return 0;
        ret = count;

        dst = vc->c_bar->mapbase;
        if (dst == NULL)
                return -EIO;
        dst += tpos;

        while ((ssize_t)count > 0) {
                u32 tbuf[32];
                size_t tlen;
                tlen = sizeof(tbuf);
                if (tlen > count)
                        tlen = count;
                copy_from_user(tbuf, ubuf, tlen);
                /*
                 * FIXME: Unless the architecture does not use the
                 * generic memcpy_fromio(), this call only works when
                 * PCI_IOBASE == (void __iomem *)0.
                 */
                memcpy_toio(dst, tbuf, tlen);
                dst += tlen;
                count -= tlen;
                ubuf += tlen;
        }

        *pos = tpos + ret;
        return ret;
}

#if 0
/* XXX: Either here or in udev, which is better? */
static int v120_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
        add_uevent_var(env, "MODE=%#o", 0666);
        return 0;
}
#endif

static int v120_cv_init(struct v120_dev_t *v120, unsigned int minor,
                        unsigned int chardev_enum,
                        const struct file_operations *filops)
{
        int result = 0;
        struct v120_chardev_t *vc;
        dev_t cdevno;
        char cdevname[20];

        snprintf(cdevname, 19, "%s%u", V120_NAME(chardev_enum), minor);

        /* Get device number */
        if (V120_MAJOR(chardev_enum) == 0) {
                v120_debug(v120, "major number incorrectly set to zero\n");
                result = -ERANGE;
                goto err_major;
        }

        cdevno = MKDEV(V120_MAJOR(chardev_enum), minor);
        vc = &v120->p_chardev[chardev_enum];

        vc->c_class = class_create(THIS_MODULE, cdevname);
        if (IS_ERR(vc->c_class)) {
                v120_debug(v120, "Failed to register device class %s\n",
                           cdevname);
                result = PTR_ERR(vc->c_class);
                goto err_class;
        }

        cdev_init(&vc->c_cdev, filops);
        vc->c_cdev.owner = THIS_MODULE;
        vc->c_cdev.ops   = filops;

        /* Initialize the device */
        vc->c_owner = v120;
        vc->c_devno = cdevno;
        result = cdev_add(&vc->c_cdev, cdevno, 1);
        if (result != 0) {
                v120_debug(v120, "error in cdev_add(name=%s)=%d\n",
                           cdevname, result);
                goto err_add;
        }

        /*
         * This is correct. v120->p_bar indexes to number of BARs, and
         * chardev_enum indexes to number of chardev -- not necessarily
         * the same.
         */
        vc->c_bar = &v120->p_bar[V120_BARNO(chardev_enum)];

        vc->c_device = device_create(vc->c_class, NULL, cdevno,
                                        NULL, cdevname);
        if (IS_ERR(vc->c_device)) {
                v120_debug(v120, "Failed to create device %s_device\n",
                           cdevname);
                result = PTR_ERR(vc->c_device);
                goto err_device;
        }

        kobject_set_name(&vc->c_cdev.kobj, cdevname);
#if 0
        vc->c_class->dev_uevent = v120_dev_uevent;
#endif
        v120_info(v120, "%s=%d:%d\n", cdevname,
                  MAJOR(vc->c_devno), MINOR(vc->c_devno));
        return 0;

        /* start of chardev error cleanup */
err_device:
        class_unregister(vc->c_class);
        class_destroy(vc->c_class);
err_class:
        cdev_del(&vc->c_cdev);
err_add:
err_major:
        v120_warning(v120, "Failed to initialize C/V char device\n");
        return result;
}

static void v120_cv_exit(struct v120_chardev_t *vc)
{
        v120_info(vc->c_owner, "Removing char dev\n");
        cdev_del(&vc->c_cdev);
        device_destroy(vc->c_class, vc->c_devno);
        class_unregister(vc->c_class);
        class_destroy(vc->c_class);
        unregister_chrdev_region(vc->c_devno, 1);
}


/* *********************************************************************
 *              Control region-only chardev code
 **********************************************************************/

static int v120_c_close(struct inode *inode, struct file *file)
{
        return 0;
}

static long v120_c_ioctl(struct file *file, unsigned int cmd,
                         unsigned long arg)
{
        long res;
        struct v120_dev_t *v120 = to_v120_dev(file);

        switch (cmd) {
        case V120_IOC_PERSIST:
                v120_restore_config(v120);
                res = 0;
                break;
        default:
                res = -EINVAL;
                break;
        }

        return res;
}

static struct file_operations v120_c_fops = {
       .owner          = THIS_MODULE,
       .open           = v120_cv_open,
       .release        = v120_c_close,
       .read           = v120_cv_read,
       .write          = v120_cv_write,
       .unlocked_ioctl = v120_c_ioctl,
       .llseek         = NULL,
       .mmap           = v120_cv_mmap,
};

static int v120_c_init(struct v120_dev_t *v120, unsigned int minor)
{
        int ret;
        ret = v120_cv_init(v120, minor, V120_C_IDX, &v120_c_fops);
        if (ret != 0)
                return ret;

        return 0;
}

static void v120_c_exit(struct v120_dev_t *v120)
{
        struct v120_chardev_t *vc = &v120->p_chardev[V120_C_IDX];

        v120_cv_exit(vc);
}


/* *********************************************************************
 *              VME region-only chardev code
 **********************************************************************/

static int v120_v_close(struct inode *inode, struct file *file)
{
        struct v120_dev_t *v120 = to_v120_dev(file);
        if (V120_HAS_DMA(v120))
                v120_driver_dma_close(v120, file);
        return 0;
}

static struct file_operations v120_v_fops = {
       .owner   = THIS_MODULE,
       .open    = v120_cv_open,
       .release = v120_v_close,
       .read    = v120_cv_read,
       .write   = v120_cv_write,
       .unlocked_ioctl = v120_driver_dma_ioctl,
       .llseek  = NULL,
       .mmap    = v120_cv_mmap,
};

static int v120_v_init(struct v120_dev_t *v120, unsigned int minor)
{
        return v120_cv_init(v120, minor, V120_V_IDX, &v120_v_fops);
}

static void v120_v_exit(struct v120_dev_t *v120)
{
        v120_cv_exit(&v120->p_chardev[V120_V_IDX]);
}


/* *********************************************************************
 *              IRQ chardev code
 **********************************************************************/

static int v120_q_close(struct inode *inode, struct file *file)
{
        return 0;
}

/*
 * IRQ read - Don't actually 'read' anything, just block until interrupt.
 * If this returns at all, then either there is a pending interrupt or
 * there was an error.
 */
static ssize_t v120_q_read(struct file *file, char __user *ubuf,
                       size_t count, loff_t *pos)
{
        struct v120_dev_t *v120 = to_v120_dev(file);
        int res;

        res = wait_event_interruptible(v120->p_qwq, !!v120->p_irq_pending);
        if (v120->p_irq_pending) {
                /*
                 * FIXME: Race condition! What if another process
                 * needs this info! What if there's another interrupt
                 * between this set an cleared again! But how else to
                 * clear it?
                 */
                v120->p_irq_pending = false;
                /*
                 * We don't accually write 1 byte to @ubuf, but the
                 * "reaad" data is don't-care
                 */
                res = 0;
        } else {
                res = -EIO;
        }
        return res;
}

static unsigned int v120_q_poll(struct file *filp, poll_table *wait)
{
        struct v120_dev_t *v120 = to_v120_dev(filp);

        poll_wait(filp, &v120->p_qwq, wait);
        /* No POLLRDNORM, because the read data is not very normal */
        return v120->p_irq_pending ? POLLIN : 0;
}

static struct file_operations v120_q_fops = {
        .owner   = THIS_MODULE,
        .open    = v120_cv_open,
        .release = v120_q_close,
        .read    = v120_q_read,
        .write   = NULL,
        .llseek  = NULL,
        .mmap    = NULL,
        .poll    = v120_q_poll,
};

static int v120_q_init(struct v120_dev_t *v120, unsigned int minor)
{
        return v120_cv_init(v120, minor, V120_Q_IDX, &v120_q_fops);
}

static void v120_q_exit(struct v120_dev_t *v120)
{
        v120_cv_exit(&v120->p_chardev[V120_Q_IDX]);
}

/* *********************************************************************
 *                       Device attributes
 **********************************************************************/

/**
 * v120_isconnected - Make an educated guess whether or not device is
 *                    still connected.
 * @v120: Pointer to a V120 device.
 *
 * FIXME: (a very big FIXME)
 *
 * Only hefty-duty, industrial-strength motherboards have hotplug
 * controllers.  For the common commercial PC, when a hot-UNplug event
 * occurs, the remove() method is not called.  The PCI config registers
 * go all -1's while still maintaining a device folder in sysfs, and
 * while still showing up on lspci.  Yet if we try to read the registers
 * again we crash and burn.
 *
 * If the crate plugs back in later, some of the PCI config registers are
 * restored, but the BAR regions, command word, and interrupt line are
 * zero.  Rewriting the BAR map seems to fix things when the device plugs
 * back in, accomplished from user space as something looking like:
 *      sudo cp saved_config /sys/bus/pci/${bus_no}/config
 * but that is a dangerous hack at best.
 *
 * So, any time we find that BAR 0 is not the BAR 0 we acquired at
 * probe time, we declare it 'disconnected'.  This will only be used
 * for the sysfs hooks.
 *
 * Users should be advised not to hot-unplug their V120/V124 crates.
 */
static int v120_isconnected(struct v120_dev_t *v120)
{
        u32 bar0;

        if (pci_read_config_dword(v120->p_pci_dev, 0x10, &bar0))
                return 0;

        if (bar0 != v120->p_bar[0].start) {
                v120_crit(v120, "BAR0 mismatch! Device shut off?\n");
                return 0;
        }
        return 1;
}

/* TODO: ABI documentation needed for this? */
/* Show use the contents of the ULED register */
static ssize_t uled_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        struct v120_dev_t *v120;
        u32 uled;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (v120 == NULL)
                return -ENOSYS;

        if (!v120_isconnected(v120))
                return -ENOSYS;

        uled = ioread32(&v120->p_cfgregs->uled);
        return scnprintf(buf, PAGE_SIZE, "0x%08X\n", (unsigned int)uled);
}

/* Set the user led ULED from user-space */
static ssize_t uled_store(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
        unsigned long v;
        char mybuf[16];
        struct v120_dev_t *v120;
        int res;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (v120 == NULL)
                return -ENOSYS;

        if (!v120_isconnected(v120))
                return -ENOSYS;

        snprintf(mybuf, sizeof(mybuf), "%s", buf);
        if ((res = kstrtoul(mybuf, 0, &v)) != 0)
                return res;

        iowrite32((u32)v, &v120->p_cfgregs->uled);
        return count;
}

/* Show user the DIPS register as read at probe() */
static ssize_t dips_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
        struct v120_dev_t *v120;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (v120 == NULL)
                return -ENOSYS;

        return scnprintf(buf, PAGE_SIZE, "%05d\n",
                         (unsigned int)v120->p_dips);
}

/* Show user the module serial number */
static ssize_t serial_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        struct v120_dev_t *v120;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (v120 == NULL)
                return -ENOSYS;

        return scnprintf(buf, PAGE_SIZE, "%05d\n",
                         (unsigned int)v120->p_serial);
}

/* Show user the module's hardware revision */
static ssize_t hardware_rev_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
        struct v120_dev_t *v120;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (v120 == NULL)
                return -ENOSYS;

        return scnprintf(buf, PAGE_SIZE, "%c\n",
                         (int)v120->p_hardware_rev);
}

/*
 * Show user the module's firmware revision
 *
 * FIXME: During a firmware upgrade the module is rebooted.  If user
 * does the "cp saved_file /sys/bus/pci/devices/.../config" hack,
 * then subsequent reads of the 'firmware_rev' attribute will be
 * inaccurate, because probe() will not have been called again.  The
 * user would have to rmmod and modprobe again to update this
 * attribute after a V120 firmware upgrade.
 */
static ssize_t firmware_rev_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
        struct v120_dev_t *v120;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (v120 == NULL)
                return -ENOSYS;

        return scnprintf(buf, PAGE_SIZE, "%c\n",
                         (int)v120->p_firmware_rev);
}

static ssize_t dash_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
        struct v120_dev_t *v120;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (v120 == NULL)
                return -ENOSYS;

        return scnprintf(buf, PAGE_SIZE, "%d\n", (int)v120->p_dash);
}

/* Helper struct for converting flags to their names for sysfs disp. */
struct flag_to_name_t {
        u32 flag;
        const char *name;
};

/* Helper to show_dma - Prints flags, returns number of bytes printed */
static int print_flags(u32 flags, char *start,
                const struct flag_to_name_t *tbl, int tbl_len)
{
        int i;
        int count = 0;
        char *s = start;
        for (i = 0; i < tbl_len; ++i) {
                if (!!(flags & tbl[i].flag)) {
                        if (count > 0)
                                *s++ = '|';
                        ++count;
                        s += sprintf(s, tbl[i].name);
                }
        }
        if (!count)
                *s++ = '0';
        *s++ = '\n';
        *s = '\0';
        return s - start;
}

#define PR_FLAGS(F,S,T)         print_flags(F,S,T,ARRAY_SIZE(T))
#define V120_DMA_NAME(ty,nm)    { V120_DMA_##ty##_##nm, __stringify(nm) }

static ssize_t dma_show(struct device *dev,
                        struct device_attribute *addr, char *buf)
{
        static const struct flag_to_name_t dma_vmeacc_names[] = {
                V120_DMA_NAME(VMEACC,AF),
                V120_DMA_NAME(VMEACC,BTO),
                V120_DMA_NAME(VMEACC,RETRY),
                V120_DMA_NAME(VMEACC,BERR),
                V120_DMA_NAME(VMEACC,DTACK),
        };
        static const struct flag_to_name_t dma_stat_names[] = {
                V120_DMA_NAME(STATUS,IFLAG),
                V120_DMA_NAME(STATUS,VAERR),
                V120_DMA_NAME(STATUS,BAERR),
                V120_DMA_NAME(STATUS,LENERR),
                V120_DMA_NAME(STATUS,CHKERR),
                V120_DMA_NAME(STATUS,VMEERR),
                V120_DMA_NAME(STATUS,OK),
        };
        struct v120_dma_status_t stat;
        struct v120_dev_t *v120;
        ssize_t size = PAGE_SIZE;
        int res;
        char *s = buf;

        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        if (!V120_HAS_DMA(v120))
                return scnprintf(s, size, "not available\n");

        v120_fill_dma_status(v120, &stat);
        res = scnprintf(s, size, "STATUS flags: ");
        s += res;
        size -= res;
        res = PR_FLAGS(stat.status, s, dma_stat_names);
        s += res;
        size -= res;
        res = scnprintf(s, size, "STATUS DCOMP: %u\n",
                        V120_DMA_STATUS_DCOMP(stat.status));
        s += res;
        size -= res;
        res = scnprintf(s, size, "STATUS DFETCH: %u\n",
                        V120_DMA_STATUS_DFETCH(stat.status));
        s += res;
        size -= res;
        res = scnprintf(s, size, "VMEACC flags: ");
        s += res;
        size -= res;
        res = PR_FLAGS(stat.vme_acc, s, dma_vmeacc_names);
        s += res;
        size -= res;
        res = scnprintf(s, size, "VMEACC TIMER: %u\n",
                        V120_DMA_VMEACC_TIMER(stat.vme_acc));
        s += res;
        size -= res;
        res = scnprintf(s, size, "ERRADDR: 0x%llX\nLASTVME: 0x%llX\n",
                        stat.erraddr, stat.lastvme);
        s += res;
        size -= res;
        res = scnprintf(s, size, "N_INT: %llu\n", v120->p_n_dma_int);
        s += res;
        return s - buf;
}

static ssize_t dma_store(struct device *dev, struct device_attribute *addr,
                         const char *buf, size_t count)
{
        struct v120_dev_t *v120;
        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        v120->p_n_dma_int = 0LL;
        return count;
}

static ssize_t nirq_show(struct device *dev,
                         struct device_attribute *addr, char *buf)
{
        struct v120_dev_t *v120;
        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        return scnprintf(buf, PAGE_SIZE, "%llu\n", v120->p_nirq);
}

static ssize_t nirq_store(struct device *dev, struct device_attribute *addr,
                          const char *buf, size_t count)
{
        struct v120_dev_t *v120;
        v120 = (struct v120_dev_t *)dev_get_drvdata(dev);
        v120->p_nirq = 0LL;
        return count;
}

static DEVICE_ATTR_RW(uled);
static DEVICE_ATTR_RO(dips);
static DEVICE_ATTR_RO(serial);
static DEVICE_ATTR_RO(firmware_rev);
static DEVICE_ATTR_RO(hardware_rev);
static DEVICE_ATTR_RO(dash);
static DEVICE_ATTR_RW(dma);
static DEVICE_ATTR_RW(nirq);


/* *********************************************************************
 *                  PCI Driver portion of the driver
 **********************************************************************/

#define VALID_HTI_ID    V120_MFR_ID

/* Map 1 BAR.  Helper for v120_initialize_bars(). */
static int v120_map_bars(struct v120_dev_t *v120, unsigned int bar)
{
        struct v120_bar_t *pbar = &v120->p_bar[bar];
        struct pci_dev *pdev = v120->p_pci_dev;

        pbar->start = pci_resource_start(pdev, bar);
        pbar->end   = pci_resource_end(pdev, bar);
        pbar->len   = pci_resource_len(pdev, bar);
        if (pbar->len == 0) {
                v120_debug(v120, "BAR%u length is zero\n", bar);
                return -ENOSPC;
        }

        pbar->mapbase = ioremap(pbar->start, pbar->len);
        if (pbar->mapbase == NULL) {
                v120_warning(v120,
                             "ioremap of BAR%u failed. Consider modifying boot parameter \"vmalloc\".\n",
                             bar);
                return -ENOMEM;
        }

        v120_debug(v120, "BAR%u at 0x%llx; length = %llu.\n",
                   bar, (unsigned long long)pbar->start,
                   (unsigned long long)pbar->len);

        return 0;
}

/*
 * Initialize a V120 device's register pointers.
 * Config BAR must have been mapped already.
 *
 * Returns a first-use sanity check: zero or -EIO.
 */
static int v120_initialize_pointers(struct v120_dev_t *v120)
{
        unsigned int id;

        v120->p_cfgregs = (V120_CDEV_BASE(v120, V120_C_IDX)
                           + CREG_CFG_OFFS);
        v120->p_dmaregs = (V120_CDEV_BASE(v120, V120_C_IDX)
                           + CREG_DMA_OFFS);
        v120->p_pdregs  = (V120_CDEV_BASE(v120, V120_C_IDX)
                           + CREG_PD_OFFS);

        /* Sanity check */
        id = ioread32(&v120->p_cfgregs->mfr_id);
        if (id != VALID_HTI_ID) {
                v120_err(v120, HW_ERR "Incorrect ID number 0x%X\n", id);
                return -EIO;
        }
        return 0;
}

/*
 * Map all bars for this device.
 *
 * We're using literal numbers for each bar during this initialization
 * phase, rather than using V120_BARNO(chardev), because some of our
 * enumerated chardev types may use the same BAR number and we do not
 * want to map the same BAR twice.
 *
 * If we cannot map BAR 1, either it's old firmware and we can continue
 * without a v120_v(n) char device, or it's new firmware but we just
 * plain cannot map it -- we'll bail in the latter case.
 *
 * At the very least we must be able to map BAR 0 for the probe to
 * complete.  If we cannot map BAR 0 we'll bail.
 *
 * See comments to v120_handle_revid().  That function should have been
 * called first.
 */
static int v120_initialize_bars(struct v120_dev_t *v120)
{
        int result;

        if ((result = v120_map_bars(v120, 0)) != 0)
                return result;

        if ((result = v120_map_bars(v120, 1)) == 0) {
                V120_SET_FLAG(v120, V120_FLAG_HAVE_BAR1);
        } else if (V120_HAS_SEP_VME(v120)) {
                v120_debug(v120, "VME is BAR1 but we cannot map it\n");
                iounmap((void *)V120_BASE(v120, 0));
                return result;
        }

        if (V120_HAS_DMA(v120)) {
                if ((result = v120_map_bars(v120, 2)) != 0) {
                        v120_warn(v120, "Cannot map DMA BAR; continuing without DMA\n");
                        V120_CLEAR_FLAG(v120, V120_FLAG_HAVE_DMA);
                        /* fall through, return "success(ful enough)" */
                }
        }

        return 0;
}

/*
 * Unmap BAR regions and invalidate pointers.
 * Do not attempt to read from a device's IO memory after this.
 */
static void v120_unmap_bars(struct v120_dev_t *v120)
{
        v120->p_cfgregs = NULL;
        v120->p_dmaregs = NULL;
        v120->p_pdregs  = NULL;

        if (V120_HAS_BAR1(v120))
                iounmap((void *)V120_BASE(v120, 1));
        iounmap((void *)V120_BASE(v120, 0));
}

/*
 * Get minor number, 0 to 15.  Put it in v120, and return it.
 * This always passes.
 */
static int v120_get_minor(struct v120_dev_t *v120)
{
        /* Minor shall be the crate number, found in the DIPS register */
        v120->p_minor = V120_DIPS_MINOR(ioread32(&v120->p_cfgregs->dips));
        return v120->p_minor;
}

/*
 * Determine a few things about a crate's capabilities, based upon its
 * revision.
 *
 * Rev A firmware (and prior to rev B draft 9) (id <= 0x02):
 *     Everything is on BAR 0.  We'll continue our driver's operations,
 *     but it no longer supports VME being on BAR 0.  The user will not
 *     have access to a 'v120_v[0-15]' VME char device.
 *
 * Rev B, C, and D firmware: id == 0x03
 *     Non-VME control registers and page descriptors are BAR 0.  The
 *     VME registers are on BAR 1.  This driver supports what is, but
 *     since this rev V120 does not support DMA, the user will not have
 *     access to a 'v120_d[0-15]' DMA char device.
 *
 * Rev E or later firmware: (id >= 0x04)
 *     DMA-capable. The 'v' device should have some DMA-related ioctls,
 *     if the host arch supports it.
 */
static void v120_handle_revid(struct v120_dev_t *v120)
{
        u8 id = 0;
        bool sep_vme;
        bool dma;

        /*
         * We get this info from the 'Revision ID' register (byte 8)
         * of the PCI configuration space.
         */
        pci_read_config_byte(v120->p_pci_dev, 8, &id);

        sep_vme = (id > 2);
        dma = (id >= 4);

        V120_FLAG_COND(v120, V120_FLAG_SEP_VME, sep_vme);
        V120_FLAG_COND(v120, V120_FLAG_HAVE_DMA, dma);
        v120_debug(v120, "probe: config ID=%hhu -> VME BAR=%d, DMA=FW-%ssupported\n",
                   id, sep_vme ? 1 : 0, dma ? "" : "un");
}

static struct device_attribute *const v120_dev_attrs[] = {
        &dev_attr_uled,
        &dev_attr_dips,
        &dev_attr_serial,
        &dev_attr_hardware_rev,
        &dev_attr_firmware_rev,
        &dev_attr_dash,
        &dev_attr_dma,
        &dev_attr_nirq,
};

/*
 * TODO: We should handle failures in device_create_file() to
 * prevent mistakes with device_remove_file(): set flags in init
 * and clear them in exit.
 */

static void v120_sysfs_init(struct device *dev)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(v120_dev_attrs); i++) {
                device_create_file(dev, v120_dev_attrs[i]);
        }
}

static void v120_sysfs_exit(struct device *dev)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(v120_dev_attrs); i++) {
                device_remove_file(dev, v120_dev_attrs[i]);
        }
}

#define REG_IRQSTATUS(V120)   (((void *)(V120)->p_cfgregs) + 0x4400U)
#define REG_PCIIRQ(V120)      (((void *)(V120)->p_cfgregs) + 0x440CU)
#define REG_IRQCTL(V120)      (((void *)(V120)->p_cfgregs) + 0x4404U)

static inline void v120_disable_local_int(struct v120_dev_t *v120)
{
        iowrite32(0L, REG_IRQCTL(v120));
}

static inline void v120_ack_local_int(struct v120_dev_t *v120)
{
        iowrite32(1L, REG_PCIIRQ(v120));
}

/* For now just tell kernel that the V120 interrupt works */
static irqreturn_t v120_irq_handler(int irq, void *dev_id)
{
        struct v120_dev_t *v120 = (struct v120_dev_t *)dev_id;

        if (!v120 || v120->p_minor >= 16 || v120->p_minor < 0)
                return IRQ_NONE;

        if (v120_dma_irq_handler(v120, irq) != IRQ_HANDLED) {
                ++v120->p_nirq;
                v120->p_irq_pending = true;
                wake_up_interruptible(&v120->p_qwq);
                v120_ack_local_int(v120);
        }

        return IRQ_HANDLED;
}

/*
 * Used to print warnings if somehow a crate number is stepping on the
 * toes of another crate.
 *
 * To prevent a memory leak and a possible zombie driver, this prevents
 * accidental repeats of the same crate.
 */
static struct v120_dev_t *v120_dev_ptrs[V120_NCRATES] = { NULL };

/* PCI device 'probe' method for a V120 crate. */
static int v120_probe(struct pci_dev *pdev,
                      const struct pci_device_id *id)
{
        int result = 0;
        int minor;
        int irq_flag = IRQF_SHARED;
        struct v120_dev_t *v120 = NULL;

        v120 = kzalloc(sizeof(*v120), GFP_KERNEL);
        if (v120 == NULL)
                goto err_alloc;

        /* Pre-initialize v120's pristine fields */
        v120->p_pci_dev = pdev;

        /* Usual pci-device initialization stuff */
        dev_set_drvdata(&pdev->dev, v120);
        if ((result = pci_enable_device(pdev)) != 0) {
                v120_debug(v120, "pci_enable_device()=%d\n", result);
                goto err_enable;
        }
        pci_set_master(pdev);
        if ((result = pci_request_regions(pdev, DRV_NAME)) != 0) {
                v120_debug(v120, "pci_request_regions()=%d\n", result);
                goto err_regions;
        }

        /* V120 configuration, mapping, v120's regptr init, etc. */
        v120_handle_revid(v120);
        if ((result = v120_initialize_bars(v120)) != 0) {
                v120_debug(v120, "v120_initialize_bars()=%d\n", result);
                goto err_map;
        }
        if ((result = v120_initialize_pointers(v120)) != 0) {
                v120_debug(v120, "v120_initialize_pointers()=%d\n", result);
                goto err_sanity;
        }

        /*
         * We can now read/write V120's control region.  Start by
         * getting our read-once attributes.
         */
        v120->p_serial       = ioread32(&v120->p_cfgregs->serial);
        v120->p_dips         = ioread32(&v120->p_cfgregs->dips);
        v120->p_hardware_rev = ioread32(&v120->p_cfgregs->modrev);
        v120->p_firmware_rev = ioread32(&v120->p_cfgregs->rom_rev);
        v120->p_dash         = ioread32(&v120->p_cfgregs->dash);

        minor = v120_get_minor(v120);
        if (v120_dev_ptrs[minor] != NULL) {
                v120_notice(v120, "Duplicate crate numbers\n");
                goto err_minor;
        }
        v120_dev_ptrs[minor] = v120;

        /*
         * TODO: Spinlock-proof the DMA engine controller here, before
         * initializing the char devices.
         */

        /* Initialize char devices */
        if ((result = v120_c_init(v120, minor)) != 0)
                goto err_cdev_c;
        if (V120_HAS_SEP_VME(v120)) {
                if ((result = v120_v_init(v120, minor)) != 0)
                        goto err_cdev_v;
        }
        V120_CLEAR_FLAG(v120, V120_FLAG_HAVE_IRQ_CDEV);
        if (v120_irq_ready(v120)) {
                if ((result = v120_q_init(v120, minor)) != 0) {
                        v120_debug(v120,
                                   "Cannot create /dev/v120_q%d, continuing without\n",
                                   minor);
                } else {
                        V120_SET_FLAG(v120, V120_FLAG_HAVE_IRQ_CDEV);
                }
        }
        if (V120_HAS_DMA(v120))
                v120_driver_dma_init(v120, minor);
#if !DMA_PSEUDO
        /* TODO: Remove this when ready */
        V120_CLEAR_FLAG(v120, V120_FLAG_HAVE_DMA);
#endif

        v120_sysfs_init(&v120->p_pci_dev->dev);

        v120_disable_local_int(v120);
        v120_ack_local_int(v120);

        /* Must be done before enabling interrupts */
        init_waitqueue_head(&v120->p_qwq);
        init_waitqueue_head(&v120->p_dwq);

        /* Maybe set up MSI */
        V120_CLEAR_FLAG(v120, V120_FLAG_HAVE_MSI);
#ifdef CONFIG_PCI_MSI
        if (pci_msi_enabled()) {
                result = pci_enable_msi(v120->p_pci_dev);
                if (result == 0) {
                        /* MSI successful.  Clear IRQF_SHARED from flag */
                        V120_SET_FLAG(v120, V120_FLAG_HAVE_MSI);
                        irq_flag = 0;
                }
        }
#endif /* CONFIG_PCI_MSI */

        result = request_irq(v120->p_pci_dev->irq, v120_irq_handler,
                             irq_flag, DRV_NAME, (void *)v120);
        if (result < 0) {
                v120_warning(v120, "Cannot register interrupt\n");
                goto err_irq;
        }
        /* Clear any pending interrupts */
        v120_ack_local_int(v120);
#if 0
        /* XXX: Why this? */
        pci_write_config_byte(v120->p_pci_dev, PCI_INTERRUPT_LINE,
                              v120->p_pci_dev->irq);
#endif

        /* Need this later to kluge around hotplug problems */
        v120_save_config(v120);

        v120_info(v120, "probe func succeeded\n");
        return 0;

        /* Start of probe's error cleanup */
        if (V120_HAS_MSI(v120)) {
                pci_disable_msi(v120->p_pci_dev);
                V120_CLEAR_FLAG(v120, V120_FLAG_HAVE_MSI);
        }
        free_irq(v120->p_pci_dev->irq, (void *)v120);

err_irq:
        if (V120_HAS_DMA(v120))
                v120_driver_dma_exit(v120);
        if (V120_HAS_IRQ_CDEV(v120))
                v120_q_exit(v120);
        if (V120_HAS_SEP_VME(v120))
                v120_v_exit(v120);
err_cdev_v:
        v120_c_exit(v120);
err_cdev_c:
err_minor:
err_sanity:
        v120_unmap_bars(v120);
err_map:
        /* XXX: necessary before pci_release_regions? */
        pci_release_regions(pdev);
err_regions:
        pci_disable_device(pdev);
err_enable:
        kfree(v120);
err_alloc:
        return result;
}

/* PCI device 'remove' method for a V120 crate. */
static void v120_remove(struct pci_dev *pdev)
{
        struct v120_dev_t *v120;

        if (pdev == NULL)
                return;

        v120 = (struct v120_dev_t *)dev_get_drvdata(&pdev->dev);
        if (v120 == NULL)
                return;

        v120_info(v120, "Removing PCI driver\n");

        /* TODO: Stop any DMA engine in progress */

        /* TODO: If we had interrupts, stop them here. */
        free_irq(v120->p_pci_dev->irq, (void *)v120);
        if (V120_HAS_MSI(v120)) {
                pci_disable_msi(v120->p_pci_dev);
                V120_CLEAR_FLAG(v120, V120_FLAG_HAVE_MSI);
        }

        if (V120_HAS_DMA(v120))
                v120_driver_dma_exit(v120);

        /* Destroy char devices */
        if (V120_HAS_IRQ_CDEV(v120))
                v120_q_exit(v120);
        if (V120_HAS_SEP_VME(v120))
                v120_v_exit(v120);
        v120_c_exit(v120);

        v120_unmap_bars(v120);

        v120_dev_ptrs[v120->p_minor] = NULL;

        v120_sysfs_exit(&v120->p_pci_dev->dev);

        pci_disable_device(pdev);
        pci_release_regions(pdev);

        kfree(v120);
}

/*
 * TODO: suspend and resume.  Likelihood of getting them into the
 * mainline kernel increases when power management is included.
 */
static int v120_suspend(struct pci_dev *pdev, pm_message_t state)
{
        dev_dbg(&pdev->dev, "%s\n", __FUNCTION__);
        return 0;
}

static int v120_resume(struct pci_dev *pdev)
{
        dev_dbg(&pdev->dev, "%s\n", __FUNCTION__);
        return 0;
}

static pci_ers_result_t
v120_error_detected(struct pci_dev *dev, enum pci_channel_state error)
{
        char *serror;
        switch (error) {
        case pci_channel_io_normal:
                serror = "normal";
                break;
        case pci_channel_io_frozen:
                serror = "frozen";
                break;
        case pci_channel_io_perm_failure:
                serror = "screwed";
                break;
        default:
                serror = "(enum???)";
                break;
        }
        dev_err(&dev->dev, "PCI bus %s error detected\n", serror);
        return PCI_ERS_RESULT_NONE;
}

static void
v120_error_resume(struct pci_dev *dev)
{
        dev_err(&dev->dev, "PCI bus resume detected\n");
}

static struct pci_error_handlers v120_err_handler = {
        .error_detected = v120_error_detected,
        .resume = v120_error_resume,
};

static struct pci_driver v120_driver = {
        .name       = DRV_NAME,
        .id_table   = v120_pci_ids,
        .probe      = v120_probe,
        .remove     = v120_remove,
        .resume     = v120_resume,
        .suspend    = v120_suspend,
        .err_handler = &v120_err_handler,
};

static void __iomem *v120_ptr(int minor, int bar)
{
        struct v120_dev_t *v120;

        BUG_ON((unsigned)bar >= V120_NBARS);

        v120 = v120_minor_to_dev(minor);
        if (v120 == NULL)
                return NULL;

        return v120->p_bar[bar].mapbase;
}

/**
 * v120_minor_to_dev - Get a V120's private driver data
 * @minor: Crate number, 0 to %V120_NCRATES
 *
 * Returns a pointer to a &struct v120_dev_t
 */
struct v120_dev_t *v120_minor_to_dev(int minor)
{
        WARN_ON((unsigned)minor >= V120_NCRATES);
        if (unlikely((unsigned)minor >= V120_NCRATES))
                return NULL;

        return v120_dev_ptrs[minor];
}
EXPORT_SYMBOL(v120_minor_to_dev);

/**
 * v120_base_ptr - Get a pointer to the VME BAR address of a V120
 * @minor: Crate number, 0 to %V120_NCRATES
 *
 * Returns an iomapped pointer variable, or %NULL in this memory is not
 * mapped
 */
void __iomem *v120_base_ptr(int minor)
{
        return v120_ptr(minor, 1);
}

EXPORT_SYMBOL(v120_base_ptr);

/**
 * v210_ctl_ptr - Get a pointer to the control region of a V120
 * @minor: Crate number, 0 to %V120_NCRATES
 *
 * Returns an iomapped pointer variable, or %NULL in this memory is not
 * mapped

 */
void __iomem *v120_ctl_ptr(int minor)
{
        return v120_ptr(minor, 1);
}

EXPORT_SYMBOL(v120_ctl_ptr);

static int __init v120_init(void)
{
        int result;
        int icdev;

        result = 0;
        pr_info(DRV_NAME "-" DRV_VERSION "\n");

        /*
         * Get major number, one for each type of char device ('c', 'v',
         * 'd') and each major with 'V120_NCRATES' amount of possible
         * minor numbers.
         *
         * (quasi-)FIXME: v120_init() fails spectacularly if this is
         * not called until after pci_register_driver(), because (it
         * appears that) probe is called asynchronously before this
         * function even finishes. Smarter kernelions wanted.
         */
        for (icdev = 0; icdev < V120_NCHARDEVS; ++icdev) {
                dev_t cdevno;
                result = alloc_chrdev_region(&cdevno, 0, V120_NCRATES,
                                             V120_NAME(icdev));
                V120_SET_MAJOR(icdev, MAJOR(cdevno));
                if (result < 0)
                        goto err_major;
        }

        result = pci_register_driver(&v120_driver);
        if (result < 0) {
                pr_warning("v120: Unable to register PCI driver\n");
                goto err_pcidrv;
        }

        goto done;

        /* Start of error cleanup */
        pci_unregister_driver(&v120_driver);
err_pcidrv:
err_major:
        for (--icdev; icdev >= 0; --icdev) {
                unregister_chrdev_region(MKDEV(V120_MAJOR(icdev), 0),
                                         V120_NCRATES);
        }
done:
        return result;
}

static void __exit v120_exit(void)
{
        dev_t cdevno;
        int i;

        for (i = 0; i < V120_NCHARDEVS; ++i) {
                cdevno = MKDEV(V120_MAJOR(i), 0);
                unregister_chrdev_region(cdevno, V120_NCRATES);
                /* Now yt we do not need this anymore, clear it. */
                /* TODO: Why? Is this module still in memory? */
                V120_SET_MAJOR(i, 0);
        }

        /* unregister this driver from the PCI bus driver */
        pci_unregister_driver(&v120_driver);
        pr_info("v120: Exiting driver\n");
}

module_init(v120_init);
module_exit(v120_exit);
