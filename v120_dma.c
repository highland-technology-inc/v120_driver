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

#include "v120_struct.h"
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/sched.h>

#if !HEAVY_DEBUG
# define v120_debg_(...) do { (void)0; } while (0)
#else
# define v120_debg_(...) v120_warn(__VA_ARGS__)
#endif
/*
 * TODO: Make sure this works -- kernel does not support *arbitrary*
 * 64-bit division, and gcc does not seem to fully support >32-bit
 * value shifts.
 */
#define U64_MSBS(N) ((u32)(((u64)(N) >> 16) >> 16))
#define U64_LSBS(N) ((u32)(N) & 0xFFFFFFFFU)

#define REG_CTRL(P)         ((void *)(P) + 0x00)
#define REG_STATUS(P)       ((void *)(P) + 0x04)
#define REG_NEXTL(P)        ((void *)(P) + 0x08)
#define REG_NEXTH(P)        ((void *)(P) + 0x0C)
#define REG_EADDRL(P)       ((void *)(P) + 0x10)
#define REG_EADDRH(P)       ((void *)(P) + 0x14)
#define REG_LASTVL(P)       ((void *)(P) + 0x18)
#define REG_LASTVH(P)       ((void *)(P) + 0x1C)
#define REG_VMEACC(P)       ((void *)(P) + 0x20)

/* defines for above register map */
#define CTRL_IACK           (1U << 1)
#define CTRL_RUN            (1U << 0)

#define STATUS_ERR (V120_DMA_STATUS_VAERR    \
         | V120_DMA_STATUS_BAERR  | V120_DMA_STATUS_LENERR \
         | V120_DMA_STATUS_CHKERR | V120_DMA_STATUS_VMEERR)

/* 1-MB: sure, why not */
#define V120_DMA_BUFSIZE       (1024 * 1024)

/* Set unused, to verify checksum algo. XXX remove this when verified */
#define UNUSED_MAGIC            (0xdeadbeef)

/**
 * struct v120_dma_hwdesc_t - hardware descriptor for a single DMA
 *                            transfer
 * @ctl:        Control flags.  Same as @flags field in struct
 *              v120_dma_desc_t.
 * @len:        Number of bytes to transfer
 * @vme_addr:   Address in the V120 of VME
 * @bus_addr:   Address in PC memory
 * @next_address: Address of the next one of these descriptors, or 0 if
 *              this is the last transaction for the DMA engine
 * @unused:     Currently unused
 * @checksum:   Inverse of the word-wise sum of all other words
 *
 * this resides in PC memory, passed as a bus address to the V120.
 * The .bus_addr and .next_addr fields are also bus addresses, cast into
 * little-endian u32 doublets.
 */
struct v120_dma_hwdesc_t {
        u32 ctl;
        u32 len;
        u32 vme_addr[2];
        u32 bus_addr[2];
        u32 next_addr[2];
        u32 unused;
        u32 checksum;
}; /* TODO: linux-ized "packed" */

/**
 * struct v120_dma_kdesc_t - Kernel-space DMA chain descriptor
 * @hwdesc: Nested hardware descriptor that will be read by the V120
 * @list: Linked list to the other descriptors
 * @uptr: User-space pointer accessed by the driver for double-buffering.
 * @dma_p_this: bus address of this descriptor (not @hwdesc!).
 * @p_kmem: The local-memory buffer where the DMA engine will really
 *     write to/read from.  This will be copied to/from @uptr.
 */
struct v120_dma_kdesc_t {
        struct v120_dma_hwdesc_t hwdesc;
        struct list_head list;
        void __iomem *uptr;
        dma_addr_t dma_p_this;
        void *p_kmem;
};

static inline __iomem void *dma_regbase(struct v120_dev_t *v120)
{
        /* TODO: Correct BAR number? */
        return v120->p_bar[2].mapbase;
}

/*
 * There's apparently no portable way to do this, so we got our own
 * version.  Fortunately, none of our 64-bit accesses need to be atomic.
 * le64_to_cpu() is already implied in the result, since we know the
 * hardware endianness, and we're putting together the u64 ourselves.
 */
static u64 v120_ioread64(void *p)
{
        u64 a = (u64)ioread32(p);
        u64 b = (u64)ioread32(p + 4);
        return b * 0x100000000uLL + a;
}

#if DMA_PSEUDO

# if HEAVY_DEBUG
static void print_desc(struct v120_dev_t *v120,
                       struct v120_dma_hwdesc_t *h)
{
        v120_debg_(v120, "-- @%p DESCRIPTOR --\n", h);
        v120_debg_(v120, "  CTL=%#x\n", h->ctl);
        v120_debg_(v120, "  LEN=%#X\n", h->len);
        v120_debg_(v120, "  VMEADDR={%#X, %#X}\n",
                   h->vme_addr[0], h->vme_addr[1]);
        v120_debg_(v120, "  BUSADDR={%#X, %#X}\n",
                   h->bus_addr[0], h->bus_addr[1]);
        v120_debg_(v120, "  NEXTADDR={%#X, %#X}\n",
                   h->next_addr[0], h->next_addr[1]);
        v120_debg_(v120, "  UNUSED=%#X\n", h->unused);
        v120_debg_(v120, "  CHECKSUM=%#X\n", h->checksum);
}

static void print_reg_desc(struct v120_dev_t *v120)
{
        struct v120_dma_hwdesc_t *h, tmp;
        h = (struct v120_dma_hwdesc_t *)(dma_regbase(v120) + 0x24);
        memcpy_fromio(&tmp, h, sizeof(tmp));
        print_desc(v120, &tmp);
}
# else /* !HEAVY_DEBUG */
#  define print_reg_desc(v)      do { (void)0; } while (0)
#  define print_desc(v, t)       do { (void)0; } while (0)
# endif /* !HEAVY_DEBUG */

static void set_dma_cksum(struct v120_dma_hwdesc_t *h)
{
        u32 *p;
        u32 sum;
        for (sum = 0, p = (u32 *)h; p < &h->checksum; ++p)
                sum += *p;

        h->checksum = ~sum;
}

/* copy a u64 address into a u32[2] array */
static void u64_addr_copy(u32 *to, u64 from)
{
        to[0] = U64_LSBS(from);
        to[1] = U64_MSBS(from);
}

static void v120_engine_start(struct v120_dev_t *v120,
                              struct list_head *hdr_list)
{
        struct v120_dma_kdesc_t *p, *q;
        u64 addr;
        void __iomem *pdma = dma_regbase(v120);

        /* Finish .hwdesc list pointers, checksums */
        p = list_first_entry(hdr_list, typeof(*p), list);
        while (&p->list != hdr_list) {
                struct v120_dma_hwdesc_t *h = &p->hwdesc;

                q = list_next_entry(p, list);

                u64_addr_copy(h->next_addr,
                        &q->list == hdr_list ? 0LL : q->dma_p_this);
                h->unused = UNUSED_MAGIC;
                wmb(); /* XXX: necessary? */
                set_dma_cksum(h);
                print_desc(v120, h);

                p = q;
        }

        p = list_first_entry(hdr_list, struct v120_dma_kdesc_t, list);
        addr = p->dma_p_this;
        iowrite32(U64_LSBS(addr), REG_NEXTL(pdma));
        iowrite32(U64_MSBS(addr), REG_NEXTH(pdma));
        v120_debg_(v120, "NEXTL reg<-%#X\n", U64_LSBS(addr));
        v120_debg_(v120, "NEXTH reg<-%#X\n", U64_MSBS(addr));

        v120->p_dma.flag = 0;

        wmb();
        iowrite32(CTRL_IACK | CTRL_RUN, REG_CTRL(pdma));
}

static void v120_engine_stop(struct v120_dev_t *v120)
{
        iowrite32(0, REG_CTRL(dma_regbase(v120)));
}

static int v120_dma_iswrite(u32 flags)
{
        return !!(flags & V120_DMA_CTL_WRITE);
}

#define KDESC_ALLOC_SIZE        (64 * 1024)
#define N_KDESC \
        (KDESC_ALLOC_SIZE / sizeof(struct v120_dma_kdesc_t))
#define V120_DMA_ALLOC_SIZE     (KDESC_ALLOC_SIZE + V120_DMA_BUFSIZE)
#define V120_DMA_SIZE_LIMIT     (1UL << 30) /* 1GB - Sure, why not */

struct v120_engine_t {
        int cropped: 1;
        struct v120_dma_desc_t udesc_save;
        unsigned int direction;
        unsigned int verify_type;
        unsigned int flags;
        void __user *uptr;
};

/**
 * v120_dma_xfr_cluster - helper to v120_dma_xfr()
 * @v120:       Handle to the V120
 * @engine:     Pointer to flags, user-space pointer, etc.
 *
 * In most cases this will run the entire DMA chain.  However, if the DMA
 * chain has too many links, or if the cumulative number of bytes to copy
 * is too much, this will split off a smaller DMA chain from the bigger
 * one, run the smaller one, and return.
 *
 * Returns: 0 if success, negative error if broke.
 *
 * Things to improve: This uses buffered I/O to user instead of direct
 * I/O.
 */
static int v120_dma_xfr_cluster(struct v120_dev_t *v120,
                                struct v120_engine_t *engine)
{
        struct list_head hdr_list;
        struct v120_dma_kdesc_t *pool, *kdesc, *q;
        unsigned long cursize;
        int res;
        int ret;
        dma_addr_t dma_handle;
        struct v120_dma_buf_t *dmab = &v120->p_dma;

        pool = (struct v120_dma_kdesc_t *)(dmab->buf + V120_DMA_BUFSIZE);

        INIT_LIST_HEAD(&hdr_list);
        dma_handle = dmab->addr + V120_DMA_BUFSIZE;
        kdesc = pool;
        cursize = 0;
        while (engine->uptr != NULL
               && kdesc < &pool[N_KDESC] && cursize < V120_DMA_BUFSIZE) {
                struct v120_dma_desc_t udesc;
                if (engine->cropped) {
                        /* Continue with cropped descriptor */
                        memcpy(&udesc, &engine->udesc_save, sizeof(udesc));
                } else {
                        u32 align;
                        unsigned int verify_type;

                        /* Fetch new descriptor */
                        if (!access_ok(VERIFY_READ,
                                       engine->uptr, sizeof(udesc))) {
                                ret = -EFAULT;
                                goto cleanup;
                        }
                        copy_from_user(&udesc, engine->uptr, sizeof(udesc));

                        /* Check .ptr (verify_type looks backwards here,
                         * because DMA "write" is a "read" from user
                         * space.)
                         */
                        verify_type = v120_dma_iswrite(udesc.flags)
                                        ? VERIFY_READ : VERIFY_WRITE;
                        if (!access_ok(verify_type,
                                       (void __user *)udesc.ptr,
                                       udesc.size)) {
                                ret = -EFAULT;
                                goto cleanup;
                        }

                        /* Check size and alignment */
                        align = !!(udesc.flags & V120_PD_D16) ? 1 : 3;
                        if (udesc.size == 0
                            || udesc.size >= V120_DMA_SIZE_LIMIT
                            || !!(udesc.size & align)
                            || !!(udesc.vme_address & (u64)align)
                            || !!(udesc.ptr & (u64)align)) {
                                ret = -EINVAL;
                                goto cleanup;
                        }
                }

                if (udesc.size > V120_DMA_BUFSIZE
                    || (udesc.size + cursize) > V120_DMA_BUFSIZE) {
                        /* Need to split this into multiple transfers */
                        long df = V120_DMA_BUFSIZE - cursize;
                        BUG_ON(df < 0 || df >= udesc.size);
                        memcpy(&engine->udesc_save, &udesc, sizeof(udesc));
                        udesc.size = df;
                        engine->udesc_save.ptr += df;
                        engine->udesc_save.vme_address += df;
                        engine->udesc_save.size -= df;
                        engine->cropped = true;
                } else {
                        engine->uptr = (void __user *)udesc.next;
                        engine->cropped = false;
                }

                list_add_tail(&kdesc->list, &hdr_list); /* XXX: o.o.o.? */
                kdesc->uptr = (void __user *)udesc.ptr;
                kdesc->dma_p_this = dma_handle
                            + offsetof(struct v120_dma_kdesc_t, hwdesc);
                kdesc->p_kmem = dmab->buf + cursize;
                kdesc->hwdesc.ctl = udesc.flags;
                kdesc->hwdesc.len = udesc.size;
                u64_addr_copy(kdesc->hwdesc.vme_addr, udesc.vme_address);
                u64_addr_copy(kdesc->hwdesc.bus_addr,
                              (u64)(dmab->addr + cursize));
                if (v120_dma_iswrite(kdesc->hwdesc.ctl)) {
                        copy_from_user(kdesc->p_kmem,
                                       kdesc->uptr, kdesc->hwdesc.len);
                }

                ++kdesc;
                dma_handle += sizeof(*kdesc);
                /* Keep cursize aligned to biggest possible data type */
                cursize += kdesc->hwdesc.len;
                cursize = ((cursize + 7) & ~7);
                BUG_ON(engine->cropped && cursize < V120_DMA_BUFSIZE);
        }

        v120_engine_start(v120, &hdr_list);

        res = wait_event_interruptible(v120->p_dwq, dmab->flag != 0);
        if (!!(dmab->flag & STATUS_ERR)) {
                v120_warn(v120, "DMA error\n");
                ret = -EIO;
        } else if (likely(!!(dmab->flag & V120_DMA_STATUS_OK))) {
                ret = 0;
        } else if (res < 0) {
                v120_warn(v120, "DMA Interrupted\n");
                /* TODO: if xfr incomplete, printk # of successful xfrs */
                v120_engine_stop(v120);
                /* TODO: Ensure engine truly stopped before returning. */
                ret = res;
        } else if (unlikely(!!(dmab->flag & V120_DMA_STATUS_IFLAG))) {
                v120_warn(v120, HW_ERR "Surprise DMA IRQ flag\n");
                ret = -EIO;
        } else {
                v120_warn(v120, "Unexpected FLG value: 0x%08X\n",
                          dmab->flag);
                ret = -EIO;
        }
        print_reg_desc(v120);

cleanup:
        list_for_each_entry_safe(kdesc, q, &hdr_list, list) {
                if (ret == 0 && !v120_dma_iswrite(kdesc->hwdesc.ctl)) {
                        v120_debg_(v120, "Copying to user @%p\n",
                                   kdesc->uptr);
                        copy_to_user(kdesc->uptr,
                                     kdesc->p_kmem, kdesc->hwdesc.len);
                }
                list_del(&kdesc->list);
        }

        return ret;
}

static int v120_dma_xfr(struct v120_dev_t *v120, void __user *parg)
{
        struct v120_dma_buf_t *dmab = &v120->p_dma;
        struct v120_engine_t engine;
        int res = -EINVAL;

        if (dmab->buf == NULL) {
                dmab->buf = dma_alloc_coherent(&v120->p_pci_dev->dev,
                                               V120_DMA_ALLOC_SIZE,
                                               &dmab->addr, GFP_KERNEL);
                if (dmab->buf == NULL)
                        return -ENOMEM;

                v120_debg_(v120, "buf=%p, dma_addr=%llX\n",
                           dmab->buf, (u64)dmab->addr);
        }

        engine.cropped = false;
        engine.uptr = parg;

        while (engine.cropped || (engine.uptr != NULL)) {
                mutex_lock(&dmab->mutex);
                res = v120_dma_xfr_cluster(v120, &engine);
                mutex_unlock(&dmab->mutex);
                if (res < 0)
                        break;
        }
        v120_debg_(v120, "Transaction complete, returning %d\n", res);
        return res;
}

#endif /* DMA_PSEUDO */

static int v120_dma_status(struct v120_dev_t *v120, void __user *parg)
{
        struct v120_dma_status_t status;
        if (!access_ok(VERIFY_WRITE, parg, sizeof(status)))
                return -EFAULT;
        v120_fill_dma_status(v120, &status);
        copy_to_user(parg, &status, sizeof(status));
        return 0;
}

/* Fill a &struct v120_dma_status_t with data from V120 DMA regs */
void v120_fill_dma_status(struct v120_dev_t *v120,
                          struct v120_dma_status_t *status)
{
        void __iomem *pdma = dma_regbase(v120);
        memset(status, 0, sizeof(*status));
        status->status = ioread32(REG_STATUS(pdma));
        status->erraddr = v120_ioread64(REG_EADDRL(pdma));
        status->lastvme = v120_ioread64(REG_LASTVL(pdma));
        status->vme_acc = ioread32(REG_VMEACC(pdma));
}

/*
 * IOCTL call for V120 DMA char driver.
 * IOCTL commands and structs are defined in v120_uapi.h for both user
 * and driver.
 */
long v120_driver_dma_ioctl(struct file *file, unsigned int cmd,
                           unsigned long arg)
{
        long res;
        struct v120_dev_t *v120 = to_v120_dev(file);
        if (!V120_HAS_DMA(v120))
                return -ENOSYS;

        switch (cmd) {
        /* There'll be more, I'm sure */
#if DMA_PSEUDO
        case V120_IOC_DMA_XFR:
                res = v120_dma_xfr(v120, (void __user *)arg);
                break;
# if V120_DIRECT_IO
        case V120_IOC_DMA_LXFR:
                res = v120_dma_dio_xfr(v120, (void __user *)arg);
                break;
# endif /* V120_DIRECT_IO */
#endif /* DMA_PSEUDO */
        case V120_IOC_DMA_STATUS:
                res = v120_dma_status(v120, (void __user *)arg);
                break;
        default:
                /* POSIX says "-ENOTTY", but no one else seems to do that */
                res = -EINVAL;
                break;
        }

        return res;
}

irqreturn_t v120_dma_irq_handler(struct v120_dev_t *v120, int irq)
{
        int ret;
        void __iomem *pdma;
        u32 status;

        if (!V120_HAS_DMA(v120))
                return IRQ_NONE;

        pdma = dma_regbase(v120);
        status = ioread32(REG_STATUS(pdma));
        if (!!(status & V120_DMA_STATUS_IFLAG)) {
                ret = IRQ_HANDLED;
                ++v120->p_n_dma_int;
                v120->p_dma.flag = status;
                /* clear the interrupt */
                iowrite32(CTRL_IACK, REG_CTRL(pdma));
                wake_up_interruptible(&v120->p_dwq);
        } else {
                ret = IRQ_NONE;
        }

        return ret;
}

/*
 * Close call for v120 DMA char driver.
 * Also clears any DMA chains used by 'file'. It should be the same
 * process.
 */
void v120_driver_dma_close(struct v120_dev_t *v120, struct file *file)
{
        if (!V120_HAS_DMA(v120))
                return;
        v120_debg_(v120, "Closing DMA\n");
        /* TODO: this */
}


/*
 * Perform those parts of initializing the char device that must be
 * in source with the DMA sub-module.
 * v120_cv_init() has already been called.
 */
void v120_driver_dma_init(struct v120_dev_t *v120, unsigned int minor)
{
        struct device *dev;

        if (!V120_HAS_DMA(v120))
                return;

        dev = &v120->p_pci_dev->dev;
#if USE_DMA64
        if (!dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64))) {
                v120_info(v120, "Supporting full 64-bit DMA mask\n");
                V120_SET_FLAG(v120, V120_FLAG_DAC);
        } else
#endif
        if (!dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32))) {
                v120_notice(v120, "Supporting 32-bit DMA\n");
                V120_CLEAR_FLAG(v120, V120_FLAG_DAC);
        } else {
                v120_warn(v120, "No suitable DMA available\n");
                V120_CLEAR_FLAG(v120, V120_FLAG_HAVE_DMA);
        }

        v120->p_dma.flag = 0;
        v120->p_dma.addr = (dma_addr_t)0;
        v120->p_dma.buf = NULL;
        mutex_init(&v120->p_dma.mutex);
        /* TODO: the rest */
}

/*
 * Perform those parts of initializing the char device that must be
 * in source with the DMA sub-module.
 * Call this BEFORE calling v120_cv_exit().
 */
void v120_driver_dma_exit(struct v120_dev_t *v120)
{
        if (!V120_HAS_DMA(v120))
                return;

        if (v120->p_dma.buf != NULL) {
                v120_debg_(v120, "Deleting big buffer %p\n", v120->p_dma.buf);
                dma_free_coherent(&v120->p_pci_dev->dev, V120_DMA_ALLOC_SIZE,
                                  v120->p_dma.buf, v120->p_dma.addr);
        }

        /* TODO: the rest */
}
