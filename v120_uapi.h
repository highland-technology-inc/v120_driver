/*
 * v120_uapi.h - Definitions shared between kernel and userland.
 *
 * This would be "include/uapi/misc/v120.h" if it was in the Linux source
 * tree.
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

/*
 * TODO: While not a part of the Linux source tree, This should be in a
 * folder shared by both application and driver.
 */
#ifndef V120_UAPI_H
#define V120_UAPI_H

#ifndef V120_NOLINUX_DEBUG
# include <asm/ioctl.h>
# include <linux/types.h>
#else
/* WARNING: This produces different IOCTL numbers! Only define
 * V120_NOLINUX_DEBUG if you are developing code and don't have a
 * GNU/Linux computer with the above headers available, to compile-
 * check it.  And DO NOT INSTALL if V120_NOLINUX_DEBUG is defined.
 */
# include <stdint.h>
typedef uint32_t __u32;
typedef uint64_t __u64;
# define _IO(type,nr)   (nr)
# define _IOR(type,nr,size)  (nr)
# define _IOW(type,nr,size)  (nr)
# define _IOWR(type,nr,size) (nr)
#endif

/* Number of persistent DMA chains permissible for a given device. */
/* TODO: Is this used for something? */
#define V120_NCHAINS 32

/*
 * Page descriptor definitions
 *
 * TODO: These may not be needed by driver if all driver does is
 * pass vio_pagedesc to V120 hardware as-is.
 */
#define V120_PD_SPEED_(n_)  ((n_) << 6)
#define V120_PD_ENDIAN_(n_) ((n_) << 9)
#define V120_PD_AM_(n_)     ((n_) << 0)
#define V120_PD_DWID_(n_)   ((n_) << 11)
#define V120_PD_RW_(n_)     ((n_) << 8)
#define V120_PD_SPEED_MASK  V120_PD_SPEED_(3ULL)
#define V120_PD_SMAX        V120_PD_SPEED_(3ULL)
#define V120_PD_SFAST       V120_PD_SPEED_(2ULL)
#define V120_PD_SMED        V120_PD_SPEED_(1ULL)
#define V120_PD_SSLOW       V120_PD_SPEED_(0ULL)
#define V120_PD_ENDIAN_MASK V120_PD_ENDIAN_(3ULL)
#define V120_PD_EAUTO       V120_PD_ENDIAN_(0ULL)
#define V120_PD_EBYTE       V120_PD_ENDIAN_(1ULL)
#define V120_PD_ESHORT      V120_PD_ENDIAN_(2ULL)
#define V120_PD_ELONG       V120_PD_ENDIAN_(3ULL)
#define V120_PD_RW_MASK     V120_PD_RW_(1ULL)
#define V120_PD_RW          V120_PD_RW_(0ULL)
#define V120_PD_RO          V120_PD_RW_(1ULL)
#define V120_PD_DWID_MASK   V120_PD_DWID_(1ULL)
#define V120_PD_D32         V120_PD_DWID_(0ULL)
#define V120_PD_D16         V120_PD_DWID_(1ULL)
#define V120_PD_AWID_MASK   V120_PD_AM_(0x3FULL)
#define V120_PD_A16         V120_PD_AM_(0x2DULL)
#define V120_PD_A24         V120_PD_AM_(0x3DULL)
#define V120_PD_A32         V120_PD_AM_(0x0DULL)

/*
 *      DMA IOCTL definitions
 */

/*
 * See linux source tree's Documentation/ioctl/botching-up-ioctls.txt for
 * an explanation of my types in the ioctl arguments, esp. using __u64
 * for pointers.
 */

/**
 * struct v120_dma_desc_t - Userland DMA chain descriptor for V120
 * @ptr: If reading from V120, pointer to a buffer to store the data; if
 *         writing to the V120, pointer to the data to write
 * @flags:  Control flags for the entire transaction chain.  For address
 *          widths, speed, data-width ("split"), and endianness, use
 *          %V120_PD_* flags. Use %V120_DMA_CTL_HOLD to prevent
 *          increasing the address on the VME bus (useful for FIFO
 *          accesses).  Use %V120_DMA_CTL_WRITE for writes to the V120;
 *          keep that bit clear for reads from the V120.
 * @size: Size of buffer at @ptr, in bytes.
 * @reserved: Set to zero
 * @next: Pointer to next descriptor on chain, or %NULL if end of chain
 * @vme_address: Address in VME to read from/write to
 */
struct v120_dma_desc_t {
        __u64 ptr;
        __u32 flags;
        __u32 size;
        __u64 next;
        __u64 vme_address;
};

#define V120_DMA_CTL_WRITE    (1U << 17)
#define V120_DMA_CTL_HOLD     (1U << 16)

/*
 * TODO: Move this stuff to a text file documentation for the library
 * calls to the ioctl.  Makes easier building of man pages.
 */

/**
 * struct v120_dma_status_t - Argument to %V120_IOC_DMA_STATUS ioctl
 * @status: Contents of the DMA block's STATUS register.
 * @vme_acc: Status of the last VME access attempted.  This is fast
 *         moving status information while the DMA transfer is occuring,
 *         but will be the information on the VME failure if
 *         %V120_DMA_STATUS_VMEERR is set in @status.
 * @erraddr: Address of the DMA descriptor in which an error, indicated
 *         by one of the error bits in @status.
 * @lastvme: Address of the last VME access attempted.  This is fast
 *         moving status information while the DMA transfer is occuring,
 *         but will be the address of the VME failure is
 *         %V120_DMA_STATUS_VMEERR is set in @status.
 *
 * Basically this is a register dump of the read-only DMA block registers
 * in the V120.  It's used for driver/V120 debugging.
 *
 * The @status field uses the following flags:
 *
 * %V120_DMA_STATUS_IFLAG - The state of the DMA controller interrupt
 *         flag.
 *
 * %V120_DMA_STATUS_VAERR - The DMA chain terminated because the VME
 *         address was not divisible by the VME transfer size.  @erraddr
 *         will contain the address of the failed descriptor.
 *
 * %V120_DMA_STATUS_BAERR - The DMA chain terminated because the bus
 *         address was not divisible by the VME transfer size.  @erraddr
 *         will contain the address of the failed descriptor.
 *
 * %V120_DMA_STATUS_LENERR - The DMA chain terminated because the DMA
 *         descriptor had a zero length, or a length not divisible by the
 *         VME transfer size.  @erraddr will contain the address of the
 *         failed descriptor.
 *
 * %V120_DMA_STATUS_CHKERR - The DMA chain terminated because the DMA
 *         descriptor had a bad checksum.  @erraddr will contain the
 *         address of the failed descriptor.
 *
 * %V120_DMA_STATUS_VMEERR - The DMA chain terminated due to a VME error.
 *         @erraddr will contain the address of the descriptor that
 *         failed.  @lastvme will contain the VME address that failed.
 *         @vme_acc will have more information about the transfer.
 *
 *         %V120_DMA_STATUS_VMEERR is the only error that will be seen
 *         if the V120 and driver are both functioning properly.  It
 *         suggests a bad VME card.
 *
 * %V120_DMA_STATUS_OK - The DMA chain completed successfully.
 *
 * In addition to the above flags, the @status field uses the following
 * operators:
 *
 * %V120_DMA_STATUS_DCOMP(@status) will return the number of
 *         descriptors that have been completed.
 *
 * %V120_DMA_STATUS_DFETCH(@status) will return the number of
 *         descriptors that have been fetched.
 *
 * The @vme_acc field uses the following flags:
 *
 * %V120_DMA_VMEACC_AF - A transaction cannot complete due to a failure
 *         to win arbitration on the VME bus
 *
 * %V120_DMA_VMEACC_BTO - A transaction cannot complete before the
 *         bus timer expires
 *
 * %V120_DMA_VMEACC_RETRY - A transaction completed with a RETRY
 *         response
 *
 * %V120_DMA_VMEACC_BERR - A transaction completed with a BERR response
 *
 * %V120_DMA_VMEACC_DTACK - A transaction completed with a DTACK
 *         response.
 *
 * In addition to the above flags, the @vmeacc field uses the following
 * operator:
 *
 * %V120_DMA_VMEACC_TIMER(@vme_acc) will return the number of 8-ns clock
 * ticks that the transaction took to complete.
 */
struct v120_dma_status_t {
        __u32 status;
        __u32 vme_acc;
        __u64 erraddr;
        __u64 lastvme;
        __u32 reserved[26];
};

/* Defines for above struct */
#define V120_DMA_STATUS_IFLAG        (1U << 22)
#define V120_DMA_STATUS_VAERR        (1U << 21)
#define V120_DMA_STATUS_BAERR        (1U << 20)
#define V120_DMA_STATUS_LENERR       (1U << 19)
#define V120_DMA_STATUS_CHKERR       (1U << 18)
#define V120_DMA_STATUS_VMEERR       (1U << 17)
#define V120_DMA_STATUS_OK           (1U << 16)
#define V120_DMA_STATUS_DFETCH(v)    ((v) & 0xFFU)
#define V120_DMA_STATUS_DCOMP(v)     (((v) >> 8) & 0xFFU)
#define V120_DMA_VMEACC_AF           (1U << 4)
#define V120_DMA_VMEACC_BTO          (1U << 3)
#define V120_DMA_VMEACC_RETRY        (1U << 2)
#define V120_DMA_VMEACC_BERR         (1U << 1)
#define V120_DMA_VMEACC_DTACK        (1U << 0)
#define V120_DMA_VMEACC_TIMER(v)     (((v) >> 16) & 0xFFFFU)

#define V120_IOCTL_BASE       'v'
#define V120_IO(nr)           _IO(V120_IOCTL_BASE,nr)
#define V120_IOR(nr,type)     _IOR(V120_IOCTL_BASE,nr,type)
#define V120_IOW(nr,type)     _IOW(V120_IOCTL_BASE,nr,type)
#define V120_IOWR(nr,type)    _IOWR(V120_IOCTL_BASE,nr,type)

/*
 *      The IOCTL commands
 */

#define V120_IOC_DMA_XFR      V120_IOW(0x01, struct v120_dma_desc_t)
#define V120_IOC_DMA_STATUS   V120_IOR(0x02, struct v120_dma_status_t)
#define V120_IOC_PERSIST      V120_IO(0x03)


/* **********************************************************************
 *                      V120 register maps
 ***********************************************************************/

/* Base address offsets from mmap()ing /dev/v120_c* */
#define V120_CONFIG_OFFS       (0x10000U)
#define V120_MON_OFFS          (0x14000U)
#define V120_RECORD_OFFS       (0x14800U)

/* Valid ID for V120 */
#define V120_MFR_ID            (0xFEEEU)

/**
 * struct v120_config_regs_t - CONFIG register map, located in BAR0
 * @mfr_id:   Manufacturer ID. Always %V120_MFR_ID
 * @modrev:   Module's hardware revision letter, ascii 'A' to 'Z'
 * @serial:   Module serial number
 * @dash:     Module dash number
 * @rom_rev:  Module's firmware revision letter
 * @uled:     Module's user-programmable LED register
 * @dips:     Module's DIP switch and rotary switch settings
 *
 * This is not to be confused with the PCIe configuration registers for
 * the V120.  This is the register map inside of a V120's BAR0 region
 * that holds V120-specific configuration and status registers.
 *
 * Only a small amount of these fields are used by the driver (and hence
 * documented here).  The rest are only used by user programs.  Look in
 * those user headers and the V120 manual for complete documentation of
 * this struct.
 *
 * A user can get a V120's pointer to this register map by mmap()ing
 * /dev/v120_c* at offset %V120_CONFIG_OFFS.  After probe(), and except
 * for a few sysfs exceptions, these registers, the V120's PCIe monitor
 * registers (a little higher up in BAR0 -- see the manual), and the
 * V120's PCIe records registers (ditto) are under userland's
 * jurisdiction.  The register map for the DMA engine, however, is run
 * internally by the driver, and it should not be used by users.
 *
 * Kernel code must explicitly append the __iomem qualifier for access to
 * this struct's fields; they are not written in the definition.  User
 * code must explicitly append the volatile qualifier for access to this
 * struct's fields; they are not written in the definition.
 */
struct v120_config_regs_t {
        __u32 mfr_id;         /* Offset 0x0000 */
        __u32 modtype;
        __u32 modrev;
        __u32 serial;
        __u32 dash;
        __u32 RESERVED1[3];
        __u32 rom_id;
        __u32 rom_rev;
        __u32 build;
        __u32 RESERVED2[5];
        __u32 status;         /* Offset 0x0040 */
        __u32 mcount;
        __u32 uptime;
        __u32 uled;
        __u32 dips;
        __u32 RESERVED3;

        /* Macros for reflashing et al. */
        __u32 macro;
        __u32 mp[4];
        __u32 RESERVED4[5];

        /* VME/VXI diagnostics */
        __u32 vme_acc;        /* Offset 0x0080 */
        __u32 vme_wc;
        __u32 vme_rc;
        __u32 utility;
        __u32 requester;
        __u32 arbstate;
        __u32 RESERVED5_1[(0xC4 - 0x98)/4];

        __u32 clockctl; /* Offset 0x00C4 */
        __u32 clocksta;
        __u32 fpctl;
        __u32 fpmon;
        __u32 RESERVED5_2[(0xF0 - 0xD4)/4];

        /* Ethernet info */
        __u64 mac_addr;       /* Offset 0x00F0 */
        __u32 ip_addr;
        __u32 RESERVED6[(0x104 - 0xFC)/4];

        /* System controller */
        __u32 slot;           /* Offset 0x0104 */
        __u32 RESERVED7[(0x140 - 0x108)/4];

        /* Diagnostics */
        __u32 pdflags;        /* Offset 0x0140 */
        __u32 temperature;
        __u32 airflow;
        __u32 psP5Vme;
        __u32 psP12Vme;
        __u32 psN12Vme;
        __u32 psP24Vxi;
        __u32 psN24Vxi;
        __u32 psP3_3Vxi;
        __u32 psN2Vxi;
        __u32 psN5_2Vxi;
        __u32 psP1_2Int;
        __u32 psP2_5Int;
        __u32 psP3_3Int;
        __u32 RESERVED8[128 - 94];
        __u32 ram[256 - 128];       /* Offset 0x0200 */
        __u32 buf[1024 / 4];        /* Offset 0x0400 */
} __attribute__ ((packed));


#endif /* V120_UAPI_H */
