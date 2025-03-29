/*
 * kernel-versions.h -- Kernel specific flag setting for V120 crate controller.
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
 * Author: Rob Gaddi
 * Company: Highland Technology, Inc.
 * Date: 28 March 2025
 */

#include <linux/version.h>

/* Older kernel versions have different API for sysfs.
 * Just don't use it on those.
 */
#define V120_USE_SYSFS  (LINUX_VERSION_CODE > KERNEL_VERSION(3, 11, 0))

/* Kernel 5.0.0 changed access_ok() from 3 arguements to 2, getting rid
 * of support for VERIFY_READ and VERIFY_WRITE.
 */
#define TWO_ARG_ACCESS_OK (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0))

/* In kernel 6.3.0, the vm_flags member of struct vm_area_struct became
 * something requiring the use of the vm_set_flags accessor macro, whereas
 * it had previously been directly writable.
 */
#ifndef VM_FLAGS_KERNEL_FUNCTION
  #define VM_FLAGS_KERNEL_FUNCTION (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0))
#endif

/* Note: RedHat backported these changes as far as kernel 5.14.0, so this
 * check doesn't just automagically work.  Define CLASS_CREATE_SINGLE_ARG
 * from the command line if necessary.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
  #define CLASS_CREATE_SINGLE_ARG
#endif
