===========
v120_driver
===========

-------------------------------------------------------------------------------------------
Linux device driver for Highland Technology V120/V124 PCI Express VME/VXI Crate Controllers
-------------------------------------------------------------------------------------------


Before installing "v120" driver
===============================

If you have "vmepci" installed in your system, you must uninstall it
first.  "vmepci" was a development name for the driver; most people do
not have this installed on their system.

The steps look something like the following (but be sure that "vmepci"
is from us and not someone else before removing it -- check using
"modinfo vmepci"):

::

    $ sudo rmmod vmepci
    $ sudo rm -rf /lib/modules/`uname -r`/kernel/drivers/vmepci
    $ sudo rm /etc/udev/rules.d/10-vmepci.rules

Then edit /etc/modules and remove any occurence of the line

::

    vmepci

Installing "v120" driver
========================

If this is the first time installing this driver on this computer,
you should copy the rules file to /etc/udev/rules.d/

::

    $ sudo cp 10-v120.rules /etc/udev/rules.d/

This gives read-write permissions to the device files at startup.
Alternatively you can change the mode later.

Notes for RedHat Systems
------------------------

RedHat chose to backport a syntax change to the create_class() function from
the 6.4.0 kernel back to 5.14.0.  This affects v120_driver.c, which attempts
to use the kernel version to derive the correct syntax, and on RedHat systems
therefore fails.

This can be fixed by setting the KCPPFLAGS environment variable prior to
compilation::

    $ export KCPPFLAGS=-DCLASS_CREATE_SINGLE_ARG

Installing with DKMS (recommended)
----------------------------------

Use the following commands to install with DKMS.  The 'version'
target in this directory's Makefile will print the driver version.

::

    $ sudo dkms add `pwd`       # or /path/to/driver/source/directory
    $ sudo dkms build v120/`make version`
    $ sudo dkms install v120/`make version`

You will need to reboot for the changes to take effect.  Check the DKMS
output for fall-back instructions in case there is a problem after
reboot.


Installing with Makefile
------------------------

Since this is an external loadable module (not a part of the kernel
source tree), if you install directly with the Makefile you will need to
reinstall with every kernel update.  For this reason, and because DKMS is
more portable, the method discussed below is less preferable than
the DKMS method discussed above.

If you get a "/lib/modules...Makefile not found" error, you might not
have kernel headers installed yet.

::
    # Debian-derived systems
    $ sudo apt-get install linux-headers-`uname -r`
    
    # RedHat-derived systems
    $ yum groupinstall kernel-devel-`uname -r`

To build this driver, enter the following command in the shell:

::

    $ make

To install the driver in a general sort of way, use:

::

    $ sudo make install

However, there is an environment variable to consider:

* ``INSTALL_MOD_DIR``

  Suffix to ``/lib/modules/\`uname -r\`/`` to install module.

  Linux, by default, sets this to "extra" for out-of-tree modules.
  Our makefile sets it to "kernel/drivers/v120".  This can be
  overwritten on the command line.

There is yet another problem.  If this is a first install, and
``INSTALL_MOD_DIR`` is for a directory that does not yet exist, then the
kernel Makefile (as of 3.2.0-60) will only create the directory without
adding the kernel object v120.ko.  This is solved by simply repeating the
command:

::

    $ sudo make install
    $ sudo make install

Also, sometimes the kernel makefile will output something like
``DEPMOD  v120.ko`` but the kernel will still not be able to find it.  If
this happens, just do it manually:

::

    $ subo depmod -a

On some systems, the module needs to be included in /etc/modules.  Do
this by adding the line

::

    v120

Now, if the V120 or V124 is powered on and its PCIe cable is properly
connected to the PC, the driver should load at power-on.

- Paul Bailey, June 2015
- Updated Rob Gaddi, October 11 2024
