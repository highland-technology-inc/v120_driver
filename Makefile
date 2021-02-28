
ifeq ($(KERNELRELEASE),)

INSTALL_MOD_DIR ?= kernel/drivers/v120/
export INSTALL_MOD_DIR
export INSTALL_MOD_PATH

.PHONY: dist distclean FORCE all clean install version

endif

CC += -Wno-unused-result

v120_version := $(shell grep '^PACKAGE_VERSION' $(PWD)/dkms.conf | sed 's/^.*=//g')
v120_version_sans = $(shell echo $(v120_version) | sed 's/\"//g')
ccflags-y := -DDRV_VERSION=\"$(v120_version_sans)\"

obj-m += v120.o
v120-objs := v120_driver.o v120_dma.o

PWD ?= $(shell pwd)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

version:
	@echo $(v120_version)

ifeq ($(KERNELRELEASE),)

DISTFILES := 10-v120.rules gbl-3.0.txt Makefile Readme.txt \
             v120_driver.c v120_dma.c $(wildcard *.h) dkms.conf
distdir := v120_driver-$(v120_version_sans)


dist: $(distdir).tar.gz

$(distdir).tar.gz: $(distdir)
	-tar chof - $(distdir) | gzip -9 -c > $@
	-rm -rf $(distdir)

$(distdir): FORCE
	@test -d $(distdir) || mkdir $(distdir); \
	list='$(DISTFILES)'; \
	dist_files=`for file in $$list; do echo $$file; done`; \
	for file in $$dist_files; do \
		test -f $$file && cp $$file $(distdir)/$$file; \
	done

distclean FORCE:
	-rm $(distdir).tar.gz >/dev/null 2>&1
	-rm -rf $(distdir) >/dev/null 2>&1

endif
