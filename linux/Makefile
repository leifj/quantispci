V ?= 0
MDIR := extra

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build

ifneq ($(PATCHLEVEL),)
include $(src)/Kbuild
else

all::
	$(MAKE) -C $(KDIR) M=$(CURDIR) V=$(V) modules

install:: all
	$(MAKE) INSTALL_MOD_PATH=$(DESTDIR) INSTALL_MOD_DIR=$(MDIR) \
		-C $(KDIR) M=$(CURDIR) modules_install

clean::
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f Module.symvers

endif

install::
	/sbin/depmod -a
