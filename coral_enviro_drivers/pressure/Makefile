obj-m := bmp280.o
KVERSION := $(shell uname -r)

ccflags-y += -Wno-incompatible-pointer-types -Wno-error=incompatible-pointer-types

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean

