
CROSS_COMPILE = arm-none-linux-gnueabi-
AS		= $(CROSS_COMPILE)as
LD		= $(CROSS_COMPILE)ld
CC		= $(CROSS_COMPILE)gcc
CPP		= $(CC) -E
AR		= $(CROSS_COMPILE)ar
NM		= $(CROSS_COMPILE)nm

STRIP		= $(CROSS_COMPILE)strip
OBJCOPY		= $(CROSS_COMPILE)objcopy
OBJDUMP		= $(CROSS_COMPILE)objdump

export AS LD CC CPP AR NM
export STRIP OBJCOPY OBJDUMP

CFLAGS :=-I /home/guozi/pkg/opencvlib/13.libjpeg/libjpeg-turbo-1.2.1/tmp/include -L /home/guozi/pkg/opencvlib/13.libjpeg/libjpeg-turbo-1.2.1/tmp/lib -L /home/latelee/lib/tslib/lib -Wall  -O2 -g
CFLAGS += -I $(shell pwd)/include

LDFLAGS :=-I /home/guozi/pkg/opencvlib/13.libjpeg/libjpeg-turbo-1.2.1/tmp/include -L /home/guozi/pkg/opencvlib/13.libjpeg/libjpeg-turbo-1.2.1/tmp/lib -L /home/latelee/lib/tslib/lib 

export CFLAGS LDFLAGS

TOPDIR := $(shell pwd)
export TOPDIR

TARGET := video2ld


obj-y := main.o
obj-y += convert/
obj-y += display/
obj-y += io/
obj-y += render/
obj-y += video/
obj-y += input/
obj-y += keyway/
obj-y += file/


#SUB_DIR  : c d
#CUR_OBJS : main.o
#SUB_OBJS : c/built-in.o d/built-in.o
SUB_DIRS := $(patsubst %/,%,$(filter %/, $(obj-y)))
CUR_OBJS := $(filter-out %/, $(obj-y))
SUB_OBJS := $(foreach f,$(SUB_DIRS),$(f)/built-in.o)

.PHONY :clean distclean $(SUB_DIRS) 

all : $(SUB_DIRS) built-in.o $(CUR_OBJS)
	$(CC) $(LDFLAGS) -o $(TARGET) built-in.o -lm -lts -ljpeg -lpthread
$(SUB_DIRS):
	make -C $@ 
main.o:
	$(CC) $(CFLAGS) -Wp,-MD,.$(@).d -c -o $(@) main.c 
built-in.o : $(CUR_OBJS) $(SUB_OBJS)
	$(LD) -r -o $@ $^

clean:
	rm -f $(shell find -name "*.o")
	rm -f $(TARGET)

distclean:
	rm -f $(shell find -name "*.o")
	rm -f $(shell find -name "*.d")
	rm -f $(TARGET)
	
