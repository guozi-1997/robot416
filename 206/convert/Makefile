SUB_TAR := convert_manager.o \
			 yuv2rgb.o color.o \
			 mjpeg2rgb.o \
			 jdatasrc-tj.o \
			 rgb2rgb.o

built-in.o : $(SUB_TAR)
	$(LD) -r -o $@ $^
dep_file = .$@.d
%.o : %.c
	$(CC) $(CFLAGS) -Wp,-MD,$(dep_file) -c -o $@ $<	
