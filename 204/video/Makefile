SUB_TAR := video_manager.o\
		   v4l2.o

built-in.o : $(SUB_TAR)
	$(LD) -r -o $@ $^
dep_file = .$@.d
%.o : %.c
	$(CC) $(CFLAGS) -Wp,-MD,$(dep_file) -c -o $@ $<