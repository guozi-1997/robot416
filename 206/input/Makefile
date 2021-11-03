SUB_TAR:=input_manager.o\
		 stdin.o\
		 touchscreen.o

built-in.o : $(SUB_TAR)
	$(LD) -r -o $@ $^
dep_file = .$@.d
%.o : %.c
	$(CC) $(CFLAGS) -I/home/latelee/lib/tslib/include -Wp,-MD,$(dep_file) -c -o $@ $<	
