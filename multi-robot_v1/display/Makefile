SUB_TAR := disp_manager.o\
		   fb.o
		   
built-in.o : $(SUB_TAR)
	$(LD) -r -o $@ $^
dep_file = .$@.d
%.o : %.c
	$(CC) $(CFLAGS) -Wp,-MD,$(dep_file) -c -o $@ $<	