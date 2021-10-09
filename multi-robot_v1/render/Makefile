SUB_TAR:= zoom.o\
		  merge.o
		  
built-in.o : $(SUB_TAR)
	$(LD) -r -o $@ $^
dep_file = .$@.d
%.o : %.c
	$(CC) $(CFLAGS) -Wp,-MD,$(dep_file) -c -o $@ $<