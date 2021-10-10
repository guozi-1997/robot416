SUB_TAR := myfile.o\
		   MyMatrix.o
		   
built-in.o : $(SUB_TAR)
	$(LD) -r -o $@ $^
dep_file = .$@.d
%.o : %.c
	$(CC) $(CFLAGS) -Wp,-MD,$(dep_file) -c -o $@ $<	

