
#Set all your object files (the object files of all the .c files in your project, e.g. main.o my_sub_functions.o )
OBJ = main.o motors.o runCv.o lidar.o coxRemade.o kalman.o
 
#Set any dependant header files so that if they are edited they cause a complete re-compile (e.g. main.h some_subfunctions.h some_definitions_file.h ), or leave blank
DEPS =  runCv.h lidar.h coxRemade.h motors.h spi_com.h kalman.h

#Any special libraries you are using in your project (e.g. -lbcm2835 -lrt `pkg-config --libs gtk+-3.0` ), or leave blank
LIBS = `pkg-config --libs opencv` -lrobotic_gcc -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_highgui -lwiringPi -L/opt/vc/lib

#Set any compiler flags you want to use (e.g. -I/usr/include/somefolder `pkg-config --cflags gtk+-3.0` ), or leave blank
CFLAGS = -std=c++11 -pthread -lrt `pkg-config --cflags opencv` -I/usr/local/include/eigen3
#Set the compiler you are using ( gcc for C or g++ for C++ )
CC = g++

#Set the filename extensiton of your C files (e.g. .c or .cpp )
EXTENSION = .cpp

	

#define a rule that applies to all files ending in the .o suffix, which says that the .o file depends upon the .c version of the file and all the .h files included in the DEPS macro.  Compile each object file
%.o: %$(EXTENSION) $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
	
#Combine them into the output file
#Set your desired exe output file name here
output.a: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

#Cleanup
.PHONY: clean

clean:
	rm -f *.o *~ core *~ 
