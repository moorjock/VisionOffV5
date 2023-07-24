UNAME_S := $(shell uname)

LDFLAGS = -L/usr/local/lib/
LDFLAGS += -lglut -lGL -lGLU -lGLEW -lrealsense2 -lopencv_core \
 -lopencv_highgui -lopencv_video -lopencv_videoio -lpthread

NVCC = /usr/local/cuda/bin/nvcc
NVCC_FLAGS= -g -Xcompiler "-Wall -Wno-deprecated-declarations" --ptxas-options=-v -rdc=true
INC =  -I/usr/local/cuda/samples/common/inc -I/usr/local/include/opencv4 

all: VisionOffV5

VisionOffV5: VisionOffV5.o OpenGLFuncs.o kernel.o P2P_ICP.o P2Plane_ICP.o Cell_Search.o \
	ICP_Helpers.o device_funcs.o FMat.o GMat.o CommTypes.o MyUnixSerial.o MyDataTypes.o MyTimer.o SVD_AS.o
	$(NVCC) $^ -o $@ $(LDFLAGS)

VisionOffV5.o: VisionOffV5.cpp ../Subs/constants.h ../Subs/device_funcs.cuh kernel.h OpenGLFuncs.h \
	../Subs/FMat.h ../Subs/GMat.cuh ../Subs/MyTimer.h 
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

OpenGLFuncs.o: OpenGLFuncs.cpp OpenGLFuncs.h ../Subs/device_funcs.cuh kernel.h ../Subs/constants.h \
	../Subs/FMat.h ../Subs/MyTimer.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

kernel.o: kernel.cu ../Subs/constants.h kernel.h ../Subs/device_funcs.cuh ../Subs/GMat.cuh
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

P2P_ICP.o: P2P_ICP.cu P2P_ICP.cuh ../Subs/constants.h ../Subs/device_funcs.cuh
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

P2Plane_ICP.o: P2Plane_ICP.cu P2Plane_ICP.cuh ../Subs/constants.h ../Subs/device_funcs.cuh
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

Cell_Search.o: Cell_Search.cu Cell_Search.cuh ../Subs/constants.h ../Subs/device_funcs.cuh
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

ICP_Helpers.o: ICP_Helpers.cu ICP_Helpers.cuh ../Subs/constants.h ../Subs/device_funcs.cuh
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

device_funcs.o: ../Subs/device_funcs.cu ../Subs/device_funcs.cuh ../Subs/constants.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

GMat.o: ../Subs/GMat.cu ../Subs/GMat.cuh
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

FMat.o: ../Subs/FMat.cpp ../Subs/FMat.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

CommTypes.o: ../Subs/CommTypes.cpp ../Subs/CommTypes.h ../Subs/MyUnixSerial.h ../Subs/FMat.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

MyUnixSerial.o: ../Subs/MyUnixSerial.cpp ../Subs/MyUnixSerial.h ../Subs/MyDataTypes.h ../Subs/FMat.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

MyDataTypes.o: ../Subs/MyDataTypes.cpp ../Subs/MyDataTypes.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

MyTimer.o: ../Subs/MyTimer.cpp ../Subs/MyTimer.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

SVD_AS.o: ../Subs/SVD_AS.cpp ../Subs/SVD_AS.h ../Subs/MY_NR3.h
	$(NVCC) $(NVCC_FLAGS) $(INC) -c $< -o $@

clean:
	rm -f *.o *.exe
