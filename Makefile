CROSS_COMPILE =
CXX = $(CROSS_COMPILE)g++
CXXFLAGS = -Wall -O3 -std=c++11
LD = $(CROSS_COMPILE)g++

OBJS = fm_rx.o
LIBS = -lasound

ifndef RTLSDR
	LIBS += -liio
else
	LIBS += -lrtlsdr
	CXXFLAGS += -DRTLSDR
endif

all: $(OBJS)
	$(LD) -o fm_rx $(OBJS) $(LIBS)

fm_rx.o: fm_rx.cpp
	$(CXX) $(CXXFLAGS) -c fm_rx.cpp

clean:
	rm *.o
