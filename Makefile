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

.PHONY: clean

fm_rx: $(OBJS)
	$(LD) -o fm_rx $(OBJS) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm *.o

