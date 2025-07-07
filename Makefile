CXX = g++
CXXFLAGS = -std=c++11 -Wall
TARGET = main

SRCS = main.cpp sim_ann.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

%.o: %.cpp sim_ann.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: all
	./$(TARGET)

clean:
	rm -f *.o $(TARGET)

.PHONY: all clean run