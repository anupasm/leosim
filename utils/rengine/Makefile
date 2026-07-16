CXX ?= g++
CXXFLAGS ?= -O3 -std=c++17 -Wall -Wextra -Wpedantic

TARGET := leosim-rengine
SOURCES := leosim-rengine.cc

.PHONY: all clean self-test

all: $(TARGET)

$(TARGET): $(SOURCES) rengine-format.h
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES)

self-test: $(TARGET)
	./$(TARGET) --self-test

clean:
	$(RM) $(TARGET)
