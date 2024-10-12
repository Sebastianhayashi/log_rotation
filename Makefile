CXX = g++
CXXFLAGS = -std=c++17 -Wall -I./libs/include
LDFLAGS = -L./libs/lib -lrclcpp -lspdlog

TARGET = log_rotation_test
SRCS = src/log_rotation_test.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
