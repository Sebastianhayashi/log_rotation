CXX = g++
CXXFLAGS = -std=c++17 -Wall $(shell pkg-config --cflags-only-I rclcpp)
LDFLAGS = $(shell pkg-config --libs rclcpp rcl_logging_spdlog spdlog)

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
