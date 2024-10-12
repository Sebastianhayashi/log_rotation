CXX = g++
CXXFLAGS = -std=c++17 -Wall \
            -I/opt/ros/humble/include/rclcpp \
            -I/opt/ros/humble/include \
            -I./libs/include

LDFLAGS = -L/opt/ros/humble/lib -lrclcpp -lrcl_logging_spdlog -lspdlog

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
