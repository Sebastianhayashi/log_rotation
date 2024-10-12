# 编译器和编译选项
CXX = g++
CXXFLAGS = -std=c++17 -Wall -I/opt/ros/humble/include

# 链接选项
LDFLAGS = -L/opt/ros/humble/lib -lrclcpp -lrcl_logging_spdlog -lspdlog

# 目标文件
TARGET = log_rotation_test

# 源文件和对象文件
SRCS = src/log_rotation_test.cpp
OBJS = $(SRCS:.cpp=.o)

# 编译规则
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
