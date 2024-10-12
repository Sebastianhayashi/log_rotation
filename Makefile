# 编译器
CXX = g++

# 编译选项
CXXFLAGS = -std=c++17 -Wall -I./libs/include

# 链接选项
LDFLAGS = -L./libs/lib -lrclcpp -lspdlog

# 目标文件
TARGET = log_rotation_test

# 源代码文件
SRCS = src/log_rotation_test.cpp

# 生成的对象文件
OBJS = $(SRCS:.cpp=.o)

# 编译目标
all: $(TARGET)

# 链接生成可执行文件
$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# 编译源代码文件
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 清理生成的文件
clean:
	rm -f $(OBJS) $(TARGET)
