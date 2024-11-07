#include "compression_util.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>

CompressionUtil::CompressionUtil()
{
    // 构造函数，可以在需要时初始化配置
}

bool CompressionUtil::compressFile(const std::string &file_path, const std::string &output_path)
{
    std::string command = "tar -cJf " + output_path + " " + file_path;
    int result = std::system(command.c_str());
    if (result == 0)
    {
        std::cout << "File compressed successfully: " << output_path << std::endl;
        return true;
    }
    else
    {
        std::cerr << "Failed to compress file: " << file_path << std::endl;
        return false;
    }
}
