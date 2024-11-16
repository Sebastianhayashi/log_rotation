#ifndef COMPRESSION_UTIL_HPP
#define COMPRESSION_UTIL_HPP

#include <string>

class CompressionUtil
{
public:
    // 构造函数
    CompressionUtil();

    // 压缩文件方法
    // 参数 file_path 为待压缩文件路径，output_path 为输出的压缩文件路径
    // 返回值为 true 表示压缩成功，false 表示失败
    bool compressFile(const std::string &file_path, const std::string &output_path);
};

#endif // COMPRESSION_UTIL_HPP
