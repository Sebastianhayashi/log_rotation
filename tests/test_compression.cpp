#include <gtest/gtest.h>
#include "compression_util.hpp"
#include <fstream>

TEST(CompressionUtilTest, CompressFileTest)
{
    // 创建一个临时文件以进行压缩测试
    std::string test_file = "/tmp/test_file.txt";
    std::ofstream ofs(test_file);
    ofs << "This is a test file." << std::endl;
    ofs.close();

    CompressionUtil compressor;
    std::string output_file = "/tmp/test_file.tar.xz";
    bool result = compressor.compressFile(test_file, output_file);

    EXPECT_TRUE(result);
}
