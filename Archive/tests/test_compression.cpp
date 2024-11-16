#include <gtest/gtest.h>
#include "ros2_log_rotation/compression_util.hpp"
#include <fstream>

TEST(CompressionUtilTest, CompressFile)
{
    // 创建一个临时文件
    std::string test_file = "/tmp/test_log.log";
    std::ofstream ofs(test_file);
    ofs << "This is a test log file." << std::endl;
    ofs.close();

    // 压缩文件
    CompressionUtil compressor;
    std::string output_file = "/tmp/test_log.tar.xz";
    bool result = compressor.compressFile(test_file, output_file);

    EXPECT_TRUE(result);

    // 检查输出文件是否存在
    std::ifstream ifs(output_file);
    EXPECT_TRUE(ifs.good());

    // 清理临时文件
    std::remove(test_file.c_str());
    std::remove(output_file.c_str());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
