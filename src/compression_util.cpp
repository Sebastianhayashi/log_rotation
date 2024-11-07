#include "compression_util.hpp"
#include <archive.h>
#include <archive_entry.h>
#include <iostream>
#include <fstream>

bool CompressionUtil::compressToTarXZ(const std::string &output_path, const std::vector<std::string> &files_to_compress)
{
    struct archive *a;
    struct archive_entry *entry;
    std::ifstream file;
    char buffer[8192];

    // 创建压缩归档
    a = archive_write_new();
    archive_write_set_format_pax_restricted(a); // 设置为 tar 格式
    archive_write_add_filter_xz(a); // 使用 xz 压缩

    if (archive_write_open_filename(a, output_path.c_str()) != ARCHIVE_OK) {
        std::cerr << "Failed to open archive: " << archive_error_string(a) << std::endl;
        return false;
    }

    for (const auto &file_path : files_to_compress) {
        // 打开文件
        file.open(file_path, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << file_path << std::endl;
            continue;
        }

        // 创建归档条目
        entry = archive_entry_new();
        archive_entry_set_pathname(entry, file_path.c_str());
        archive_entry_set_size(entry, std::filesystem::file_size(file_path)); // 设置文件大小
        archive_entry_set_filetype(entry, AE_IFREG);
        archive_entry_set_perm(entry, 0644);
        archive_write_header(a, entry);

        // 写入文件数据
        while (file.read(buffer, sizeof(buffer))) {
            archive_write_data(a, buffer, file.gcount());
        }
        // 写入剩余数据
        archive_write_data(a, buffer, file.gcount());

        file.close();
        archive_entry_free(entry);
    }

    archive_write_close(a);
    archive_write_free(a);
    return true;
}
