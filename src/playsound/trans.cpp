#include <cstdlib>
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

// 转换单个MP3文件为WAV格式
bool convert_mp3_to_wav(const std::string& input, const std::string& output) {
    std::string command = "ffmpeg -i \"" + input + "\" -acodec pcm_s16le -ar 44100 -ac 2 \"" + output + "\" -y";
    int result = std::system(command.c_str());
    
    if (result == 0) {
        std::cout << "转换成功: " << output << std::endl;
        return true;
    } else {
        std::cerr << "转换失败: " << input << " (错误码: " << result << ")" << std::endl;
        return false;
    }
}

int main() {
    // 获取当前目录路径
    fs::path current_path = fs::current_path();
    std::cout << "扫描目录: " << current_path << std::endl;
    
    // 存储所有MP3文件
    std::vector<fs::path> mp3_files;
    
    // 遍历当前目录
    for (const auto& entry : fs::directory_iterator(current_path)) {
        if (entry.is_regular_file()) {
            const auto& path = entry.path();
            if (path.extension() == ".mp3") {
                mp3_files.push_back(path);
            }
        }
    }
    
    // 按文件名排序
    std::sort(mp3_files.begin(), mp3_files.end());
    
    if (mp3_files.empty()) {
        std::cout << "未找到MP3文件!" << std::endl;
        return 0;
    }
    
    std::cout << "找到 " << mp3_files.size() << " 个MP3文件，开始转换..." << std::endl;
    
    // 创建输出目录
    fs::path output_dir = current_path / "converted_wav";
    if (!fs::exists(output_dir)) {
        fs::create_directory(output_dir);
    }
    
    int success_count = 0;
    int fail_count = 0;
    
    // 转换每个MP3文件
    for (const auto& mp3_path : mp3_files) {
        // 生成输出文件名
        fs::path wav_path = output_dir / mp3_path.stem();
        wav_path.replace_extension(".wav");
        
        // 转换文件
        if (convert_mp3_to_wav(mp3_path.string(), wav_path.string())) {
            success_count++;
        } else {
            fail_count++;
        }
    }
    
    // 输出统计结果
    std::cout << "\n转换完成! 成功: " << success_count 
              << ", 失败: " << fail_count
              << ", 总计: " << mp3_files.size() << std::endl;
    
    // 在文件资源管理器中打开输出目录（仅限Windows）
    #ifdef _WIN32
    if (success_count > 0) {
        std::string open_cmd = "explorer \"" + output_dir.string() + "\"";
        std::system(open_cmd.c_str());
    }
    #endif
    
    return 0;
}