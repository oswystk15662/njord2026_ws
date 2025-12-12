#include "um982_driver/utils.hpp"
#include <algorithm>
#include <iostream>

namespace um982_driver
{
namespace utils
{

std::vector<std::string> split(const std::string & s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    
    // NMEAデータでは ",," のように空フィールドが続くことがあるため、
    // 単純な空白スキップではなく、明示的にデリミタで切る必要がある
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    
    // 末尾がデリミタで終わる場合のケア（必要に応じて）
    if (!s.empty() && s.back() == delimiter) {
        tokens.push_back("");
    }

    return tokens;
}

bool validate_checksum(const std::string & sentence)
{
    // $ または # で始まり、* を含む必要がある
    size_t start_idx = sentence.find_first_of("$#");
    size_t star_idx = sentence.find('*');

    if (start_idx == std::string::npos || star_idx == std::string::npos || star_idx < start_idx) {
        return false;
    }

    // チェックサム計算対象の範囲を取得 ($の次から*の前まで)
    std::string content = sentence.substr(start_idx + 1, star_idx - start_idx - 1);
    
    // 提供されたチェックサム文字列
    std::string checksum_str = sentence.substr(star_idx + 1);
    // 改行コードなどを除去
    checksum_str.erase(std::remove(checksum_str.begin(), checksum_str.end(), '\r'), checksum_str.end());
    checksum_str.erase(std::remove(checksum_str.begin(), checksum_str.end(), '\n'), checksum_str.end());

    int provided_checksum = 0;
    try {
        provided_checksum = std::stoi(checksum_str, nullptr, 16);
    } catch (...) {
        return false;
    }

    // XOR計算
    int calculated_checksum = 0;
    for (char c : content) {
        calculated_checksum ^= c;
    }

    return calculated_checksum == provided_checksum;
}

double convert_nmea_to_latlon(const std::string & value, const std::string & direction)
{
    if (value.empty()) {
        return 0.0;
    }

    try {
        double raw_val = std::stod(value);
        
        // ddmm.mmmm -> dd + mm.mmmm/60
        // 100で割ると 3541.605 -> 35.41605 となる
        int degrees = static_cast<int>(raw_val / 100);
        double minutes = raw_val - (degrees * 100);
        
        double decimal = degrees + (minutes / 60.0);

        if (direction == "S" || direction == "W") {
            decimal = -decimal;
        }

        return decimal;
    } catch (...) {
        // パースエラー時は安全のため0を返す、あるいは例外を投げる設計も可
        return 0.0;
    }
}

std::string base64_encode(const std::string & input)
{
    static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    const char* bytes_to_encode = input.c_str();
    size_t len = input.size();

    while (len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; (i <4) ; i++)
                ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i)
    {
        for(j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            ret += base64_chars[char_array_4[j]];

        while((i++ < 3))
            ret += '=';
    }

    return ret;
}

} // namespace utils
} // namespace um982_driver