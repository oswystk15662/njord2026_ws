#ifndef UM982_DRIVER_UTILS_HPP_
#define UM982_DRIVER_UTILS_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace um982_driver
{
namespace utils
{

/**
 * @brief 文字列を指定したデリミタで分割する
 * NMEAセンテンスの解析に使用します。空の要素も空文字として保持します。
 */
std::vector<std::string> split(const std::string & s, char delimiter);

/**
 * @brief NMEAフォーマットのチェックサムを検証する
 * 例: "$GNGGA,025754.00,0,N,0,E,0,0,0.7,63.3224,M,-9.7848,M,00,0000*58" の *58 が正しいかチェックします。
 */
bool validate_checksum(const std::string & sentence);

/**
 * @brief NMEA形式 (ddmm.mmmm) を 十進度数 (dd.dddd) に変換する
 * * @param value NMEA形式の数値文字列 (例: "3541.6053")
 * @param direction 方角文字 (N, S, E, W)
 * @return double 変換後の度数。変換失敗時は 0.0
 */
double convert_nmea_to_latlon(const std::string & value, const std::string & direction);

/**
 * @brief Base64エンコード (NTRIP認証用)
 */
std::string base64_encode(const std::string & input);

/**
 * @brief 度数法から弧度法への変換
 */
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

} // namespace utils
} // namespace um982_driver

#endif // UM982_DRIVER_UTILS_HPP_