#include "gzip.hpp"

#include <zlib.h>

namespace gcs::gz {

std::optional<std::string> inflate(std::string_view in) {
    const bool gzip = in.size() >= 2 && static_cast<uint8_t>(in[0]) == 0x1f && static_cast<uint8_t>(in[1]) == 0x8b;
    const bool zlib = in.size() >= 2 && (static_cast<uint8_t>(in[0]) & 0x0f) == 8 &&
                      ((static_cast<uint8_t>(in[0]) << 8) | static_cast<uint8_t>(in[1])) % 31 == 0;
    if (!gzip && !zlib) {
        return std::string(in);
    }
    z_stream zs{};
    if (inflateInit2(&zs, 15 + 32) != Z_OK) {  // +32: detect gzip or zlib
        return std::nullopt;
    }
    zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(in.data()));
    zs.avail_in = static_cast<uInt>(in.size());
    std::string out;
    char buf[64 * 1024];
    int rc = Z_OK;
    while (rc == Z_OK) {
        zs.next_out = reinterpret_cast<Bytef*>(buf);
        zs.avail_out = sizeof(buf);
        rc = ::inflate(&zs, Z_NO_FLUSH);
        if (rc != Z_OK && rc != Z_STREAM_END) {
            inflateEnd(&zs);
            return std::nullopt;
        }
        out.append(buf, sizeof(buf) - zs.avail_out);
        if (rc == Z_OK && zs.avail_in == 0 && zs.avail_out != 0) {
            break;  // truncated input: keep what we have
        }
    }
    inflateEnd(&zs);
    return out;
}

std::string deflateGzip(std::string_view in, int level) {
    z_stream zs{};
    deflateInit2(&zs, level, Z_DEFLATED, 15 + 16, 8, Z_DEFAULT_STRATEGY);  // +16: gzip wrapper
    zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(in.data()));
    zs.avail_in = static_cast<uInt>(in.size());
    std::string out;
    char buf[64 * 1024];
    int rc = Z_OK;
    while (rc != Z_STREAM_END) {
        zs.next_out = reinterpret_cast<Bytef*>(buf);
        zs.avail_out = sizeof(buf);
        rc = ::deflate(&zs, Z_FINISH);
        out.append(buf, sizeof(buf) - zs.avail_out);
    }
    deflateEnd(&zs);
    return out;
}

}  // namespace gcs::gz
