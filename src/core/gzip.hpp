#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace gcs::gz {

// gzip (or zlib) to bytes; nullopt on corrupt input. Input that is not
// compressed at all (no gzip/zlib header) is returned as is.
std::optional<std::string> inflate(std::string_view in);
// bytes to gzip
std::string deflateGzip(std::string_view in, int level = 6);

}  // namespace gcs::gz
