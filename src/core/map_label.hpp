#pragma once

#include <string_view>

// Which name a street-map label shows. The operator reads English or Hebrew
// only: the chosen one, else the other, else no label. A tile's local `name`
// stands in for a missing `name:en` / `name:he` only when it is written
// entirely in Latin / Hebrew letters, so no third script ever shows.
namespace gcs::map_label {

enum class Script { Latin, Hebrew, Other };

// What a UTF-8 name is written in, by its letters (digits, spaces, punctuation
// and combining marks don't count; no letters at all reads as Latin). Mixed
// Latin and Hebrew, or any other letter, is Other.
Script script(std::string_view utf8);

// the label to draw ("" = none)
std::string_view pick(std::string_view name, std::string_view en, std::string_view he, bool english);

}  // namespace gcs::map_label
