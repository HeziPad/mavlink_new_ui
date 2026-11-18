#include "coords.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace gcs::coords {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
// WGS84
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kK0 = 0.9996;
constexpr double kE0 = 500000.0;
constexpr double kNorthingSouth = 10000000.0;

// Krüger series (third order in n: well under a millimetre inside a zone)
struct Series {
    double n, a_rect, alpha[3], beta[3], delta[3];
    Series() {
        n = kF / (2.0 - kF);
        const double n2 = n * n, n3 = n2 * n;
        a_rect = kA / (1.0 + n) * (1.0 + n2 / 4.0 + n2 * n2 / 64.0);
        alpha[0] = n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0;
        alpha[1] = 13.0 * n2 / 48.0 - 3.0 * n3 / 5.0;
        alpha[2] = 61.0 * n3 / 240.0;
        beta[0] = n / 2.0 - 2.0 * n2 / 3.0 + 37.0 * n3 / 96.0;
        beta[1] = n2 / 48.0 + n3 / 15.0;
        beta[2] = 17.0 * n3 / 480.0;
        delta[0] = 2.0 * n - 2.0 * n2 / 3.0 - 2.0 * n3;
        delta[1] = 7.0 * n2 / 3.0 - 8.0 * n3 / 5.0;
        delta[2] = 56.0 * n3 / 15.0;
    }
};

const Series& series() {
    static const Series s;
    return s;
}

double centralMeridian(int zone) {
    return (zone * 6.0 - 183.0) * kDeg;
}

// numbers and hemisphere letters, in order
struct Token {
    bool number = false;
    double value = 0.0;
    char letter = 0;
    std::string text;
};

std::vector<Token> tokenize(const std::string& in) {
    std::vector<Token> out;
    size_t i = 0;
    while (i < in.size()) {
        const unsigned char c = static_cast<unsigned char>(in[i]);
        if (std::isdigit(c) || ((c == '-' || c == '+' || c == '.') && i + 1 < in.size() &&
                                 (std::isdigit(static_cast<unsigned char>(in[i + 1])) || in[i + 1] == '.'))) {
            char* end = nullptr;
            const double v = std::strtod(in.c_str() + i, &end);
            const size_t used = static_cast<size_t>(end - (in.c_str() + i));
            if (used == 0) {
                ++i;
                continue;
            }
            Token t;
            t.number = true;
            t.value = v;
            t.text = in.substr(i, used);
            i += used;
            // a letter glued to the number (UTM "36R")
            if (i < in.size() && std::isalpha(static_cast<unsigned char>(in[i])) &&
                (i + 1 >= in.size() || !std::isalpha(static_cast<unsigned char>(in[i + 1])))) {
                t.letter = static_cast<char>(std::toupper(static_cast<unsigned char>(in[i])));
                ++i;
            }
            out.push_back(t);
        } else if (std::isalpha(c)) {
            size_t j = i;
            while (j < in.size() && std::isalpha(static_cast<unsigned char>(in[j]))) {
                ++j;
            }
            if (j - i == 1) {
                Token t;
                t.letter = static_cast<char>(std::toupper(c));
                out.push_back(t);
            }
            i = j;
        } else {
            ++i;  // separators: spaces, commas, ° ' " and the UTF-8 bytes of ′ ″
        }
    }
    return out;
}

std::optional<LatLon> parseAngles(const std::vector<Token>& toks) {
    // flatten: numbers and hemisphere letters in order (a letter glued to a
    // number, "32.5N", follows it)
    struct Item {
        bool number;
        double value;
        char letter;
    };
    std::vector<Item> items;
    for (const Token& t : toks) {
        if (t.number) {
            items.push_back({true, t.value, 0});
        }
        if (t.letter) {
            if (std::string("NSEW").find(t.letter) == std::string::npos) {
                return std::nullopt;
            }
            items.push_back({false, 0.0, t.letter});
        }
    }
    std::vector<double> groups[2];
    char hemi[2] = {0, 0};
    std::vector<size_t> letters;
    for (size_t k = 0; k < items.size(); ++k) {
        if (!items[k].number) {
            letters.push_back(k);
        }
    }
    if (letters.empty()) {
        std::vector<double> all;
        for (const auto& it : items) {
            all.push_back(it.value);
        }
        if (all.empty() || all.size() % 2 != 0 || all.size() > 6) {
            return std::nullopt;
        }
        groups[0].assign(all.begin(), all.begin() + static_cast<long>(all.size() / 2));
        groups[1].assign(all.begin() + static_cast<long>(all.size() / 2), all.end());
    } else if (letters.size() == 2) {
        const bool suffix = letters[0] > 0;  // "32 05 N 34 46 E" vs "N 32 05 E 34 46"
        for (size_t k = 0; k < items.size(); ++k) {
            if (!items[k].number) {
                continue;
            }
            const int g = suffix ? (k < letters[0] ? 0 : (k < letters[1] ? 1 : 2))
                                 : (k < letters[1] ? 0 : 1);
            if (g > 1 || (!suffix && k < letters[0])) {
                return std::nullopt;  // numbers outside the two groups
            }
            groups[g].push_back(items[k].value);
        }
        hemi[0] = items[letters[0]].letter;
        hemi[1] = items[letters[1]].letter;
    } else {
        return std::nullopt;
    }
    double v[2];
    for (int k = 0; k < 2; ++k) {
        const auto& p = groups[k];
        if (p.empty() || p.size() > 3) {
            return std::nullopt;
        }
        double d = std::abs(p[0]);
        for (size_t j = 1; j < p.size(); ++j) {
            if (p[j] < 0.0 || p[j] >= 60.0 || p[j - 1] != std::floor(p[j - 1])) {
                return std::nullopt;  // minutes / seconds in range, only the last part has decimals
            }
            d += p[j] / (j == 1 ? 60.0 : 3600.0);
        }
        v[k] = p[0] < 0.0 ? -d : d;
    }
    LatLon r{v[0], v[1]};
    if (hemi[0]) {
        const bool lat_first = hemi[0] == 'N' || hemi[0] == 'S';
        const bool lon_second = hemi[1] == 'E' || hemi[1] == 'W';
        if (lat_first != lon_second) {
            return std::nullopt;  // "N .. N" / "E .. E"
        }
        if (!lat_first) {
            std::swap(r.lat, r.lon);
            std::swap(hemi[0], hemi[1]);
        }
        if (hemi[0] == 'S') {
            r.lat = -std::abs(r.lat);
        }
        if (hemi[1] == 'W') {
            r.lon = -std::abs(r.lon);
        }
    }
    if (std::abs(r.lat) > 90.0 || std::abs(r.lon) > 180.0) {
        return std::nullopt;
    }
    return r;
}

std::optional<LatLon> parseUtm(const std::vector<Token>& toks) {
    // zone[band] [N|S] easting northing
    std::vector<double> nums;
    int zone = 0;
    bool north = true;
    bool have_zone = false;
    for (const Token& t : toks) {
        if (t.number && !have_zone) {
            if (t.value < 1.0 || t.value > 60.0 || t.value != std::floor(t.value)) {
                return std::nullopt;
            }
            zone = static_cast<int>(t.value);
            have_zone = true;
            if (t.letter) {
                if (t.letter < 'C' || t.letter > 'X' || t.letter == 'I' || t.letter == 'O') {
                    return std::nullopt;
                }
                north = t.letter >= 'N';  // a latitude band
            }
        } else if (t.number) {
            nums.push_back(t.value);
        } else if (have_zone && nums.empty() && (t.letter == 'N' || t.letter == 'S')) {
            north = t.letter == 'N';  // a separate hemisphere letter
        } else if (have_zone && nums.empty() && t.letter >= 'C' && t.letter <= 'X') {
            north = t.letter >= 'N';
        }
    }
    if (!have_zone || nums.size() != 2 || nums[0] < 100000.0 || nums[0] > 900000.0 || nums[1] < 0.0 ||
        nums[1] > 10000000.0) {
        return std::nullopt;
    }
    return fromUtm(zone, north, nums[0], nums[1]);
}

}  // namespace

char utmBand(double lat) {
    static const char kBands[] = "CDEFGHJKLMNPQRSTUVWXX";
    const int i = std::clamp(static_cast<int>(std::floor((lat + 80.0) / 8.0)), 0, 20);
    return kBands[i];
}

Utm toUtm(double lat, double lon) {
    const Series& s = series();
    Utm u;
    u.zone = std::clamp(static_cast<int>(std::floor((lon + 180.0) / 6.0)) + 1, 1, 60);
    u.band = utmBand(lat);
    u.north = lat >= 0.0;
    const double phi = lat * kDeg;
    const double dl = lon * kDeg - centralMeridian(u.zone);
    const double c = 2.0 * std::sqrt(s.n) / (1.0 + s.n);
    const double t = std::sinh(std::atanh(std::sin(phi)) - c * std::atanh(c * std::sin(phi)));
    const double xi_p = std::atan2(t, std::cos(dl));
    const double eta_p = std::atanh(std::sin(dl) / std::sqrt(1.0 + t * t));
    double xi = xi_p, eta = eta_p;
    for (int j = 1; j <= 3; ++j) {
        xi += s.alpha[j - 1] * std::sin(2.0 * j * xi_p) * std::cosh(2.0 * j * eta_p);
        eta += s.alpha[j - 1] * std::cos(2.0 * j * xi_p) * std::sinh(2.0 * j * eta_p);
    }
    u.easting = kE0 + kK0 * s.a_rect * eta;
    u.northing = (u.north ? 0.0 : kNorthingSouth) + kK0 * s.a_rect * xi;
    return u;
}

LatLon fromUtm(int zone, bool north, double easting, double northing) {
    const Series& s = series();
    const double xi = (northing - (north ? 0.0 : kNorthingSouth)) / (kK0 * s.a_rect);
    const double eta = (easting - kE0) / (kK0 * s.a_rect);
    double xi_p = xi, eta_p = eta;
    for (int j = 1; j <= 3; ++j) {
        xi_p -= s.beta[j - 1] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
        eta_p -= s.beta[j - 1] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
    }
    const double chi = std::asin(std::sin(xi_p) / std::cosh(eta_p));
    double phi = chi;
    for (int j = 1; j <= 3; ++j) {
        phi += s.delta[j - 1] * std::sin(2.0 * j * chi);
    }
    const double lam = centralMeridian(zone) + std::atan2(std::sinh(eta_p), std::cos(xi_p));
    return {phi / kDeg, lam / kDeg};
}

std::string format(double lat, double lon, Fmt fmt) {
    char buf[96];
    switch (fmt) {
    case Fmt::Utm: {
        const Utm u = toUtm(lat, lon);
        std::snprintf(buf, sizeof(buf), "%d%c %06ld %07ld", u.zone, u.band, std::lround(std::floor(u.easting)),
                      std::lround(std::floor(u.northing)));
        return buf;
    }
    case Fmt::Dms: {
        auto dms = [](double v, char pos, char neg) {
            const char h = v >= 0.0 ? pos : neg;
            // round to 0.1" first so 59.95" carries into the minute
            long tenths = std::lround(std::abs(v) * 36000.0);
            const long d = tenths / 36000;
            tenths -= d * 36000;
            const long m = tenths / 600;
            tenths -= m * 600;
            char b[96];
            std::snprintf(b, sizeof(b), "%ld\xC2\xB0%02ld'%04.1f\" %c", d % 1000, m, tenths / 10.0, h);
            return std::string(b);
        };
        return dms(lat, 'N', 'S') + "  " + dms(lon, 'E', 'W');
    }
    case Fmt::Decimal:
        break;
    }
    std::snprintf(buf, sizeof(buf), "%.5f\xC2\xB0 %c  %.5f\xC2\xB0 %c", std::abs(lat), lat >= 0.0 ? 'N' : 'S',
                  std::abs(lon), lon >= 0.0 ? 'E' : 'W');
    return buf;
}

std::optional<LatLon> parse(const std::string& text, Fmt fmt) {
    const std::vector<Token> toks = tokenize(text);
    if (fmt == Fmt::Utm) {
        return parseUtm(toks);
    }
    return parseAngles(toks);
}

}  // namespace gcs::coords
