#include "vector_map.hpp"
#include "core/geo.hpp"

#include "core/gzip.hpp"
#include "core/mvt.hpp"
#include "offline_store.hpp"
#include "theme.hpp"

#include <QDir>
#include <QFontMetricsF>
#include <QMetaObject>
#include <QPainter>
#include <QPainterPath>
#include <QRunnable>

#include <algorithm>
#include <cmath>
#include <map>

namespace gcs::ui {

using namespace theme;

// ---- the file ------------------------------------------------------------------------------

struct VectorMap::Source {
    QFile file;
    pmtiles::Header header;
    std::vector<pmtiles::Entry> root;
    std::map<uint64_t, std::shared_ptr<std::vector<pmtiles::Entry>>> leaves;  // by leaf offset
    std::mutex m;

    std::optional<std::string> read(uint64_t off, uint64_t len) {
        if (!file.seek(static_cast<qint64>(off))) {
            return std::nullopt;
        }
        const QByteArray b = file.read(static_cast<qint64>(len));
        if (static_cast<uint64_t>(b.size()) != len) {
            return std::nullopt;
        }
        return std::string(b.constData(), static_cast<size_t>(b.size()));
    }
    std::optional<std::string> unpack(const std::string& raw, uint8_t compression) {
        if (compression == pmtiles::kGzip) {
            return gz::inflate(raw);
        }
        if (compression == pmtiles::kNone || compression == pmtiles::kUnknown) {
            return raw;
        }
        return std::nullopt;  // brotli / zstd: not supported
    }
    // the raw (decompressed) tile bytes, or nullopt
    std::optional<std::string> tile(int z, int x, int y) {
        const uint64_t id = pmtiles::zxyToId(static_cast<uint8_t>(z), static_cast<uint32_t>(x), static_cast<uint32_t>(y));
        std::lock_guard<std::mutex> lock(m);
        auto e = pmtiles::findEntry(root, id);
        for (int depth = 0; e && e->run_length == 0 && depth < 4; ++depth) {
            auto it = leaves.find(e->offset);
            if (it == leaves.end()) {
                const auto raw = read(header.leaf_offset + e->offset, e->length);
                const auto dir = raw ? unpack(*raw, header.internal_compression) : std::nullopt;
                const auto entries = dir ? pmtiles::decodeDirectory(*dir) : std::nullopt;
                if (!entries) {
                    return std::nullopt;
                }
                if (leaves.size() > 256) {
                    leaves.clear();
                }
                it = leaves.emplace(e->offset, std::make_shared<std::vector<pmtiles::Entry>>(*entries)).first;
            }
            e = pmtiles::findEntry(*it->second, id);
        }
        if (!e || e->run_length == 0) {
            return std::nullopt;
        }
        const auto raw = read(header.data_offset + e->offset, e->length);
        return raw ? unpack(*raw, header.tile_compression) : std::nullopt;
    }
};

// ---- style ---------------------------------------------------------------------------------

namespace {

const QColor kSea(9, 26, 38);
const QColor kLand(15, 22, 30);
const QColor kWater(12, 34, 50);
const QColor kRiver(24, 66, 92);
const QColor kGreen(17, 34, 30);
const QColor kSand(32, 31, 26);
const QColor kUrban(20, 28, 38);
const QColor kIndustry(24, 25, 36);
const QColor kMilitary(36, 22, 28);
const QColor kBuilding(30, 42, 56);
const QColor kLabel(214, 228, 238);
const QColor kHalo(5, 9, 13);

QColor landuseColor(std::string_view k) {
    static const std::map<std::string_view, QColor> m = {
        {"park", kGreen},          {"garden", kGreen},       {"grass", kGreen},        {"forest", kGreen},
        {"wood", kGreen},          {"meadow", kGreen},       {"nature_reserve", kGreen}, {"national_park", kGreen},
        {"scrub", kGreen},         {"village_green", kGreen}, {"recreation_ground", kGreen}, {"golf_course", kGreen},
        {"cemetery", QColor(19, 31, 29)}, {"farmland", QColor(19, 28, 25)}, {"orchard", QColor(18, 30, 26)},
        {"vineyard", QColor(18, 30, 26)}, {"grassland", kGreen},
        {"beach", kSand},          {"sand", kSand},          {"bare_rock", QColor(26, 27, 30)}, {"barren", kSand},
        {"residential", kUrban},   {"urban_area", kUrban},   {"pedestrian", QColor(24, 32, 42)},
        {"industrial", kIndustry}, {"commercial", kIndustry}, {"retail", kIndustry}, {"railway", kIndustry},
        {"military", kMilitary},   {"aerodrome", QColor(24, 30, 40)}, {"runway", QColor(40, 48, 60)},
        {"taxiway", QColor(34, 42, 54)}, {"school", QColor(22, 30, 40)}, {"university", QColor(22, 30, 40)},
        {"hospital", QColor(32, 24, 32)}, {"pitch", QColor(20, 36, 32)}, {"playground", kGreen},
        {"dog_park", kGreen},      {"zoo", kGreen},          {"glacier", QColor(40, 48, 56)},
    };
    const auto it = m.find(k);
    return it != m.end() ? it->second : QColor();
}

struct RoadStyle {
    QColor color;
    double base;  // width at z14, px
    int min_z;
};
RoadStyle roadStyle(std::string_view kind, std::string_view detail) {
    if (kind == "highway") return {QColor(104, 176, 204), 4.0, 5};
    if (kind == "major_road") {
        if (detail == "primary" || detail == "trunk") return {QColor(86, 148, 176), 3.4, 7};
        if (detail == "secondary") return {QColor(74, 128, 154), 2.8, 9};
        return {QColor(64, 108, 132), 2.2, 10};
    }
    if (kind == "minor_road") return {QColor(48, 78, 98), 1.5, 12};
    if (kind == "path") return {QColor(44, 66, 80), 0.9, 14};
    return {QColor(40, 62, 78), 1.0, 13};
}
double widthAt(double base, int z) {
    return std::clamp(base * std::pow(1.55, z - 14), 0.6, 40.0);
}

struct Ctx {
    QPainter& p;
    int z;          // display zoom (styling)
    double scale;   // tile units -> px
    QPointF off;    // px offset of the data tile's origin
    QRectF clip;    // the tile, in px
    bool english;
    std::vector<QRectF> labels;  // placed label boxes (collision)

    QPointF pt(const mvt::Point& q) const { return QPointF(q.x * scale, q.y * scale) + off; }
    QPolygonF poly(const std::vector<mvt::Point>& part) const {
        QPolygonF out;
        out.reserve(static_cast<int>(part.size()));
        for (const auto& q : part) out << pt(q);
        return out;
    }
    QPainterPath area(const mvt::Feature& f) const {
        QPainterPath path;
        path.setFillRule(Qt::WindingFill);
        for (const auto& ring : f.parts) {
            if (ring.size() >= 3) path.addPolygon(poly(ring));
        }
        return path;
    }
    std::string_view name(const mvt::Layer& l, const mvt::Feature& f) const {
        if (english) {
            const auto en = l.str(f, "name:en");
            if (!en.empty()) return en;
        }
        return l.str(f, "name");
    }
    bool place(const QRectF& r) {
        if (!clip.adjusted(1, 1, -1, -1).contains(r)) return false;  // kept inside its own tile
        for (const auto& o : labels) {
            if (o.intersects(r)) return false;
        }
        labels.push_back(r.adjusted(-3, -2, 3, 2));
        return true;
    }
};

void haloText(QPainter& p, const QPointF& at, const QString& s, const QColor& c) {
    p.setPen(kHalo);
    for (const QPointF d : {QPointF(-1, 0), QPointF(1, 0), QPointF(0, -1), QPointF(0, 1), QPointF(-1, -1),
                            QPointF(1, 1), QPointF(-1, 1), QPointF(1, -1)}) {
        p.drawText(at + d, s);
    }
    p.setPen(c);
    p.drawText(at, s);
}

void drawFills(Ctx& c, const mvt::Tile& t) {
    QPainter& p = c.p;
    p.setPen(Qt::NoPen);
    if (const auto* l = t.layer("earth")) {
        p.setBrush(kLand);
        for (const auto& f : l->features) {
            if (f.type == mvt::GeomType::Polygon) p.drawPath(c.area(f));
        }
    }
    for (const char* name : {"landcover", "landuse"}) {
        const auto* l = t.layer(name);
        if (!l) continue;
        for (const auto& f : l->features) {
            if (f.type != mvt::GeomType::Polygon) continue;
            const QColor col = landuseColor(l->str(f, "kind"));
            if (!col.isValid()) continue;
            p.setBrush(col);
            p.drawPath(c.area(f));
        }
    }
    if (const auto* l = t.layer("water")) {
        for (const auto& f : l->features) {
            if (f.type == mvt::GeomType::Polygon) {
                p.setPen(Qt::NoPen);
                p.setBrush(l->str(f, "kind") == "ocean" ? kSea : kWater);
                p.drawPath(c.area(f));
            } else if (f.type == mvt::GeomType::Line && c.z >= 9) {
                const auto k = l->str(f, "kind");
                const double w = k == "river" ? widthAt(1.8, c.z) : widthAt(0.9, c.z);
                p.setPen(QPen(kRiver, w, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
                p.setBrush(Qt::NoBrush);
                for (const auto& part : f.parts) p.drawPolyline(c.poly(part));
            }
        }
    }
    if (c.z >= 14) {
        if (const auto* l = t.layer("buildings")) {
            p.setPen(QPen(QColor(44, 60, 78), 0.6));
            p.setBrush(kBuilding);
            for (const auto& f : l->features) {
                if (f.type == mvt::GeomType::Polygon && l->str(f, "kind") == "building") p.drawPath(c.area(f));
            }
        }
    }
}

void drawLines(Ctx& c, const mvt::Tile& t) {
    QPainter& p = c.p;
    p.setBrush(Qt::NoBrush);
    if (const auto* l = t.layer("boundaries")) {
        for (const auto& f : l->features) {
            const bool country = l->str(f, "kind") == "country";
            if (!country && c.z < 8) continue;
            QPen pen(country ? QColor(150, 110, 150) : QColor(90, 80, 104), country ? 1.6 : 1.0, Qt::DashLine);
            p.setPen(pen);
            for (const auto& part : f.parts) p.drawPolyline(c.poly(part));
        }
    }
    if (const auto* l = t.layer("transit")) {
        if (c.z >= 10) {
            QPen rail(QColor(96, 104, 120), c.z >= 14 ? 1.6 : 1.0, Qt::DashLine);
            p.setPen(rail);
            for (const auto& f : l->features) {
                if (l->str(f, "kind") != "rail" || f.type != mvt::GeomType::Line) continue;
                for (const auto& part : f.parts) p.drawPolyline(c.poly(part));
            }
        }
    }
    const auto* roads = t.layer("roads");
    if (!roads) return;
    // draw small roads first so the big ones sit on top
    std::vector<std::pair<int, const mvt::Feature*>> order;
    for (const auto& f : roads->features) {
        if (f.type != mvt::GeomType::Line) continue;
        const auto k = roads->str(f, "kind");
        const int rank = k == "highway" ? 4 : k == "major_road" ? 3 : k == "minor_road" ? 2 : k == "path" ? 0 : 1;
        order.emplace_back(rank, &f);
    }
    std::stable_sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& [rank, f] : order) {
        const RoadStyle st = roadStyle(roads->str(*f, "kind"), roads->str(*f, "kind_detail"));
        if (c.z < st.min_z) continue;
        const bool tunnel = roads->num(*f, "is_tunnel", 0) != 0 ||
                            (roads->prop(*f, "is_tunnel") && roads->prop(*f, "is_tunnel")->b);
        QPen pen(tunnel ? st.color.darker(140) : st.color, widthAt(st.base, c.z),
                 tunnel ? Qt::DashLine : Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        if (rank == 0) pen.setStyle(Qt::DotLine);
        p.setPen(pen);
        for (const auto& part : f->parts) p.drawPolyline(c.poly(part));
    }
}

void drawLabels(Ctx& c, const mvt::Tile& t) {
    QPainter& p = c.p;
    // places: biggest first
    if (const auto* l = t.layer("places")) {
        std::vector<std::pair<double, const mvt::Feature*>> order;
        for (const auto& f : l->features) {
            if (f.type != mvt::GeomType::Point || f.parts.empty()) continue;
            if (l->num(f, "min_zoom", 0) > c.z + 0.5) continue;
            order.emplace_back(l->num(f, "population_rank", 0), &f);
        }
        std::stable_sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        for (const auto& [rank, f] : order) {
            const auto kind = l->str(*f, "kind");
            const auto detail = l->str(*f, "kind_detail");
            const QString text = QString::fromUtf8(c.name(*l, *f).data(), static_cast<int>(c.name(*l, *f).size()));
            if (text.isEmpty() || kind == "country" || kind == "region") continue;
            for (const auto& pt : f->parts.front()) {
                const QPointF at = c.pt(pt);
                if (!c.clip.contains(at)) continue;  // points in the tile buffer belong to the neighbour
                double px = 10.0;
                bool bold = false;
                QColor col = kLabel;
                if (detail == "city") { px = 15.0; bold = true; }
                else if (detail == "town") { px = 13.0; bold = true; }
                else if (detail == "village") { px = 11.5; }
                else { px = 10.5; col = QColor(150, 170, 186); }
                QFont font(QStringLiteral("DejaVu Sans"));
                font.setPixelSize(static_cast<int>(px));
                font.setBold(bold);
                const QFontMetricsF fm(font);
                const double w = fm.horizontalAdvance(text);
                QRectF box(at.x() - w / 2.0, at.y() - fm.ascent() / 2.0 - 2, w, fm.height());
                // nudge into the tile rather than dropping labels near an edge
                box.moveLeft(std::clamp(box.left(), c.clip.left() + 2, c.clip.right() - 2 - w));
                box.moveTop(std::clamp(box.top(), c.clip.top() + 2, c.clip.bottom() - 2 - box.height()));
                if (!c.place(box)) continue;
                p.setFont(font);
                haloText(p, QPointF(box.left(), box.top() + fm.ascent()), text, col);
            }
        }
    }
    // water names
    if (const auto* l = t.layer("water")) {
        QFont font(QStringLiteral("DejaVu Sans"));
        font.setPixelSize(11);
        font.setItalic(true);
        const QFontMetricsF fm(font);
        for (const auto& f : l->features) {
            if (f.type != mvt::GeomType::Point || f.parts.empty()) continue;
            const auto n = c.name(*l, f);
            if (n.empty()) continue;
            const QString text = QString::fromUtf8(n.data(), static_cast<int>(n.size()));
            const QPointF at = c.pt(f.parts.front().front());
            if (!c.clip.contains(at)) continue;
            const double w = fm.horizontalAdvance(text);
            const QRectF box(at.x() - w / 2.0, at.y() - fm.height() / 2.0, w, fm.height());
            if (!c.place(box)) continue;
            p.setFont(font);
            haloText(p, QPointF(box.left(), box.top() + fm.ascent()), text, QColor(90, 150, 190));
        }
    }
    // road names along their longest straight piece
    if (c.z >= 15) {
        if (const auto* l = t.layer("roads")) {
            QFont font(QStringLiteral("DejaVu Sans"));
            font.setPixelSize(10);
            const QFontMetricsF fm(font);
            p.setFont(font);
            for (const auto& f : l->features) {
                const auto n = c.name(*l, f);
                if (n.empty() || f.type != mvt::GeomType::Line) continue;
                const QString text = QString::fromUtf8(n.data(), static_cast<int>(n.size()));
                const double w = fm.horizontalAdvance(text);
                QLineF best;
                for (const auto& part : f.parts) {
                    for (size_t i = 1; i < part.size(); ++i) {
                        const QLineF seg(c.pt(part[i - 1]), c.pt(part[i]));
                        if (seg.length() > best.length() && c.clip.contains(seg.p1()) && c.clip.contains(seg.p2())) {
                            best = seg;
                        }
                    }
                }
                if (best.length() < w + 16) continue;
                double ang = std::atan2(best.dy(), best.dx()) * 180.0 / geo::kPi;
                if (ang > 90) ang -= 180;
                if (ang < -90) ang += 180;  // never upside down
                const QPointF mid = best.center();
                // collision on the rotated label's bounding box
                QTransform tr;
                tr.translate(mid.x(), mid.y());
                tr.rotate(ang);
                const QRectF local(-w / 2.0, -fm.height() / 2.0, w, fm.height());
                const QRectF bounds = tr.mapRect(local);
                if (!c.place(bounds)) continue;
                p.save();
                p.setTransform(tr, true);
                haloText(p, QPointF(-w / 2.0, fm.ascent() - fm.height() / 2.0), text, QColor(170, 196, 214));
                p.restore();
            }
        }
    }
}

// Renders display tile (z, x, y) from data tile bytes at zoom dz levels up.
QImage render(const std::string& bytes, int z, int x, int y, int data_z, bool english) {
    QImage img(256, 256, QImage::Format_ARGB32_Premultiplied);
    img.fill(kSea);
    mvt::Tile t;
    mvt::decode(bytes, t);
    uint32_t extent = 4096;
    if (!t.layers.empty()) extent = t.layers.front().extent;
    const int dz = z - data_z;
    const int sub_x = x - ((x >> dz) << dz);
    const int sub_y = y - ((y >> dz) << dz);
    const double scale = 256.0 / extent * (1 << dz);
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::TextAntialiasing);
    const QRectF clip(0, 0, 256, 256);
    p.setClipRect(clip);
    Ctx c{p, z, scale, QPointF(-sub_x * 256.0, -sub_y * 256.0), clip, english, {}};
    drawFills(c, t);
    drawLines(c, t);
    drawLabels(c, t);
    return img;
}

class RenderJob : public QRunnable {
public:
    RenderJob(std::shared_ptr<VectorMap::Source> src, VectorMap* owner, int z, int x, int y, int max_z, bool en,
              quint64 gen)
        : src_(std::move(src)), owner_(owner), z_(z), x_(x), y_(y), max_z_(max_z), en_(en), gen_(gen) {}
    void run() override {
        const int dz = std::max(0, z_ - max_z_);
        const int data_z = z_ - dz;
        const auto bytes = src_->tile(data_z, x_ >> dz, y_ >> dz);
        QImage img;
        if (bytes) {
            img = render(*bytes, z_, x_, y_, data_z, en_);
        }
        QMetaObject::invokeMethod(owner_, "deliver", Qt::QueuedConnection, Q_ARG(int, z_), Q_ARG(int, x_),
                                  Q_ARG(int, y_), Q_ARG(quint64, gen_), Q_ARG(QImage, img));
    }

private:
    std::shared_ptr<VectorMap::Source> src_;
    VectorMap* owner_;
    int z_, x_, y_, max_z_;
    bool en_;
    quint64 gen_;
};

}  // namespace

// ---- the object ------------------------------------------------------------------------------

VectorMap::VectorMap(QObject* parent) : QObject(parent) {
    pool_.setMaxThreadCount(2);
    reopen();
}

VectorMap::~VectorMap() {
    pool_.clear();
    pool_.waitForDone();
}

bool VectorMap::reopen() {
    pool_.clear();
    pool_.waitForDone();
    src_.reset();
    pending_.clear();
    ++gen_;
    const QDir dir(offline::folder());
    QStringList files = dir.entryList({QStringLiteral("*.pmtiles")}, QDir::Files, QDir::Name);
    files.removeAll(QStringLiteral("map.pmtiles"));
    if (QFileInfo::exists(dir.filePath(QStringLiteral("map.pmtiles")))) {
        files.prepend(QStringLiteral("map.pmtiles"));  // the downloaded one first
    }
    for (const QString& name : files) {
        auto s = std::make_shared<Source>();
        s->file.setFileName(dir.filePath(name));
        if (!s->file.open(QIODevice::ReadOnly)) continue;
        const QByteArray head = s->file.read(pmtiles::kHeaderSize);
        const auto h = pmtiles::parseHeader(std::string_view(head.constData(), static_cast<size_t>(head.size())));
        if (!h || h->tile_type != pmtiles::kMvt) continue;
        s->header = *h;
        const auto raw = s->read(h->root_offset, h->root_length);
        const auto dir_bytes = raw ? s->unpack(*raw, h->internal_compression) : std::nullopt;
        const auto root = dir_bytes ? pmtiles::decodeDirectory(*dir_bytes) : std::nullopt;
        if (!root) continue;
        s->root = *root;
        src_ = s;
        return true;
    }
    return false;
}

QString VectorMap::path() const {
    return src_ ? src_->file.fileName() : QString();
}

int VectorMap::maxDataZoom() const {
    return src_ ? src_->header.max_zoom : 0;
}

void VectorMap::setEnglish(bool english) {
    if (english == english_) return;
    english_ = english;
    pool_.clear();
    pending_.clear();
    ++gen_;
}

void VectorMap::request(int z, int x, int y) {
    if (!src_) return;
    const quint64 key = (static_cast<quint64>(z) << 58) | (static_cast<quint64>(x) << 29) | static_cast<quint64>(y);
    if (pending_.contains(key)) return;
    pending_.insert(key);
    pool_.start(new RenderJob(src_, this, z, x, y, src_->header.max_zoom, english_, gen_));
}

void VectorMap::deliver(int z, int x, int y, quint64 gen, const QImage& img) {
    const quint64 key = (static_cast<quint64>(z) << 58) | (static_cast<quint64>(x) << 29) | static_cast<quint64>(y);
    pending_.remove(key);
    if (gen != gen_) return;  // made for an older style / file
    emit rendered(z, x, y, gen, img);
}

}  // namespace gcs::ui
