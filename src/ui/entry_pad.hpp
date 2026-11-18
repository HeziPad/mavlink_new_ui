#pragma once

#include "core/altitude.hpp"
#include "core/coords.hpp"

#include <QWidget>

#include <functional>
#include <optional>
#include <vector>

class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace gcs::ui {

// Touch-first modal for typed input: coordinates (in the format chosen in
// Settings: decimal / DMS / UTM), altitudes (ASL / AGL / AFL, checked against
// the ground at the point) and hold times. Each number has its own box with a
// visible cursor; tap a box (or a place in it) to move there. Letters are
// never typed: hemispheres are toggle buttons. Full-window, dims what's
// behind. A physical keyboard types into the focused box (digits, '.',
// Backspace / Delete, arrows, Tab = next box, Enter = OK); it is the only
// place the keyboard is used. Results come back through the open*() callback.
class EntryPad : public QWidget {
    Q_OBJECT

public:
    explicit EntryPad(QWidget* parent);

    using GroundFn = std::function<std::optional<double>(double lat, double lon)>;

    // Context for previews and checks
    void setAircraft(std::optional<coords::LatLon> pos) { aircraft_ = pos; }
    void setHome(std::optional<coords::LatLon> home) { home_ = home; }
    void setGroundLookup(GroundFn fn) { ground_ = std::move(fn); }

    // A position. `initial` pre-fills the boxes.
    void openCoordinate(const QString& title, std::optional<coords::LatLon> initial,
                        std::function<void(double lat, double lon)> done);

    // A height at (lat, lon). `initial_rel` is the current height above home;
    // `land` the landing point (for AFL); `min_agl` the clearance required.
    // `done` receives the height above home to command.
    void openAltitude(const QString& title, double lat, double lon, double initial_rel, double home_amsl,
                      std::optional<coords::LatLon> land, int ref, double min_agl,
                      std::function<void(double rel_home, int ref)> done);

    // A duration in minutes and seconds, 0..max_s; 0 = none ("fly through").
    void openDuration(const QString& title, double seconds, double max_s, std::function<void(double seconds)> done);

    bool isOpen() const { return isVisible(); }
    void close();

    static constexpr double kMaxFromHome = 50000.0;  // m: further is a typo, not a flight

signals:
    void closed();
    void refChanged(int ref);  // the altitude reference the operator picked (remembered)

protected:
    void paintEvent(QPaintEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    bool eventFilter(QObject* obj, QEvent* e) override;

private:
    enum class Kind { Coordinate, Altitude, Duration };
    // one number: its box, and the length at which typing moves on to the next box (0 = never)
    struct Box {
        QLineEdit* edit = nullptr;
        int advance_at = 0;
    };
    // a two-way toggle (N|S, E|W)
    struct Toggle {
        QPushButton* a = nullptr;
        QPushButton* b = nullptr;
        bool first = true;
    };
    void build();
    void showFor(Kind kind);  // after the boxes are built
    void clearBoxes();
    QWidget* newRow();
    void addLabel(QWidget* row, const QString& text, const char* name = "padUnit");
    void addBox(QWidget* row, int chars, int decimals, int advance_at, const QString& placeholder);
    void addToggle(QWidget* row, const QString& a, const QString& b, bool first);
    void setPresets(const std::vector<std::pair<QString, double>>& chips);  // label, seconds
    void focusBox(int i);
    int activeIndex() const;
    QString boxText(int i) const;
    double boxValue(int i, bool* ok = nullptr) const;
    void setBoxText(int i, const QString& t);
    void typeText(const QString& s);
    void setDuration(double seconds);
    void refresh();  // re-read the boxes, update preview and OK
    void refreshCoordinate(QString& preview, bool& ok);
    void refreshDuration(QString& preview, bool& ok);
    void refreshAltitude(QString& preview, bool& ok);
    void accept();
    void layoutCard();
    altitude::Grounds grounds() const;

    Kind kind_ = Kind::Coordinate;
    coords::Fmt fmt_ = coords::Fmt::Decimal;
    int ref_ = 0;
    double min_agl_ = altitude::kMinAgl;
    double lat_ = 0.0, lon_ = 0.0, home_amsl_ = 0.0;
    std::optional<coords::LatLon> land_;
    std::optional<coords::LatLon> aircraft_, home_;
    GroundFn ground_;
    std::function<void(double, double)> coord_done_;
    std::function<void(double, int)> alt_done_;
    std::function<void(double)> dur_done_;
    double max_value_ = 0.0;
    std::optional<double> value_;  // a valid duration (s)
    std::optional<coords::LatLon> parsed_;
    std::optional<double> rel_;      // valid altitude, as height above home
    double min_rel_ = 0.0;

    std::vector<Box> boxes_;
    std::vector<Toggle> toggles_;
    QLineEdit* active_ = nullptr;    // the box the keys type into

    QWidget* card_ = nullptr;
    QLabel* title_ = nullptr;
    QWidget* fields_ = nullptr;      // the rows of boxes (rebuilt per open)
    QLabel* preview_ = nullptr;
    std::vector<QPushButton*> ref_buttons_;
    QWidget* ref_row_ = nullptr;
    QWidget* nudge_keys_ = nullptr;  // -10 / +10 (altitude)
    QWidget* presets_ = nullptr;     // quick chips (duration)
    QPushButton* dot_key_ = nullptr;
    QPushButton* ok_ = nullptr;
    QPushButton* use_min_ = nullptr;
    QTimer* poll_ = nullptr;         // terrain tiles arrive asynchronously
};

}  // namespace gcs::ui
