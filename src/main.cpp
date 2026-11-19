#include "link/serial_transport.hpp"
#include "ui/app_icon.hpp"
#include "ui/theme.hpp"
#include "ui/main_window.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFont>
#include <QThreadPool>

#include <algorithm>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    // Qt's raster engine splits large fills, gradients and scaling across the
    // global thread pool (one thread per core). Measured at 1920x1200: two
    // threads paint a frame as fast as sixteen, and the other fourteen only
    // burned ~40 % CPU waking up. GCS_PAINT_THREADS overrides.
    const int paint_threads = qEnvironmentVariableIntValue("GCS_PAINT_THREADS");
    QThreadPool::globalInstance()->setMaxThreadCount(paint_threads > 0 ? paint_threads : 2);
    QApplication::setApplicationName(QStringLiteral("mavlink_gcs_ui"));
    QApplication::setOrganizationName(QStringLiteral("mavlink_gcs_ui"));
    QApplication::setStyle(QStringLiteral("Fusion"));
    QApplication::setWindowIcon(gcs::ui::appIcon());
    QApplication::setDesktopFileName(QStringLiteral("mavlink_gcs_ui"));
    qSetMessagePattern(QStringLiteral("%{message}"));

    if (!gcs::ui::theme::loadFonts()) {
        qWarning("bundled fonts missing from the resources: text will use a system font");
    }
    QFont base(QString::fromLatin1(gcs::ui::theme::kSansFamily));
    base.setPixelSize(13);
    QApplication::setFont(base);
    app.setStyleSheet(QStringLiteral(
        "QToolTip { background: #0a141c; color: #e6f1f8; border: 1px solid #3fe0ff; padding: 5px; "
        "font: 12px 'DejaVu Sans Mono'; }"));

    gcs::ui::AppSettings settings = gcs::ui::AppSettings::load();

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Minimal game-style GCS for an ArduPilot QuadPlane"));
    parser.addHelpOption();
    QCommandLineOption udp(QStringLiteral("udp"), QStringLiteral("Local UDP endpoint to listen on (default %1:%2)")
                                                      .arg(settings.host)
                                                      .arg(settings.port),
                           QStringLiteral("host:port"));
    QCommandLineOption serial(QStringLiteral("serial"),
                              QStringLiteral("Serial port of the autopilot or telemetry radio, optionally with the baud "
                                             "rate, e.g. /dev/ttyACM0, /dev/ttyUSB0:57600, COM4:115200"),
                              QStringLiteral("port[:baud]"));
    QCommandLineOption video(QStringLiteral("video-port"),
                             QStringLiteral("UDP port of the RTP/H.264 stream (default %1)").arg(settings.video_port),
                             QStringLiteral("port"));
    QCommandLineOption reduced(QStringLiteral("reduced-motion"),
                               QStringLiteral("Disable glitch, scanline, ripple and pulse effects"));
    parser.addOption(udp);
    parser.addOption(serial);
    parser.addOption(video);
    QCommandLineOption export_icon(QStringLiteral("export-icon"),
                                   QStringLiteral("Write the app icon as PNGs (16-512 px) into <dir> and exit"),
                                   QStringLiteral("dir"));
    parser.addOption(reduced);
    parser.addOption(export_icon);
    parser.process(app);

    if (parser.isSet(export_icon)) {
        const QDir dir(parser.value(export_icon));
        QDir().mkpath(dir.path());
        for (int s : {16, 24, 32, 48, 64, 128, 256, 512}) {
            if (!gcs::ui::appIconImage(s).save(dir.filePath(QStringLiteral("mavlink_gcs_ui_%1.png").arg(s)))) {
                return 1;
            }
        }
        return 0;
    }

    if (parser.isSet(udp)) {
        const QString v = parser.value(udp);
        const int colon = v.lastIndexOf(QLatin1Char(':'));
        bool ok = false;
        const int port = v.mid(colon + 1).toInt(&ok);
        if (colon <= 0 || !ok || port <= 0 || port > 65535) {
            qCritical("--udp expects host:port, e.g. 127.0.0.1:14551");
            return 2;
        }
        settings.host = v.left(colon);
        settings.port = port;
        settings.link_serial = false;
    }
    if (parser.isSet(serial)) {
        QString v = parser.value(serial);
        const int colon = v.lastIndexOf(QLatin1Char(':'));
        bool ok = false;
        const int baud = colon > 0 ? v.mid(colon + 1).toInt(&ok) : 0;
        if (ok) {
            const auto& bauds = gcs::link::serialBauds();
            if (std::find(bauds.begin(), bauds.end(), baud) == bauds.end()) {
                qCritical("--serial: baud must be one of 57600, 115200, 230400, 460800, 921600");
                return 2;
            }
            settings.serial_baud = baud;
            v = v.left(colon);
        }
        if (v.isEmpty()) {
            qCritical("--serial expects a port, e.g. /dev/ttyACM0 or COM4:57600");
            return 2;
        }
        settings.serial_port = v;
        settings.link_serial = true;
    }
    if (parser.isSet(video)) {
        settings.video_port = parser.value(video).toInt();
    }
    if (parser.isSet(reduced)) {
        settings.reduced_motion = true;
    }

    gcs::ui::MainWindow window(settings);
    window.show();
    return app.exec();
}
