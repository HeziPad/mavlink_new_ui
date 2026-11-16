#include "ui/main_window.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <QFont>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("mavlink_gcs_ui"));
    QApplication::setOrganizationName(QStringLiteral("mavlink_gcs_ui"));
    QApplication::setStyle(QStringLiteral("Fusion"));
    qSetMessagePattern(QStringLiteral("%{message}"));

    QFont base(QStringLiteral("DejaVu Sans"));
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
    QCommandLineOption video(QStringLiteral("video-port"),
                             QStringLiteral("UDP port of the RTP/H.264 stream (default %1)").arg(settings.video_port),
                             QStringLiteral("port"));
    QCommandLineOption reduced(QStringLiteral("reduced-motion"),
                               QStringLiteral("Disable glitch, scanline, ripple and pulse effects"));
    parser.addOption(udp);
    parser.addOption(video);
    parser.addOption(reduced);
    parser.process(app);

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
