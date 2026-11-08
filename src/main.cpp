#include "main_window.hpp"

#include <QApplication>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("mavlink_gcs_ui"));
    QApplication::setStyle(QStringLiteral("Fusion"));
    qSetMessagePattern(QStringLiteral("%{message}"));

    MainWindow window;
    window.show();
    return app.exec();
}
