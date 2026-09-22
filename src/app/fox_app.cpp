#include "fox/fox_app.h"
#include "fox/fox_calib.h"
#include "fox/fox_live.h"
#include "fox/fox_v4l2.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdio>

#include <QApplication>
#include <QCloseEvent>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

static int jpeg_gray(const uint8_t *jpg, size_t n, int w, int h, std::vector<uint8_t> &y) {
    cv::Mat encoded(1, (int)n, CV_8UC1, const_cast<uint8_t *>(jpg));
    cv::Mat gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (gray.empty() || gray.cols != w || gray.rows != h || !gray.isContinuous()) return -1;
    y.assign(gray.data, gray.data + (size_t)w * h);
    return 0;
}

static int grab_pair(fox_cam *a, fox_cam *b, std::vector<uint8_t> &ya, std::vector<uint8_t> &yb) {
    int w = fox_cam_width(a), h = fox_cam_height(a);
    std::vector<uint8_t> ja(2 * 1024 * 1024), jb(2 * 1024 * 1024);
    size_t na = 0, nb = 0;
    if (fox_cam_grab_latest(a, ja.data(), ja.size(), &na, NULL) < 0) return -1;
    if (fox_cam_grab_latest(b, jb.data(), jb.size(), &nb, NULL) < 0) return -1;
    if (jpeg_gray(ja.data(), na, w, h, ya) < 0 || jpeg_gray(jb.data(), nb, w, h, yb) < 0) return -1;
    return 0;
}

static int load_calib(const char *serial, const char *explicit_path, fox_calib *cal) {
    char used[512];
    if (fox_calib_ensure(serial, explicit_path, cal, used, sizeof used) != 0) return -1;
    return 0;
}

/* The picture is the 3D view plus the camera column. Mouse events on the
 * picture are forwarded in image pixels so dragging orbits the model. */
class Stage : public QLabel {
public:
    fox_live *live = nullptr;
    int img_w = 0;
    int img_h = 0;
    QSize drawn;
    QPoint origin;

    bool to_image(QPoint p, int &ix, int &iy) const {
        if (drawn.isEmpty() || img_w <= 0 || img_h <= 0) return false;
        int x = p.x() - origin.x();
        int y = p.y() - origin.y();
        if (x < 0 || y < 0 || x >= drawn.width() || y >= drawn.height()) return false;
        ix = x * img_w / drawn.width();
        iy = y * img_h / drawn.height();
        return true;
    }

protected:
    void mousePressEvent(QMouseEvent *e) override {
        int ix, iy;
        if (e->button() == Qt::LeftButton && to_image(e->position().toPoint(), ix, iy))
            fox_live_mouse(live, cv::EVENT_LBUTTONDOWN, ix, iy, 0);
    }
    void mouseMoveEvent(QMouseEvent *e) override {
        int ix, iy;
        if (to_image(e->position().toPoint(), ix, iy))
            fox_live_mouse(live, cv::EVENT_MOUSEMOVE, ix, iy, 0);
    }
    void mouseReleaseEvent(QMouseEvent *e) override {
        int ix, iy;
        if (to_image(e->position().toPoint(), ix, iy))
            fox_live_mouse(live, cv::EVENT_LBUTTONUP, ix, iy, 0);
    }
    void wheelEvent(QWheelEvent *e) override {
        int ix, iy;
        if (!to_image(e->position().toPoint(), ix, iy)) return;
        int flags = e->angleDelta().y() > 0 ? 120 << 16 : -(120 << 16);
        fox_live_mouse(live, cv::EVENT_MOUSEWHEEL, ix, iy, flags);
    }
};

class FoxWindow : public QMainWindow {
public:
    FoxWindow(fox_cam *cam_a, fox_cam *cam_b, fox_live *model, QString serial_text)
        : a(cam_a), b(cam_b), live(model), serial(std::move(serial_text)) {
        setWindowTitle("Fox 3D");
        resize(1280, 800);

        start = new QPushButton("Start scan");
        pause = new QPushButton("Pause");
        stop = new QPushButton("Stop");
        reset = new QPushButton("Reset");
        exp = new QPushButton("Export…");
        start->setObjectName("start");
        exp->setObjectName("export");

        auto *tools = new QHBoxLayout;
        tools->setContentsMargins(12, 10, 12, 6);
        tools->setSpacing(8);
        tools->addWidget(start);
        tools->addWidget(pause);
        tools->addWidget(stop);
        tools->addWidget(reset);
        tools->addStretch(1);
        tools->addWidget(exp);

        stage = new Stage;
        stage->live = live;
        stage->setAlignment(Qt::AlignCenter);
        stage->setMinimumSize(960, 540);
        stage->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        auto *page = new QWidget;
        auto *col = new QVBoxLayout(page);
        col->setContentsMargins(0, 0, 0, 0);
        col->setSpacing(0);
        col->addLayout(tools);
        col->addWidget(stage, 1);
        setCentralWidget(page);
        statusBar()->showMessage(serial + "   ready");

        connect(start, &QPushButton::clicked, this, [this] {
            fox_live_set_mode(live, FOX_MODE_SCAN);
            refresh_buttons();
        });
        connect(pause, &QPushButton::clicked, this, [this] {
            fox_live_set_mode(live, FOX_MODE_PAUSE);
            refresh_buttons();
        });
        connect(stop, &QPushButton::clicked, this, [this] {
            fox_live_set_mode(live, FOX_MODE_STOP);
            refresh_buttons();
        });
        connect(reset, &QPushButton::clicked, this, [this] {
            fox_live_reset(live);
            fox_live_set_mode(live, FOX_MODE_STOP);
            statusBar()->showMessage(serial + "   ready   scan cleared");
            refresh_buttons();
        });
        connect(exp, &QPushButton::clicked, this, [this] { export_mesh(); });

        refresh_buttons();
        timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this] { tick(); });
        timer->start(15);
    }

    ~FoxWindow() override {
        if (timer) timer->stop();
        fox_cam_close(a);
        fox_cam_close(b);
        fox_live_destroy(live);
        a = b = nullptr;
        live = nullptr;
    }

protected:
    void closeEvent(QCloseEvent *e) override {
        if (timer) timer->stop();
        e->accept();
    }

private:
    void refresh_buttons() {
        int mode = fox_live_mode(live);
        start->setEnabled(mode != FOX_MODE_SCAN);
        pause->setEnabled(mode == FOX_MODE_SCAN);
        stop->setEnabled(mode != FOX_MODE_STOP);
        reset->setEnabled(mode != FOX_MODE_STOP || fox_live_points(live) > 0);
    }

    void export_mesh() {
        if (fox_live_points(live) < 80) {
            QMessageBox::information(this, "Export mesh",
                                     "There is nothing to export yet.\n\n"
                                     "Start a scan, rotate the object until the 3D view fills in, "
                                     "then export.");
            return;
        }
        QString selected;
        QString path = QFileDialog::getSaveFileName(
            this, "Export mesh", "model.stl",
            "STL (*.stl);;Wavefront OBJ (*.obj);;PLY (*.ply)", &selected);
        if (path.isEmpty()) return;
        if (!path.contains('.')) {
            if (selected.contains("OBJ")) path += ".obj";
            else if (selected.contains("PLY")) path += ".ply";
            else path += ".stl";
        }
        int tris = 0;
        if (fox_live_write(live, path.toUtf8().constData(), &tris) != 0) {
            QMessageBox::warning(this, "Export mesh", "Could not write " + path);
            return;
        }
        QMessageBox::information(this, "Export mesh",
                                 QString("Saved %1\n%2 triangles.").arg(path).arg(tris));
    }

    void tick() {
        if (busy || !live || !a || !b) return;
        busy = true;
        if (grab_pair(a, b, ya, yb) == 0) {
            fox_live_status st{};
            if (fox_live_push(live, ya.data(), yb.data(), fox_cam_width(a), fox_cam_height(a), &st) == 0) {
                int w = 0, h = 0;
                const uint8_t *bg = fox_live_preview_bgr(live, &w, &h);
                if (bg && w > 0 && h > 0) {
                    QImage img(bg, w, h, w * 3, QImage::Format_BGR888);
                    QPixmap pm = QPixmap::fromImage(img.copy());
                    QPixmap scaled = pm.scaled(stage->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
                    stage->img_w = w;
                    stage->img_h = h;
                    stage->drawn = scaled.size();
                    stage->origin = QPoint((stage->width() - scaled.width()) / 2,
                                           (stage->height() - scaled.height()) / 2);
                    stage->setPixmap(scaled);
                }
                const char *mode = st.mode == FOX_MODE_SCAN ? "scanning" :
                                   st.mode == FOX_MODE_PAUSE ? "paused" : "ready";
                statusBar()->showMessage(
                    QString("%1   %2   scanned %3°   not scanned %4°   detail x%5   %6 tris")
                        .arg(serial, mode)
                        .arg(st.scanned_deg)
                        .arg(360 - st.scanned_deg)
                        .arg(std::max(1, st.detail))
                        .arg(st.points));
                refresh_buttons();
            }
        }
        busy = false;
    }

    fox_cam *a = nullptr;
    fox_cam *b = nullptr;
    fox_live *live = nullptr;
    QString serial;
    Stage *stage = nullptr;
    QPushButton *start = nullptr;
    QPushButton *pause = nullptr;
    QPushButton *stop = nullptr;
    QPushButton *reset = nullptr;
    QPushButton *exp = nullptr;
    QTimer *timer = nullptr;
    bool busy = false;
    std::vector<uint8_t> ya, yb;
};

static void style_app(QApplication &app) {
    app.setStyleSheet(
        "QMainWindow, QWidget { background: #1c1e22; color: #ececec; }"
        "QPushButton { background: #2c3138; color: #f2f2f2; border: 1px solid #5c636e;"
        " padding: 8px 18px; border-radius: 4px; font-size: 14px; }"
        "QPushButton:hover { background: #3a414b; }"
        "QPushButton:disabled { color: #8a8f98; background: #262a30; }"
        "QPushButton#start { background: #1d6b3a; border-color: #2e9a56; }"
        "QPushButton#start:hover { background: #238548; }"
        "QPushButton#export { background: #8a5a14; border-color: #c48a2a; }"
        "QPushButton#export:hover { background: #a56b18; }"
        "QStatusBar { background: #14161a; color: #d0d0d0; }"
        "QLabel { background: #121418; }");
}

int fox_app_main(int argc, char **argv, const char *calib_path,
                 int exp_a, int gain_a, int exp_b, int gain_b,
                 double scale, double min_mm, double max_mm) {
    QApplication app(argc, argv);
    style_app(app);

    char path_a[256], path_b[256], serial[64];
    if (fox_find_cameras(path_a, path_b, sizeof path_a, serial, sizeof serial) < 0) {
        QMessageBox::critical(nullptr, "Fox 3D",
                              "No Fox scanner was found.\n\n"
                              "Plug in the scanner and open the app again.");
        return 1;
    }
    fox_calib cal;
    if (load_calib(serial, calib_path, &cal) != 0) {
        QMessageBox::critical(nullptr, "Fox 3D",
                              QString("No calibration for %1.\n\n"
                                      "The scanner serial is read from the device and the "
                                      "factory file is downloaded when it is not already on disk. "
                                      "That download failed.")
                                  .arg(serial));
        return 1;
    }
    fox_cam *a = fox_cam_open(path_a, 1280, 720, 10, 1);
    fox_cam *b = fox_cam_open(path_b, 1280, 720, 10, 1);
    if (!a || !b || fox_cam_set_exposure(a, exp_a, gain_a) < 0 ||
        fox_cam_set_exposure(b, exp_b, gain_b) < 0 ||
        fox_cam_start(a) < 0 || fox_cam_start(b) < 0) {
        fox_cam_close(a);
        fox_cam_close(b);
        QMessageBox::critical(nullptr, "Fox 3D", "The cameras did not start streaming.");
        return 1;
    }
    fox_scan_opts opt;
    opt.scale = scale;
    opt.min_mm = min_mm;
    opt.max_mm = max_mm;
    opt.edge_mm = 4.0;
    opt.preview_png = nullptr;
    fox_live *live = fox_live_create(&cal, &opt);
    if (!live) {
        fox_cam_close(a);
        fox_cam_close(b);
        return 1;
    }
    fox_live_set_mode(live, FOX_MODE_STOP);

    FoxWindow window(a, b, live, QString::fromUtf8(serial));
    window.show();
    return app.exec();
}
