#include "fox/fox_app.h"
#include "fox/fox_calib.h"
#include "fox/fox_live.h"
#include "fox/fox_v4l2.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdio>

#include <QApplication>
#include <QCloseEvent>
#include <QFileDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QSettings>
#include <QSizePolicy>
#include <QSlider>
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
    FoxWindow(fox_cam *cam_a, fox_cam *cam_b, fox_live *model, QString serial_text,
              int exposure_a, int gain_a_init, int exposure_b, int gain_b_init)
        : a(cam_a), b(cam_b), live(model), serial(std::move(serial_text)),
          exp_a(exposure_a), gain_a(gain_a_init), exp_b(exposure_b), gain_b(gain_b_init) {
        setWindowTitle("Fox 3D");
        resize(1280, 860);

        start = new QPushButton("Start scan");
        pause = new QPushButton("Pause");
        stop = new QPushButton("Stop");
        reset = new QPushButton("Reset");
        settings_btn = new QPushButton("Settings");
        exp = new QPushButton("Export…");
        start->setObjectName("start");
        settings_btn->setObjectName("settings");
        settings_btn->setCheckable(true);
        settings_btn->setChecked(true);
        exp->setObjectName("export");

        auto *tools = new QHBoxLayout;
        tools->setContentsMargins(12, 10, 12, 6);
        tools->setSpacing(8);
        tools->addWidget(start);
        tools->addWidget(pause);
        tools->addWidget(stop);
        tools->addWidget(reset);
        tools->addWidget(settings_btn);
        tools->addStretch(1);
        tools->addWidget(exp);

        settings = build_settings();
        settings->setObjectName("settingsPanel");
        settings->setVisible(true);

        stage = new Stage;
        stage->setObjectName("stage");
        stage->live = live;
        stage->setAlignment(Qt::AlignCenter);
        stage->setMinimumSize(960, 540);
        stage->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        auto *page = new QWidget;
        auto *col = new QVBoxLayout(page);
        col->setContentsMargins(0, 0, 0, 0);
        col->setSpacing(0);
        col->addLayout(tools);
        col->addWidget(settings);
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
        connect(settings_btn, &QPushButton::toggled, settings, &QWidget::setVisible);

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

    QSlider *slider(int min, int max, int value) {
        auto *s = new QSlider(Qt::Horizontal);
        s->setRange(min, max);
        s->setValue(value);
        s->setMinimumWidth(180);
        s->setSingleStep(1);
        s->setPageStep(5);
        return s;
    }

    static int clamp_int(int v, int lo, int hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static QString exposure_text(int v) {
        return QString("%1  ·  %2 ms").arg(v).arg(v * 0.1, 0, 'f', 1);
    }

    QWidget *build_settings() {
        QSettings saved("fox3d", "fox3d");
        int distance = clamp_int(saved.value("distance-mm", (int)fox_live_distance_mm(live)).toInt(), 100, 500);
        fox_live_set_distance_mm(live, (float)distance);

        auto *panel = new QWidget;
        auto *grid = new QGridLayout(panel);
        grid->setContentsMargins(16, 4, 16, 10);
        grid->setHorizontalSpacing(12);
        grid->setVerticalSpacing(6);
        grid->setColumnStretch(1, 1);

        auto add_row = [&](int row, const char *name, QSlider *s, QLabel *value, const char *tip) {
            auto *lab = new QLabel(name);
            lab->setToolTip(tip);
            s->setToolTip(tip);
            value->setMinimumWidth(108);
            value->setText(QString::number(s->value()));
            grid->addWidget(lab, row, 0);
            grid->addWidget(s, row, 1);
            grid->addWidget(value, row, 2);
        };

        auto *ea = slider(1, 200, exp_a);
        auto *ga = slider(0, 100, gain_a);
        auto *eb = slider(1, 200, exp_b);
        auto *gb = slider(0, 100, gain_b);
        auto *dist = slider(100, 500, distance);
        auto *ea_n = new QLabel;
        auto *ga_n = new QLabel;
        auto *eb_n = new QLabel;
        auto *gb_n = new QLabel;
        auto *dist_n = new QLabel;
        add_row(0, "Camera A exposure", ea, ea_n,
                "How long camera A collects light. 1 is 0.1 ms, 200 is 20 ms. Raise it if A is too dark.");
        add_row(1, "Camera A gain", ga, ga_n,
                "Amplifies camera A after the exposure. Keep this low so the dots stay sharp.");
        add_row(2, "Camera B exposure", eb, eb_n,
                "Camera B is the clean view. Lower this if the object is blown out white.");
        add_row(3, "Camera B gain", gb, gb_n, "Amplifies camera B. Usually lower than camera A.");
        add_row(4, "Distance (mm)", dist, dist_n,
                "Distance to the object. This sets the size of the model in millimetres. "
                "The Fox works best around 200–400 mm. Reset the scan after changing it.");
        ea_n->setText(exposure_text(ea->value()));
        eb_n->setText(exposure_text(eb->value()));

        auto remember = [this] {
            QSettings s("fox3d", "fox3d");
            s.setValue("exposure-a", exp_a);
            s.setValue("gain-a", gain_a);
            s.setValue("exposure-b", exp_b);
            s.setValue("gain-b", gain_b);
            s.setValue("distance-mm", (int)std::lround(fox_live_distance_mm(live)));
        };
        auto apply_cam = [this, remember] {
            fox_cam_set_exposure(a, exp_a, gain_a);
            fox_cam_set_exposure(b, exp_b, gain_b);
            remember();
        };
        connect(ea, &QSlider::valueChanged, this, [this, ea_n, apply_cam](int v) {
            exp_a = v;
            ea_n->setText(exposure_text(v));
            apply_cam();
        });
        connect(ga, &QSlider::valueChanged, this, [this, ga_n, apply_cam](int v) {
            gain_a = v;
            ga_n->setText(QString::number(v));
            apply_cam();
        });
        connect(eb, &QSlider::valueChanged, this, [this, eb_n, apply_cam](int v) {
            exp_b = v;
            eb_n->setText(exposure_text(v));
            apply_cam();
        });
        connect(gb, &QSlider::valueChanged, this, [this, gb_n, apply_cam](int v) {
            gain_b = v;
            gb_n->setText(QString::number(v));
            apply_cam();
        });
        connect(dist, &QSlider::valueChanged, this, [this, dist_n, remember](int v) {
            fox_live_set_distance_mm(live, (float)v);
            dist_n->setText(QString("%1 mm").arg(v));
            remember();
        });
        dist_n->setText(QString("%1 mm").arg(dist->value()));

        auto *hint = new QLabel(
            "Camera changes apply immediately and are kept for the next launch. "
            "Reset the scan after changing distance, then keep the whole object in frame, including the top.");
        hint->setWordWrap(true);
        grid->addWidget(hint, 5, 0, 1, 2);
        auto *defaults = new QPushButton("Defaults");
        defaults->setObjectName("quiet");
        defaults->setToolTip("Camera A 22 / 6, camera B 16 / 4, distance 220 mm");
        grid->addWidget(defaults, 5, 2, Qt::AlignRight);
        connect(defaults, &QPushButton::clicked, this, [ea, ga, eb, gb, dist] {
            ea->setValue(22);
            ga->setValue(6);
            eb->setValue(16);
            gb->setValue(4);
            dist->setValue(220);
        });
        remember();
        return panel;
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
    QPushButton *settings_btn = nullptr;
    QPushButton *exp = nullptr;
    QWidget *settings = nullptr;
    int exp_a = 22, gain_a = 6, exp_b = 16, gain_b = 4;
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
        "QPushButton#settings { background: #1d4e8a; border-color: #3d7ec8; }"
        "QPushButton#settings:hover { background: #2460a8; }"
        "QPushButton#settings:checked { background: #163a68; border-color: #8eb7e8; }"
        "QPushButton#quiet { padding: 4px 12px; font-size: 13px; }"
        "QWidget#settingsPanel { background: #262a30; border-bottom: 1px solid #5c636e; }"
        "QStatusBar { background: #14161a; color: #d0d0d0; }"
        "QLabel#stage { background: #121418; }"
        "QSlider::groove:horizontal { height: 4px; background: #3a414b; border-radius: 2px; }"
        "QSlider::handle:horizontal { width: 14px; height: 14px; margin: -6px 0;"
        " background: #e6e6e6; border-radius: 7px; }");
}

int fox_app_main(int argc, char **argv, const char *calib_path,
                 int exp_a, int gain_a, int exp_b, int gain_b,
                 double scale, double min_mm, double max_mm) {
    QApplication app(argc, argv);
    app.setOrganizationName("fox3d");
    app.setApplicationName("fox3d");
    style_app(app);
    QSettings saved("fox3d", "fox3d");
    auto pick = [&](int given, const char *key, int fallback, int lo, int hi) {
        int v = given >= 0 ? given : saved.value(key, fallback).toInt();
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    };
    exp_a = pick(exp_a, "exposure-a", 22, 1, 200);
    gain_a = pick(gain_a, "gain-a", 6, 0, 100);
    exp_b = pick(exp_b, "exposure-b", 16, 1, 200);
    gain_b = pick(gain_b, "gain-b", 4, 0, 100);

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

    FoxWindow window(a, b, live, QString::fromUtf8(serial), exp_a, gain_a, exp_b, gain_b);
    window.show();
    return app.exec();
}
