#include "openscan/openscan_app.h"
#include "openscan/openscan_calib.h"
#include "openscan/openscan_live.h"
#include "openscan/openscan_v4l2.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstdio>

#include <QApplication>
#include <QCheckBox>
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

static int grab_pair(openscan_cam *a, openscan_cam *b, std::vector<uint8_t> &ya, std::vector<uint8_t> &yb) {
    int w = openscan_cam_width(a), h = openscan_cam_height(a);
    std::vector<uint8_t> ja(2 * 1024 * 1024), jb(2 * 1024 * 1024);
    size_t na = 0, nb = 0;
    if (openscan_cam_grab_latest(a, ja.data(), ja.size(), &na, NULL) < 0) return -1;
    if (openscan_cam_grab_latest(b, jb.data(), jb.size(), &nb, NULL) < 0) return -1;
    if (jpeg_gray(ja.data(), na, w, h, ya) < 0 || jpeg_gray(jb.data(), nb, w, h, yb) < 0) return -1;
    return 0;
}

static int load_calib(const char *serial, const char *explicit_path, openscan_calib *cal) {
    char used[512];
    if (openscan_calib_ensure(serial, explicit_path, cal, used, sizeof used) != 0) return -1;
    return 0;
}

/* The picture is the 3D view plus the camera column. Mouse events on the
 * picture are forwarded in image pixels so dragging orbits the model. */
class Stage : public QLabel {
public:
    openscan_live *live = nullptr;
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
            openscan_live_mouse(live, cv::EVENT_LBUTTONDOWN, ix, iy, 0);
    }
    void mouseMoveEvent(QMouseEvent *e) override {
        int ix, iy;
        if (to_image(e->position().toPoint(), ix, iy))
            openscan_live_mouse(live, cv::EVENT_MOUSEMOVE, ix, iy, 0);
    }
    void mouseReleaseEvent(QMouseEvent *e) override {
        int ix, iy;
        if (to_image(e->position().toPoint(), ix, iy))
            openscan_live_mouse(live, cv::EVENT_LBUTTONUP, ix, iy, 0);
    }
    void wheelEvent(QWheelEvent *e) override {
        int ix, iy;
        if (!to_image(e->position().toPoint(), ix, iy)) return;
        int flags = e->angleDelta().y() > 0 ? 120 << 16 : -(120 << 16);
        openscan_live_mouse(live, cv::EVENT_MOUSEWHEEL, ix, iy, flags);
    }
};

class OpenScanWindow : public QMainWindow {
public:
    OpenScanWindow(openscan_cam *cam_a, openscan_cam *cam_b, openscan_live *model, QString serial_text,
              int exposure_a, int gain_a_init, int exposure_b, int gain_b_init)
        : a(cam_a), b(cam_b), live(model), serial(std::move(serial_text)),
          exp_a(exposure_a), gain_a(gain_a_init), exp_b(exposure_b), gain_b(gain_b_init) {
        setWindowTitle(QString("OpenScan " OPENSCAN_VERSION));
        resize(1280, 860);

        start = new QPushButton("Start scan");
        pause = new QPushButton("Pause");
        stop = new QPushButton("Stop");
        reset = new QPushButton("Reset");
        settings_btn = new QPushButton("Settings");
        mold_btn = new QPushButton("Mold");
        measured_btn = new QPushButton("Measured");
        exp = new QPushButton("Export…");
        start->setObjectName("start");
        settings_btn->setObjectName("settings");
        settings_btn->setCheckable(true);
        settings_btn->setChecked(true);
        mold_btn->setObjectName("shape");
        measured_btn->setObjectName("shape");
        mold_btn->setCheckable(true);
        measured_btn->setCheckable(true);
        exp->setObjectName("export");
        int saved_shape = QSettings("openscan", "openscan").value("shape", OPENSCAN_SHAPE_MOLD).toInt();
        if (saved_shape != OPENSCAN_SHAPE_MEASURED) saved_shape = OPENSCAN_SHAPE_MOLD;
        openscan_live_set_shape(live, saved_shape);
        mold_btn->setChecked(saved_shape == OPENSCAN_SHAPE_MOLD);
        measured_btn->setChecked(saved_shape == OPENSCAN_SHAPE_MEASURED);
        mold_btn->setToolTip("The scanned shape, made 8 mm thick. It is a solid of the measurement, not a round outline.");
        measured_btn->setToolTip("The surface the projector stripes measured. Each new view is aligned and added.");

        auto *tools = new QHBoxLayout;
        tools->setContentsMargins(12, 10, 12, 6);
        tools->setSpacing(8);
        tools->addWidget(start);
        tools->addWidget(pause);
        tools->addWidget(stop);
        tools->addWidget(reset);
        tools->addWidget(mold_btn);
        tools->addWidget(measured_btn);
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
            openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
            refresh_buttons();
        });
        connect(pause, &QPushButton::clicked, this, [this] {
            openscan_live_set_mode(live, OPENSCAN_MODE_PAUSE);
            refresh_buttons();
        });
        connect(stop, &QPushButton::clicked, this, [this] {
            openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
            refresh_buttons();
        });
        connect(reset, &QPushButton::clicked, this, [this] {
            openscan_live_reset(live);
            openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
            statusBar()->showMessage(serial + "   ready   scan cleared");
            refresh_buttons();
        });
        connect(exp, &QPushButton::clicked, this, [this] { export_mesh(); });
        connect(settings_btn, &QPushButton::toggled, settings, &QWidget::setVisible);
        connect(mold_btn, &QPushButton::clicked, this, [this] {
            mold_btn->setChecked(true);
            measured_btn->setChecked(false);
            if (openscan_live_shape(live) == OPENSCAN_SHAPE_MOLD) return;
            openscan_live_set_shape(live, OPENSCAN_SHAPE_MOLD);
            QSettings("openscan", "openscan").setValue("shape", OPENSCAN_SHAPE_MOLD);
            statusBar()->showMessage(serial + "   mold   model cleared");
        });
        connect(measured_btn, &QPushButton::clicked, this, [this] {
            measured_btn->setChecked(true);
            mold_btn->setChecked(false);
            if (openscan_live_shape(live) == OPENSCAN_SHAPE_MEASURED) return;
            openscan_live_set_shape(live, OPENSCAN_SHAPE_MEASURED);
            QSettings("openscan", "openscan").setValue("shape", OPENSCAN_SHAPE_MEASURED);
            statusBar()->showMessage(serial + "   measured   model cleared");
        });

        refresh_buttons();
        timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this] { tick(); });
        timer->start(15);
    }

    ~OpenScanWindow() override {
        if (timer) timer->stop();
        openscan_cam_close(a);
        openscan_cam_close(b);
        openscan_live_destroy(live);
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
        int mode = openscan_live_mode(live);
        start->setEnabled(mode != OPENSCAN_MODE_SCAN);
        pause->setEnabled(mode == OPENSCAN_MODE_SCAN);
        stop->setEnabled(mode != OPENSCAN_MODE_STOP);
        reset->setEnabled(mode != OPENSCAN_MODE_STOP || openscan_live_points(live) > 0);
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
        QSettings saved("openscan", "openscan");
        int distance = clamp_int(saved.value("distance-mm", (int)openscan_live_distance_mm(live)).toInt(), 100, 500);
        openscan_live_set_distance_mm(live, (float)distance);
        openscan_build build{};
        openscan_live_get_build(live, &build);
        build.near_mm = (float)clamp_int(saved.value("near-mm", (int)build.near_mm).toInt(), 60, 450);
        build.far_mm = (float)clamp_int(saved.value("far-mm", (int)build.far_mm).toInt(), 100, 800);
        build.stride = clamp_int(saved.value("stride", build.stride).toInt(), 1, 6);
        build.smooth = clamp_int(saved.value("smooth", build.smooth).toInt(), 0, 8);
        build.sweep_deg = clamp_int(saved.value("sweep-deg", build.sweep_deg).toInt(), 20, 140);
        build.relief = clamp_int(saved.value("relief", build.relief).toInt(), 0, 100);
        build.solid = saved.value("solid", build.solid).toInt() ? 1 : 0;
        build.flip = saved.value("flip", 1).toInt() ? 1 : 0;
        openscan_live_set_build(live, &build);

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
        auto *near_s = slider(60, 450, (int)build.near_mm);
        auto *far_s = slider(100, 800, (int)build.far_mm);
        auto *stride_s = slider(1, 6, build.stride);
        auto *smooth_s = slider(0, 8, build.smooth);
        auto *sweep_s = slider(20, 140, build.sweep_deg);
        auto *relief_s = slider(0, 100, build.relief);
        auto *solid_c = new QCheckBox("Solid mesh");
        solid_c->setChecked(build.solid);
        auto *flip_c = new QCheckBox("Flip scanner");
        flip_c->setChecked(build.flip);
        auto *ea_n = new QLabel;
        auto *ga_n = new QLabel;
        auto *eb_n = new QLabel;
        auto *gb_n = new QLabel;
        auto *dist_n = new QLabel;
        auto *near_n = new QLabel;
        auto *far_n = new QLabel;
        auto *stride_n = new QLabel;
        auto *smooth_n = new QLabel;
        auto *sweep_n = new QLabel;
        auto *relief_n = new QLabel;
        add_row(0, "Camera A exposure", ea, ea_n,
                "How long camera A collects light. 1 is 0.1 ms, 200 is 20 ms. Raise it if A is too dark.");
        add_row(1, "Camera A gain", ga, ga_n,
                "Amplifies camera A after the exposure. Keep this low so the dots stay sharp.");
        add_row(2, "Camera B exposure", eb, eb_n,
                "Camera B is the clean view. Lower this if the object is blown out white.");
        add_row(3, "Camera B gain", gb, gb_n, "Amplifies camera B. Usually lower than camera A.");
        add_row(4, "Distance (mm)", dist, dist_n,
                "Working distance. Mold uses it as the size of the solid. Measured uses it to pick the stripe band.");
        add_row(5, "Near (mm)", near_s, near_n,
                "Drops any 3D point closer than this. Same job as Kinect Near.");
        add_row(6, "Far (mm)", far_s, far_n,
                "Drops any 3D point farther than this. Same job as Kinect Far.");
        add_row(7, "Stride", stride_s, stride_n,
                "Sample step of the mesh. 1 is the full grid. 6 keeps every sixth row, so the model is coarser.");
        add_row(8, "Smooth", smooth_s, smooth_n,
                "How much depth is blurred before the triangles are built. 0 keeps every stripe. 8 clays the surface.");
        add_row(9, "Mold sweep (°)", sweep_s, sweep_n,
                "How far Mold wraps the camera outline into a solid. 20 is almost flat. 140 is a round body.");
        add_row(10, "Mold relief", relief_s, relief_n,
                "How strongly the camera shading pushes the Mold surface. 0 is a smooth solid. 100 cuts in eyes and nose.");
        ea_n->setText(exposure_text(ea->value()));
        eb_n->setText(exposure_text(eb->value()));

        auto remember = [this] {
            QSettings s("openscan", "openscan");
            s.setValue("exposure-a", exp_a);
            s.setValue("gain-a", gain_a);
            s.setValue("exposure-b", exp_b);
            s.setValue("gain-b", gain_b);
            s.setValue("distance-mm", (int)std::lround(openscan_live_distance_mm(live)));
            openscan_build b{};
            openscan_live_get_build(live, &b);
            s.setValue("near-mm", (int)std::lround(b.near_mm));
            s.setValue("far-mm", (int)std::lround(b.far_mm));
            s.setValue("stride", b.stride);
            s.setValue("smooth", b.smooth);
            s.setValue("sweep-deg", b.sweep_deg);
            s.setValue("relief", b.relief);
            s.setValue("solid", b.solid);
            s.setValue("flip", b.flip);
        };
        auto apply_build = [this, near_s, far_s, stride_s, smooth_s, sweep_s, relief_s, solid_c, flip_c, remember] {
            openscan_build b{};
            openscan_live_get_build(live, &b);
            b.near_mm = (float)near_s->value();
            b.far_mm = (float)far_s->value();
            b.stride = stride_s->value();
            b.smooth = smooth_s->value();
            b.sweep_deg = sweep_s->value();
            b.relief = relief_s->value();
            b.solid = solid_c->isChecked() ? 1 : 0;
            b.flip = flip_c->isChecked() ? 1 : 0;
            openscan_live_set_build(live, &b);
            remember();
        };
        auto apply_cam = [this, remember] {
            openscan_cam_set_exposure(a, exp_a, gain_a);
            openscan_cam_set_exposure(b, exp_b, gain_b);
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
            openscan_live_set_distance_mm(live, (float)v);
            dist_n->setText(QString("%1 mm").arg(v));
            remember();
        });
        dist_n->setText(QString("%1 mm").arg(dist->value()));
        auto wire = [&](QSlider *s, QLabel *n, const char *suffix) {
            n->setText(QString("%1%2").arg(s->value()).arg(suffix));
            connect(s, &QSlider::valueChanged, this, [n, suffix, apply_build](int v) {
                n->setText(QString("%1%2").arg(v).arg(suffix));
                apply_build();
            });
        };
        wire(near_s, near_n, " mm");
        wire(far_s, far_n, " mm");
        wire(stride_s, stride_n, "");
        wire(smooth_s, smooth_n, "");
        wire(sweep_s, sweep_n, "°");
        wire(relief_s, relief_n, "");
        solid_c->setToolTip("On builds triangles. Off keeps the point cloud, the way Kinect can show points instead of a solid map.");
        grid->addWidget(solid_c, 11, 1);
        connect(solid_c, &QCheckBox::toggled, this, [apply_build](bool) { apply_build(); });
        flip_c->setToolTip("Turns both cameras upside down. The Fox sensors are mounted that way, so leave this on unless the picture is upside down.");
        grid->addWidget(flip_c, 12, 1);
        connect(flip_c, &QCheckBox::toggled, this, [apply_build](bool) { apply_build(); });

        auto *hint = new QLabel(
            "Near, Far, Stride, Smooth and Solid change the points and the triangles on the next frame. "
            "Sweep and Relief change the Mold solid only. Measured stays the stripe surface. "
            "Flip scanner turns the picture and the model together. "
            "A missing calibration file is downloaded from 3DMakerpro's servers. "
            "Tested on the Fox. Other 3DMakerpro scanners may work.");
        hint->setWordWrap(true);
        grid->addWidget(hint, 13, 0, 1, 2);
        auto *defaults = new QPushButton("Defaults");
        defaults->setObjectName("quiet");
        defaults->setToolTip("Cameras 22/6 and 16/4, distance 220, near 80, far 550, stride 1, smooth 5, sweep 80, relief 40, solid on, flip on");
        grid->addWidget(defaults, 13, 2, Qt::AlignRight);
        connect(defaults, &QPushButton::clicked, this,
                [ea, ga, eb, gb, dist, near_s, far_s, stride_s, smooth_s, sweep_s, relief_s, solid_c, flip_c] {
            ea->setValue(22);
            ga->setValue(6);
            eb->setValue(16);
            gb->setValue(4);
            dist->setValue(220);
            near_s->setValue(80);
            far_s->setValue(550);
            stride_s->setValue(1);
            smooth_s->setValue(5);
            sweep_s->setValue(80);
            relief_s->setValue(40);
            solid_c->setChecked(true);
            flip_c->setChecked(true);
        });
        remember();
        return panel;
    }

    void export_mesh() {
        if (openscan_live_points(live) < 80) {
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
        if (openscan_live_write(live, path.toUtf8().constData(), &tris) != 0) {
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
            openscan_live_status st{};
            if (openscan_live_push(live, ya.data(), yb.data(), openscan_cam_width(a), openscan_cam_height(a), &st) == 0) {
                int w = 0, h = 0;
                const uint8_t *bg = openscan_live_preview_bgr(live, &w, &h);
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
                const char *mode = st.mode == OPENSCAN_MODE_SCAN ? "scanning" :
                                   st.mode == OPENSCAN_MODE_PAUSE ? "paused" : "ready";
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

    openscan_cam *a = nullptr;
    openscan_cam *b = nullptr;
    openscan_live *live = nullptr;
    QString serial;
    Stage *stage = nullptr;
    QPushButton *start = nullptr;
    QPushButton *pause = nullptr;
    QPushButton *stop = nullptr;
    QPushButton *reset = nullptr;
    QPushButton *settings_btn = nullptr;
    QPushButton *mold_btn = nullptr;
    QPushButton *measured_btn = nullptr;
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
        "QPushButton#shape:checked { background: #1d4e8a; border-color: #8eb7e8; }"
        "QPushButton#quiet { padding: 4px 12px; font-size: 13px; }"
        "QWidget#settingsPanel { background: #262a30; border-bottom: 1px solid #5c636e; }"
        "QStatusBar { background: #14161a; color: #d0d0d0; }"
        "QLabel#stage { background: #121418; }"
        "QSlider::groove:horizontal { height: 4px; background: #3a414b; border-radius: 2px; }"
        "QSlider::handle:horizontal { width: 14px; height: 14px; margin: -6px 0;"
        " background: #e6e6e6; border-radius: 7px; }");
}

int openscan_app_main(int argc, char **argv, const char *calib_path,
                 int exp_a, int gain_a, int exp_b, int gain_b,
                 double scale, double min_mm, double max_mm) {
    QApplication app(argc, argv);
    app.setOrganizationName("openscan");
    app.setApplicationName("openscan");
    style_app(app);
    QSettings saved("openscan", "openscan");
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
    if (openscan_find_cameras(path_a, path_b, sizeof path_a, serial, sizeof serial) < 0) {
        QMessageBox::critical(nullptr, "OpenScan",
                              "No scanner was found.\n\n"
                              "Plug in the scanner and open the app again.");
        return 1;
    }
    openscan_calib cal;
    if (load_calib(serial, calib_path, &cal) != 0) {
        QMessageBox::critical(nullptr, "OpenScan",
                              QString("No calibration for %1.\n\n"
                                      "The scanner serial is read from the device and the "
                                      "factory file is downloaded when it is not already on disk. "
                                      "That download failed.")
                                  .arg(serial));
        return 1;
    }
    openscan_cam *a = openscan_cam_open(path_a, 1280, 720, 10, 1);
    openscan_cam *b = openscan_cam_open(path_b, 1280, 720, 10, 1);
    if (!a || !b || openscan_cam_set_exposure(a, exp_a, gain_a) < 0 ||
        openscan_cam_set_exposure(b, exp_b, gain_b) < 0 ||
        openscan_cam_start(a) < 0 || openscan_cam_start(b) < 0) {
        openscan_cam_close(a);
        openscan_cam_close(b);
        QMessageBox::critical(nullptr, "OpenScan", "The cameras did not start streaming.");
        return 1;
    }
    openscan_scan_opts opt;
    opt.scale = scale;
    opt.min_mm = min_mm;
    opt.max_mm = max_mm;
    opt.edge_mm = 4.0;
    opt.preview_png = nullptr;
    openscan_live *live = openscan_live_create(&cal, &opt);
    if (!live) {
        openscan_cam_close(a);
        openscan_cam_close(b);
        return 1;
    }
    openscan_live_set_mode(live, OPENSCAN_MODE_STOP);

    OpenScanWindow window(a, b, live, QString::fromUtf8(serial), exp_a, gain_a, exp_b, gain_b);
    window.show();
    return app.exec();
}
