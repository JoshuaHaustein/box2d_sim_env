//
// Created by joshua on 6/26/17.
//

#include <Box2D/Common/b2Math.h>
#include <Box2D/Box2D.h>
#include <sim_env/Box2DWorldViewer.h>
#include <QtGui/QPushButton>
#include <QtGui/QPainter>
#include <QWheelEvent>

//////////////////////////////////////// Box2DDrawingInterface ////////////////////////////////////////
sim_env::viewer::Box2DDrawingInterface::Box2DDrawingInterface(sim_env::Box2DWorldPtr world,
    int width, int height, int min_width, int min_height) {
    _world = world;
    _desired_size = QSize(width, height);
    _min_size = QSize(min_width, min_height);
    _arrow_length = 10.0f;
    _zoom_level = 1.0f;
    SetFlags(e_shapeBit);
}

sim_env::viewer::Box2DDrawingInterface::~Box2DDrawingInterface() {
}

void sim_env::viewer::Box2DDrawingInterface::DrawPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("Draw polygon was called", "[sim_env::viewer::Box2DDrawingInterface::DrawPolygon]");
    Polygon new_polygon;
    createPolygon(vertices, vertexCount, color, new_polygon);
    new_polygon.solid = false;
    _polygons.push_back(new_polygon);
}

void sim_env::viewer::Box2DDrawingInterface::DrawSolidPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("Draw solid polygon was called", "[sim_env::viewer::Box2DDrawingInterface::DrawSolidPolygon]");
    Polygon new_polygon;
    createPolygon(vertices, vertexCount, color, new_polygon);
    new_polygon.solid = false;
    _polygons.push_back(new_polygon);
}

void sim_env::viewer::Box2DDrawingInterface::DrawCircle(const b2Vec2 &center, float32 radius, const b2Color &color) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("Draw circle was called", "[sim_env::viewer::Box2DDrawingInterface::DrawCircle]");
    Circle circle;
    createCircle(center, radius, color, circle);
    circle.solid = false;
    _circles.push_back(circle);
}

void sim_env::viewer::Box2DDrawingInterface::DrawSolidCircle(const b2Vec2 &center, float32 radius, const b2Vec2 &axis,
                                                     const b2Color &color) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("Draw solid circle was called", "[sim_env::viewer::Box2DDrawingInterface::DrawSolidCircle]");
    Circle circle;
    createCircle(center, radius, color, circle);
    circle.solid = true;
    _circles.push_back(circle);
}

void sim_env::viewer::Box2DDrawingInterface::DrawSegment(const b2Vec2 &p1, const b2Vec2 &p2, const b2Color &color) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("Draw segment was called", "[sim_env::viewer::Box2DDrawingInterface::DrawSegment]");
    LineSegment line_segment = sim_env::viewer::LineSegment();
    line_segment.color.setRgb(int(color.r * 255), int(color.g * 255), int(color.b * 255), int(color.a * 255));
    line_segment.point_a = toScreenPoint(p1);
    line_segment.point_b = toScreenPoint(p2);
    _lines.push_back(line_segment);
}

void sim_env::viewer::Box2DDrawingInterface::DrawTransform(const b2Transform &xf) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("Draw transform was called", "[sim_env::viewer::Box2DDrawingInterface::DrawTransform]");
    // red line for x_axis
    b2Vec2 dir(xf.q.GetXAxis());
    dir *= toWorldLength(_arrow_length);
    LineSegment x_axis = sim_env::viewer::LineSegment();
    x_axis.point_a = toScreenPoint(xf.p);
    x_axis.point_b = toScreenPoint(xf.p + dir);
    x_axis.color.setRgb(255, 0, 0);
    _lines.push_back(x_axis);

    // green line for y-axis
    dir = xf.q.GetYAxis();
    dir *= toWorldLength(_arrow_length);
    LineSegment y_axis = sim_env::viewer::LineSegment();
    y_axis.point_a = toScreenPoint(xf.p);
    y_axis.point_b = toScreenPoint(xf.p + dir);
    y_axis.color.setRgb(0, 255, 0);
    _lines.push_back(y_axis);
}

QSize sim_env::viewer::Box2DDrawingInterface::minimumSizeHint() const {
    return _min_size;
}

QSize sim_env::viewer::Box2DDrawingInterface::sizeHint() const {
    return _desired_size;
}

void sim_env::viewer::Box2DDrawingInterface::paintEvent(QPaintEvent *event) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("paintEvent", "[sim_env::viewer::Box2DDrawingInterface]");
    if (_world.expired()) {
        // TODO print error message on GUI
        return;
    }
    auto world = _world.lock();
    logger->logDebug("Calling drawWorld", "[sim_env::viewer::Box2DDrawingInterface]");
    // let box2d call our callbacks
    world->drawWorld(shared_from_this());
    // Now our buffers should be filled, so let's draw
    QPainter painter(this);
    painter.eraseRect(0, 0, width(), height());
    painter.scale(_zoom_level, _zoom_level);
    QBrush brush(Qt::BrushStyle::SolidPattern);
    QPen pen;
    // first let's draw all polygons
    for (const Polygon& polygon : _polygons) {
        logger->logDebug("Drawing a polygon", "[sim_env::viewer::Box2DDrawingInterface::paintEvent]");
        pen.setColor(polygon.color);
        brush.setColor(polygon.color);
        if (polygon.solid) {
            brush.setStyle(Qt::BrushStyle::SolidPattern);
        } else {
            brush.setStyle(Qt::BrushStyle::NoBrush);
        }
        painter.setPen(pen);
        painter.setBrush(brush);
        painter.drawPolygon(polygon.vertices);
    }

    Circle dummy;
    dummy.radius = 10.0f;
    dummy.color.setRgb(255, 0, 0);
    dummy.solid = true;
    dummy.center.setX(250);
    dummy.center.setY(300);
    _circles.push_back(dummy);
    // next let's draw all circles
    for (const Circle& circle : _circles) {
        logger->logDebug("Drawing a circle", "[sim_env::viewer::Box2DDrawingInterface::paintEvent]");
        pen.setColor(circle.color);
        brush.setColor(circle.color);
        if (circle.solid) {
            brush.setStyle(Qt::BrushStyle::SolidPattern);
        } else {
            brush.setStyle(Qt::BrushStyle::NoBrush);
        }
        painter.setPen(pen);
        painter.setBrush(brush);
        painter.drawEllipse(circle.center,(int)circle.radius, (int)circle.radius);
    }

    // finally all lines
    for (const LineSegment& line : _lines) {
        logger->logDebug("Drawing a line", "[sim_env::viewer::Box2DDrawingInterface::paintEvent]");
        pen.setColor(line.color);
        brush.setColor(line.color);
        painter.setPen(pen);
        painter.drawLine(line.point_a, line.point_b);
    }
    _polygons.clear();
    _lines.clear();
    _circles.clear();
}

void sim_env::viewer::Box2DDrawingInterface::wheelEvent(QWheelEvent *event) {
    _zoom_level += 0.02f * event->delta();
    _zoom_level = std::max(0.001f, _zoom_level);
    update();
}

void sim_env::viewer::Box2DDrawingInterface::setScale(float scale) {
    _scale = scale;
    assert(_scale != 0.0f);
}

QPoint sim_env::viewer::Box2DDrawingInterface::toScreenPoint(const b2Vec2 &vec) const {
    return QPoint(int(toScreenLength(vec.x)), int(toScreenLength(vec.y)));
}

float sim_env::viewer::Box2DDrawingInterface::toScreenLength(float length) const {
    return length * 1.0f / _scale;
}

float sim_env::viewer::Box2DDrawingInterface::toWorldLength(float length) const {
    return length * _scale;
}
void sim_env::viewer::Box2DDrawingInterface::createPolygon(const b2Vec2 *vertices,
                                                           int32 vertex_count, const b2Color &color,
                                                           sim_env::viewer::Polygon &output) const {
    for (int32 v = 0; v < vertex_count; ++v) {
        output.vertices.push_back(toScreenPoint(vertices[v]));
    }
    output.color.setRgb(int(color.r * 255), int(color.g * 255), int(color.b * 255), int(color.a * 255));
}

void sim_env::viewer::Box2DDrawingInterface::createCircle(const b2Vec2 &center, float32 radius, const b2Color color,
                                                          sim_env::viewer::Circle &circle) const {
    circle.color.setRgb(int(color.r * 255), int(color.g * 255), int(color.b * 255), int(color.a * 255));
    circle.center = toScreenPoint(center);
    circle.radius = toScreenLength(radius);
}


//////////////////////////////////////// Box2DWorldViewer ////////////////////////////////////////
sim_env::Box2DWorldViewer::Box2DWorldViewer(sim_env::Box2DWorldPtr world) {
    _world = std::weak_ptr<sim_env::Box2DWorld>(world);
}

sim_env::Box2DWorldViewer::~Box2DWorldViewer() {

}

void sim_env::Box2DWorldViewer::show(int argc, char **argv) {
    _app = std::unique_ptr<QApplication>(new QApplication(argc, argv));
    viewer::Box2DDrawingInterfacePtr render_area = std::make_shared<viewer::Box2DDrawingInterface>(_world.lock(),
        750, 500);
    _widgets.push_back(render_area);
    render_area->show();
}

int sim_env::Box2DWorldViewer::run() {
    if (!_app) {
        throw std::logic_error("[Box2DWorldViewer::run] Called run before setting up the application.");
    }
    log("Starting QApplication::exec", "[sim_env::Box2DWorldViewer]", Logger::LogLevel::Info);
    return _app->exec();
}

void sim_env::Box2DWorldViewer::drawFrame(const Eigen::Vector3f &transform) {

}

void sim_env::Box2DWorldViewer::log(const std::string &msg, const std::string& prefix,
                                    Logger::LogLevel level) const {
    if (_world.expired()) {
        auto logger = DefaultLogger::getInstance();
        logger->log(msg, level, prefix);
    }
    auto locked_world = _world.lock();
    locked_world->getLogger()->logInfo(msg, prefix);
}
