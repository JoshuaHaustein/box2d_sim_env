//
// Created by joshua on 6/26/17.
//

#include <Box2D/Common/b2Math.h>
#include <Box2D/Box2D.h>
#include <sim_env/Box2DWorldViewer.h>
#include <QtGui/QPushButton>
#include <QtGui/QPainter>
#include <QWheelEvent>
#include <memory>

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

//////////////////////////////////////// Box2DObjectView ////////////////////////////////////////
sim_env::viewer::Box2DObjectView::Box2DObjectView(sim_env::Box2DObjectConstPtr object) {
    _object = object;
    std::vector<LinkConstPtr> links;
    object->getLinks(links);
    for (auto& link : links){
        Box2DLinkConstPtr box2d_link = std::static_pointer_cast<const Box2DLink>(link);
        Box2DLinkView* link_view = new Box2DLinkView(box2d_link, this);
    }
}

sim_env::viewer::Box2DObjectView::~Box2DObjectView() {
}

QRectF sim_env::viewer::Box2DObjectView::boundingRect() const {
    return this->childrenBoundingRect();
}

void sim_env::viewer::Box2DObjectView::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                                             QWidget *widget) {
    if (_object.expired()) {
        auto logger = DefaultLogger::getInstance();
        logger->logErr("The object to visualize is not available anymore.", "[sim_env::viewer::Box2DObjectView::paint]");
        return;
    }
    Box2DObjectConstPtr object = _object.lock();
    Eigen::Affine3f object_transform = object->getTransform();
    QTransform my_transform(object_transform(0, 0), object_transform(0, 1),
                            object_transform(1, 0), object_transform(1, 1),
                            object_transform(0, 3), object_transform(1, 3));
    setTransform(my_transform);
}

//////////////////////////////////////// Box2DRobotView ////////////////////////////////////////
sim_env::viewer::Box2DRobotView::Box2DRobotView(sim_env::Box2DRobotConstPtr robot) {
    _robot = robot;
    std::vector<LinkConstPtr> links;
    robot->getLinks(links);
    for (auto& link : links) {
        Box2DLinkConstPtr box2d_link = std::static_pointer_cast<const Box2DLink>(link);
        Box2DLinkView* link_view = new Box2DLinkView(box2d_link, this);
    }
}

sim_env::viewer::Box2DRobotView::~Box2DRobotView() {

}

QRectF sim_env::viewer::Box2DRobotView::boundingRect() const {
    return this->childrenBoundingRect();
}

void sim_env::viewer::Box2DRobotView::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                                            QWidget *widget) {
    if (_robot.expired()) {
        auto logger = DefaultLogger::getInstance();
        logger->logErr("The robot to visualize is not available anymore.", "[sim_env::viewer::Box2DRobotView::paint]");
        return;
    }
    Box2DRobotConstPtr robot = _robot.lock();
    Eigen::Affine3f robot_transform = robot->getTransform();
    QTransform my_transform(robot_transform(0, 0), robot_transform(0, 1),
                            robot_transform(1, 0), robot_transform(1, 1),
                            robot_transform(0, 3), robot_transform(1, 3));
    setTransform(my_transform);
}

//////////////////////////////////////// Box2DLinkView ////////////////////////////////////////
sim_env::viewer::Box2DLinkView::Box2DLinkView(sim_env::Box2DLinkConstPtr link,
                                              QGraphicsItem* parent):QGraphicsItem(parent) {
    _link = link;
    Box2DWorldPtr world = link->getBox2DWorld();
    float scaling_factor = world->getInverseScale();

    std::vector<std::vector<Eigen::Vector2f> > geometry;
    link->getGeometry(geometry);
    for (auto& polygon : geometry) {
        QPolygonF qt_polygon;
        for (auto& point : polygon) {
            qt_polygon.push_back(QPointF(scaling_factor * point[0], scaling_factor * point[1]));
        }
        _polygons.push_back(qt_polygon);
        _bounding_rect |= qt_polygon.boundingRect();
    }

}

QRectF sim_env::viewer::Box2DLinkView::boundingRect() const {
    return _bounding_rect;
}

void sim_env::viewer::Box2DLinkView::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    auto logger = DefaultLogger::getInstance();
    logger->logWarn("painting", "[sim_env::viewer::Box2DLinkView::paint]");
    for (auto& polygon : _polygons) {
        painter->drawPolygon(polygon);
    }
}

//////////////////////////////////////// Box2DJointView ////////////////////////////////////////
sim_env::viewer::Box2DJointView::Box2DJointView(sim_env::Box2DJointConstPtr joint, QGraphicsScene* scene,
                                                QGraphicsItem* parent):QGraphicsItem(parent) {

}

//sim_env::viewer::Box2DJointView::~Box2DJointView() {
//
//}

//////////////////////////////////////// Box2DWorldView ////////////////////////////////////////
sim_env::viewer::Box2DWorldView::Box2DWorldView(int width, int height, QWidget *parent):QGraphicsView(parent) {
    _scene = new QGraphicsScene();
    _width = width;
    _height = height;
    setScene(_scene);
}

sim_env::viewer::Box2DWorldView::~Box2DWorldView() {
}

void sim_env::viewer::Box2DWorldView::setBox2DWorld(sim_env::Box2DWorldConstPtr world) {
    _world = world;
    repopulate();
}

void sim_env::viewer::Box2DWorldView::repopulate() {
    if (_world.expired()) {
        auto logger = DefaultLogger::getInstance();
        logger->logWarn("Could not repopulate Box2DWorldView. Box2DWorld is missing.",
                        "[sim_env::viewer::Box2DWorldView]");
        return;
    }
    Box2DWorldConstPtr world = _world.lock();
    // Create object views
    std::vector<ObjectPtr> objects;
    world->getObjects(objects, true);
    for (auto& object : objects) {
        Box2DObjectPtr box2d_object = std::static_pointer_cast<Box2DObject>(object);
        Box2DObjectView* obj_view = new Box2DObjectView(box2d_object);
        _scene->addItem(obj_view);
        _object_views.push_back(obj_view);
    }
    // Create robot views
    std::vector<RobotPtr> robots;
    world->getRobots(robots);
    for (auto& robot : robots) {
        Box2DRobotPtr box2d_robot = std::static_pointer_cast<Box2DRobot>(robot);
        Box2DRobotView* robot_view = new Box2DRobotView(box2d_robot);
        _scene->addItem(robot_view);
        _robot_views.push_back(robot_view);
    }
}

void sim_env::viewer::Box2DWorldView::wheelEvent(QWheelEvent *event) {
    scaleView(pow(2.0, -event->delta() / 240.0));
}

void sim_env::viewer::Box2DWorldView::scaleView(double scale_factor) {
    // this is from a qt example
    qreal factor = transform().scale(scale_factor, scale_factor).mapRect(QRectF(0, 0, 1, 1)).width();
    // TODO this is the width of a unit cube when zoomed. See whether these numbers should be dependent on sth
    if (factor < 0.07 || factor > 100) {
        return;
    }
    scale(scale_factor, scale_factor);
}

//////////////////////////////////////// Box2DWorldViewer ////////////////////////////////////////
sim_env::Box2DWorldViewer::Box2DWorldViewer(sim_env::Box2DWorldPtr world) {
    _world = std::weak_ptr<sim_env::Box2DWorld>(world);
}

sim_env::Box2DWorldViewer::~Box2DWorldViewer() {

}

void sim_env::Box2DWorldViewer::show(int argc, char **argv) {
    _app = std::unique_ptr<QApplication>(new QApplication(argc, argv));
    _world_view.reset(new viewer::Box2DWorldView(500, 500));
    if (!_world.expired()) {
        _world_view->setBox2DWorld(_world.lock());
    } else {
        // TODO error message
    }
    _world_view->show();
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

