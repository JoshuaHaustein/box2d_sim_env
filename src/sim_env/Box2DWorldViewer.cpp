//
// Created by joshua on 6/26/17.
//

#include <sim_env/Box2DWorldViewer.h>
#include <QtGui/QPushButton>
#include <QtGui/QPainter>
#include <QWheelEvent>
#include <memory>

//////////////////////////////////////// Box2DObjectView ////////////////////////////////////////
sim_env::viewer::Box2DObjectView::Box2DObjectView(sim_env::Box2DObjectConstPtr object) {
    _object = object;
    std::vector<LinkConstPtr> links;
    object->getLinks(links);
    for (auto& link : links){
        Box2DLinkConstPtr box2d_link = std::static_pointer_cast<const Box2DLink>(link);
        Box2DLinkView* link_view = new Box2DLinkView(box2d_link, this);
        if (object->isStatic()) {
            link_view->setColors(QColor(255, 20, 20), QColor(0,0,0));
        } else {
            link_view->setColors(QColor(20, 20, 255), QColor(0,0,0));
        }
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
    _border_color = QColor(0,0,0);
    _fill_color = QColor(140, 140, 140);
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

void sim_env::viewer::Box2DLinkView::setColors(const QColor& fill_color, const QColor& border_color) {
    _border_color = border_color;
    _fill_color = fill_color;
}

QRectF sim_env::viewer::Box2DLinkView::boundingRect() const {
    return _bounding_rect;
}

void sim_env::viewer::Box2DLinkView::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("painting", "[sim_env::viewer::Box2DLinkView::paint]");
    const QBrush original_brush = painter->brush();
    const QPen original_pen = painter->pen();
    QBrush my_brush;
    my_brush.setColor(_fill_color);
    my_brush.setStyle(Qt::BrushStyle::SolidPattern);
    QPen my_pen;
    my_pen.setColor(_border_color);
    my_pen.setStyle(Qt::PenStyle::SolidLine);
    painter->setBrush(my_brush);
    painter->setPen(my_pen);
    for (auto& polygon : _polygons) {
        painter->drawPolygon(polygon);
    }
    painter->setBrush(original_brush);
    painter->setPen(original_pen);
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
    setRenderHint(QPainter::RenderHint::Antialiasing, true);
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
    if (factor < 0.7 || factor > 200) {
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

