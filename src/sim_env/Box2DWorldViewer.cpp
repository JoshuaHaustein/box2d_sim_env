//
// Created by joshua on 6/26/17.
//

// out includes
#include <sim_env/Box2DWorldViewer.h>
// stl includes
#include <cstring>
#include <random>
#include <chrono>
// QT includes
#include <QtGui/QVBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include <QWheelEvent>
#include <QTimer>
#include <QtGui/QTabWidget>

//////////////////////////////////////// Box2DObjectView ////////////////////////////////////////
sim_env::viewer::Box2DObjectView::Box2DObjectView(sim_env::Box2DObjectPtr object, sim_env::viewer::Box2DWorldView* world_view) {
    _object = object;
    _world_view = world_view;
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
    // set object transform so that child links are rendered correctly
    Box2DObjectConstPtr object = _object.lock();
    Eigen::Affine3f object_transform = object->getTransform();
    // Qt uses transposed matrices
    QTransform my_transform(object_transform(0, 0), object_transform(1, 0),
                            object_transform(0, 1), object_transform(1, 1),
                            object_transform(0, 3), object_transform(1, 3));
    setTransform(my_transform);
}

void sim_env::viewer::Box2DObjectView::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    auto logger = DefaultLogger::getInstance();
    std::string prefix("[sim_env::viewer::Box2DObject::mousePressEvent]");
    logger->logDebug("Object selected", prefix);
    _world_view->setSelectedObject(_object);
}

//////////////////////////////////////// Box2DRobotView ////////////////////////////////////////
sim_env::viewer::Box2DRobotView::Box2DRobotView(sim_env::Box2DRobotPtr robot, Box2DWorldView* world_view) {
    _robot = robot;
    _world_view = world_view;
    std::vector<LinkConstPtr> links;
    robot->getLinks(links);
    for (auto& link : links) {
        Box2DLinkConstPtr box2d_link = std::static_pointer_cast<const Box2DLink>(link);
        Box2DLinkView* link_view = new Box2DLinkView(box2d_link, this);
    }
}

sim_env::viewer::Box2DRobotView::~Box2DRobotView() {
}

void sim_env::viewer::Box2DRobotView::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    auto logger = DefaultLogger::getInstance();
    std::string prefix("[sim_env::viewer::Box2DRobotView::mousePressEvent]");
    logger->logDebug("Mouse pressed on robot.", prefix);
    _world_view->setSelectedObject(_robot);
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
    // set the robot transform
    Box2DRobotConstPtr robot = _robot.lock();
    Eigen::Affine3f robot_transform = robot->getTransform();
    // Qt uses transposed matrices, hence transpose 2x2 rotation block
    QTransform my_transform(robot_transform(0, 0), robot_transform(1, 0),
                            robot_transform(0, 1), robot_transform(1, 1),
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
    // first set the transform of the link
    Box2DLinkConstPtr link = _link.lock();
    ObjectPtr object = link->getObject();
    auto logger = DefaultLogger::getInstance();
//    logger->logDebug("drawing link");
    if (not object) {
        std::stringstream ss;
        ss << "link " << link->getName() << " did not return a valid parent object";
        logger->logErr(ss.str(), "[sim_env::viewer::Box2DLinkView::paint]");
    }
    // compute and set transformation relative to object frame
    Eigen::Affine3f world_object_transform = object->getTransform();
    Eigen::Affine3f world_link_transform = link->getTransform();
    Eigen::Matrix4f relative_transform = world_object_transform.inverse().matrix() * world_link_transform.matrix();
    // Qt uses transposed matrices!!!! So we need to transpose the 2x2 rotation block
    QTransform my_transform(relative_transform(0, 0), relative_transform(1, 0),
                            relative_transform(0, 1), relative_transform(1, 1),
                            relative_transform(0, 3), relative_transform(1, 3));
    setTransform(my_transform);
    // now draw
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

//////////////////////////////////////// Box2DFrameView ////////////////////////////////////////
sim_env::viewer::Box2DFrameView::Box2DFrameView(const Eigen::Affine3f& frame,
                                                float length,
                                                float width,
                                                QGraphicsItem* parent):QGraphicsItem(parent) {
    Eigen::Matrix3f rotation_matrix = frame.rotation();
    auto translation = frame.translation();
    setTransform(QTransform(rotation_matrix(0, 0), rotation_matrix(1, 0),
                            rotation_matrix(0, 1), rotation_matrix(1, 1),
                            translation(0), translation(1)));
    _x_axis = QLineF(QPointF(0.0f, 0.0f), QPointF(length, 0.0f));
    _y_axis = QLineF(QPointF(0.0f, 0.0f), QPointF(0.0f, length));
    _width = width;
}

QRectF sim_env::viewer::Box2DFrameView::boundingRect() const {
    return QRectF(0, 0, 1, 1);
}

void sim_env::viewer::Box2DFrameView::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget *widget) {
    const QPen original_pen = painter->pen();
    QPen my_pen;
    my_pen.setColor(QColor(255, 0, 0));
    my_pen.setWidthF(_width);
    my_pen.setStyle(Qt::PenStyle::SolidLine);
    painter->setPen(my_pen);
    painter->drawLine(_x_axis);
    my_pen.setColor(QColor(0, 255, 0));
    painter->setPen(my_pen);
    painter->drawLine(_y_axis);
    painter->setPen(original_pen);
}

//////////////////////////////////////// Box2DObjectStateView ////////////////////////////////////////
sim_env::viewer::Box2DObjectStateView::Box2DObjectStateView(QWidget *parent):QGroupBox(parent) {
    setTitle("Selected object state");
    _form_layout = new QFormLayout();
    setLayout(_form_layout);
}

sim_env::viewer::Box2DObjectStateView::~Box2DObjectStateView() {
}

void sim_env::viewer::Box2DObjectStateView::synchView() {
    if (_current_object.expired()) {
        auto logger = DefaultLogger::getInstance();
        logger->logWarn("[sim_env::viewer::Box2DObjectStateView::synchView] Could not synchronize view "
                                "as there is no object information available.");
        return;
    }
    ObjectPtr object = _current_object.lock();
    auto logger = object->getWorld()->getLogger();
    std::string prefix("[sim_env::viewer::Box2DObject::synchView]");
    logger->logDebug("Synchronizing view.", prefix);
    // show name in title bar
    QString title("State of object ");
    title.append(object->getName().c_str());
    setTitle(title);
    // ensure we have a line edit item for x,y,theta
    if (_object_pose_edits.size() != 3) {
        // create edit for x value
        QLineEdit* x_edit = new QLineEdit();
        x_edit->installEventFilter(this);
        _form_layout->addRow("x:", x_edit);
        _object_pose_edits.push_back(x_edit);
        // create edit for y value
        QLineEdit* y_edit = new QLineEdit();
        y_edit->installEventFilter(this);
        _form_layout->addRow("y:", y_edit);
        _object_pose_edits.push_back(y_edit);
        // create edit for theta value
        QLineEdit* theta_edit = new QLineEdit();
        theta_edit->installEventFilter(this);
        _form_layout->addRow("theta:", theta_edit);
        _object_pose_edits.push_back(theta_edit);
    }
    // enable them for non static objects
    for (auto& edit_item : _object_pose_edits) {
        edit_item->setEnabled(not object->isStatic());
    }
    // now ensure we have sufficient sliders for joint values
    int pose_dofs = object->isStatic() ? 0 : 3;
    int num_joints = (int)object->getNumDOFs() - pose_dofs;
    // add sliders if we need more
    for (int i = (int)_joint_position_sliders.size(); i < num_joints; ++i) {
        QSlider* slider = new QSlider(Qt::Orientation::Horizontal);
        QString label("Joint %1");
        label = label.arg(i);
        slider->setTickInterval(100);
        _form_layout->addRow(label, slider);
        _joint_position_sliders.push_back(slider);
        QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderChange(int)));
    }
    // disable sliders that we do not need and ensure existing sliders are enabled
    for (int i = 0; i < (int)_joint_position_sliders.size(); ++i) {
        _joint_position_sliders.at((size_t)i)->setEnabled(i < num_joints);
    }
    logger->logDebug("View synchronized", prefix);
    // setLayout(_form_layout);
}

void sim_env::viewer::Box2DObjectStateView::showValues() {
    if (_current_object.expired()) {
        auto logger = DefaultLogger::getInstance();
        logger->logWarn("[sim_env::viewer::Box2DObjectStateView::showValues] Could not synchronize view "
                                "as there is no object information available.");
        return;
    }
    ObjectPtr object = _current_object.lock();
    Eigen::VectorXi dof_indices = object->getDOFIndices();
    Eigen::VectorXf configuration = object->getDOFPositions(dof_indices);
    Eigen::ArrayX2f limits = object->getDOFPositionLimits(dof_indices);
    unsigned int base_dof_offset = 0;
    // get pose of the object
    Eigen::Vector3f pose;
    Eigen::Affine3f tf = object->getTransform();
    pose[0] = tf.translation()(0);
    pose[1] = tf.translation()(1);
    pose[2] = std::acos(tf.rotation()(0, 0));
    pose[2] = tf.rotation()(1, 0) > 0.0 ? pose[2] : -pose[2];
    // write the pose into text fields
    for (int i = 0; i < pose.size(); ++i) {
        QString value("%L1");
        value = value.arg(pose[i]);
        _object_pose_edits.at(i)->setText(value);
    }
    // now read the rest of the configuration
    if (not object->isStatic()) {
        // if the object is not static, the pose is part of the configuration
        base_dof_offset = 3;
    }
    for (unsigned int i = base_dof_offset; i < configuration.size(); ++i) {
        int tick_value = toTickValue(configuration[i], limits(i, 0), limits(i, 1));
        QSlider* slider = _joint_position_sliders.at(i - base_dof_offset);
        slider->blockSignals(true);
        slider->setValue(tick_value);
        slider->blockSignals(false);
    }
}

void sim_env::viewer::Box2DObjectStateView::sliderChange(int value) {
    if (_current_object.expired()) {
        return;
    }
    ObjectPtr object = _current_object.lock();
    int dof_offset = object->isStatic() ? 0 : 3;
    for (size_t i = 0; i < _joint_position_sliders.size(); ++i) {
        if (_joint_position_sliders.at(i) == QObject::sender()) {
            Eigen::VectorXi indices(1);
            indices[0] = i + dof_offset;
            Eigen::ArrayX2f limits = object->getDOFPositionLimits();
            Eigen::VectorXf configuration(1);
            configuration[0] = fromTickValue(value, limits(indices[0], 0), limits(indices[0], 1));
            object->setDOFPositions(configuration, indices);
            emit valuesChanged();
            return;
        }
    }
}

bool sim_env::viewer::Box2DObjectStateView::eventFilter(QObject* qobject, QEvent* event) {
    // we are only interested in key-press and focus-out events
    if (event->type() != QEvent::FocusOut and event->type() != QEvent::KeyPress) {
        return false;
    } else if (event->type() == QEvent::KeyPress) {
        QKeyEvent *key_event = static_cast<QKeyEvent *>(event);
        // we are only reacting when the key enter was pressed
        if (key_event->key() != Qt::Key_Enter) {
            return false;
        }
    }
    // we do not need to do anything, if we do not have an object
    if (_current_object.expired()) {
        return false;
    }
    // now we are sure, we have a valid event and an object, so check which text box is the source
    for (size_t i = 0; i < _object_pose_edits.size(); ++i) {
        if (_object_pose_edits.at(i) == qobject) { // check whether text box i is the source of the event
            // if so set the respective coordinate
            ObjectPtr object = _current_object.lock();
            Eigen::Affine3f tf = object->getTransform();
            bool conversion_ok = false;
            float value = _object_pose_edits.at(i)->text().toFloat(&conversion_ok);
            if (conversion_ok) {
                if (i == 0 or i == 1) { // x or y change
                    tf.translation()(i) = value;
                    object->setTransform(tf);
                } else { // theta change
                    assert(i == 2);
                    float x = tf.translation()(0);
                    float y = tf.translation()(1);
                    tf = Eigen::Translation3f(x, y, 0.0);
                    tf.rotate(Eigen::AngleAxisf(value, Eigen::Vector3f::UnitZ()));
                    object->setTransform(tf);
                }
            } else {
                auto logger = object->getWorld()->getLogger();
                logger->logErr("Could not parse floating number value provided by user.",
                               "[sim_env::viewer::Box2DObjectStateView::textChange]");
            }
            return false; // there is always just one source, so we can safely return
        }
    }
    return false; // we didn't handle this event at all, it's not our problem
}

void sim_env::viewer::Box2DObjectStateView::setCurrentObject(sim_env::ObjectWeakPtr object) {
    _current_object = object;
    synchView();
    showValues();
}

void sim_env::viewer::Box2DObjectStateView::setObjectState() {
    // TODO
}

int sim_env::viewer::Box2DObjectStateView::toTickValue(float value, float min, float max) {
    return (int)(std::floor((value - min) / (max - min) * 100.0f));
}

float sim_env::viewer::Box2DObjectStateView::fromTickValue(int tick, float min, float max) {
    return (float)(tick) / 100.0f * (max - min) + min;
}

//////////////////////////////////////// Box2DWorldView ////////////////////////////////////////
sim_env::viewer::Box2DWorldView::Box2DWorldView(int width, int height, QWidget *parent):QGraphicsView(parent) {
    _scene = new QGraphicsScene();
    _refresh_timer = nullptr;
    _width = width;
    _height = height;
    setScene(_scene);
    setRenderHint(QPainter::RenderHint::Antialiasing, true);
    // qt has it's y axis pointing downwards, so let's revert that axis
    setTransform(QTransform(1, 0, 0, 0, -1, 0, 0, 0, 1));
//    setTransformationAnchor(QGraphicsView::ViewportAnchor::AnchorUnderMouse);
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
        Box2DObjectView* obj_view = new Box2DObjectView(box2d_object, this);
        _scene->addItem(obj_view);
        _object_views.push_back(obj_view);
    }
    // Create robot views
    std::vector<RobotPtr> robots;
    world->getRobots(robots);
    for (auto& robot : robots) {
        Box2DRobotPtr box2d_robot = std::static_pointer_cast<Box2DRobot>(robot);
        Box2DRobotView* robot_view = new Box2DRobotView(box2d_robot, this);
        _scene->addItem(robot_view);
        _robot_views.push_back(robot_view);
    }
    if (not _refresh_timer) {
        _refresh_timer = new QTimer(this);
    }
    // TODO we probably don't need this. Instead only force redraws during propagation demo
    // TODO and when modifications on the model occurred.
    connect(_refresh_timer, SIGNAL(timeout()), this, SLOT(refreshView()));
    _refresh_timer->setSingleShot(false);
    _refresh_timer->start(40); // 25Hz
}

void sim_env::viewer::Box2DWorldView::drawFrame(const Eigen::Affine3f& frame, float length, float width) {
    _scene->addItem(new Box2DFrameView(frame, length, width));
}

QSize sim_env::viewer::Box2DWorldView::sizeHint() const {
    return QSize(_width, _height);
}

void sim_env::viewer::Box2DWorldView::wheelEvent(QWheelEvent *event) {
    scaleView(pow(2.0, -event->delta() / 240.0));
}

void sim_env::viewer::Box2DWorldView::refreshView() {
//    auto logger = DefaultLogger::getInstance();
//    logger->logDebug("REFRESHING WORLD VIEW");
    _scene->update();
    update();
}

void sim_env::viewer::Box2DWorldView::setSelectedObject(sim_env::ObjectWeakPtr object) {
  _currently_selected_object = object;
  emit objectSelected(_currently_selected_object);
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
/////////////////////////////////// Box2DSimulationController ///////////////////////////////////
sim_env::viewer::Box2DSimulationController::Box2DSimulationController(Box2DWorldPtr world) {
    _simulation_thread.is_running = false;
    _simulation_thread.world = world;
}

sim_env::viewer::Box2DSimulationController::~Box2DSimulationController() {
    terminateThread();
}

void sim_env::viewer::Box2DSimulationController::terminateThread() {
    _simulation_thread.is_running = false;
    _simulation_thread.simulation_thread.join();
}

void sim_env::viewer::Box2DSimulationController::startSimulation() {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("starting simulation");
    if (_simulation_thread.is_running) {
        return;
    }
    _simulation_thread.is_running = true;
    _simulation_thread.simulation_thread = std::thread(&SimulationThread::run, std::ref(_simulation_thread));
}

void sim_env::viewer::Box2DSimulationController::stopSimulation() {
    auto logger = DefaultLogger::getInstance();
    logger->logDebug("stopping simulation");
    terminateThread();
}

void sim_env::viewer::Box2DSimulationController::triggerSimulation(bool run) {
    if (run) {
        startSimulation();
    } else {
        stopSimulation();
    }
}

void sim_env::viewer::Box2DSimulationController::SimulationThread::run() {
    while (is_running) {
        if (world.expired()) {
            is_running = false;
            continue;
        }
        Box2DWorldPtr accessible_world = world.lock();
        accessible_world->stepPhysics(1);
        float time_step = accessible_world->getPhysicsTimeStep();
        std::this_thread::sleep_for(std::chrono::duration<float>(time_step));
    }
}

//////////////////////////////////////// Box2DWorldViewer ////////////////////////////////////////
sim_env::Box2DWorldViewer::Box2DWorldViewer(sim_env::Box2DWorldPtr world) {
    _world = std::weak_ptr<sim_env::Box2DWorld>(world);
    _argv = nullptr;
    _argc = 0;
    _is_showing = false;
}

sim_env::Box2DWorldViewer::~Box2DWorldViewer() {
    deleteArgs();
}

void sim_env::Box2DWorldViewer::deleteArgs() {
    for (int i = 0; i < _argc; ++i) {
        delete[] _argv[i];
    }
    delete[] _argv;
    _argv = nullptr;
    _argc = 0;
}

void sim_env::Box2DWorldViewer::show(int argc, char **argv) {
    if (_is_showing) {
        return;
    }
    // Qt requires the parameters argc and argv to exist as long as the application exists
    // so let's copy them
    _app.reset(nullptr);
    deleteArgs();
    _argc = argc;
    _argv = new char*[argc];
    for (int i = 0; i < argc; ++i) {
        size_t length = std::strlen(argv[i]);
        _argv[i] = new char[length];
        std::strcpy(_argv[i], argv[i]);
    }
    _app = std::unique_ptr<QApplication>(new QApplication(_argc, _argv));
    createUI();
    _root_widget->show();
    _is_showing = true;
}

int sim_env::Box2DWorldViewer::run() {
    if (!_app) {
        throw std::logic_error("[Box2DWorldViewer::run] Called run before setting up the application.");
    }
    log("Starting QApplication::exec", "[sim_env::Box2DWorldViewer]", Logger::LogLevel::Info);
    return _app->exec();
}

void sim_env::Box2DWorldViewer::drawFrame(const Eigen::Affine3f &transform, float length, float width) {
    _world_view->drawFrame(transform, length, width);
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

void sim_env::Box2DWorldViewer::createUI() {
    // We are using raw pointers here, but Qt takes care of deleting these
    _root_widget.reset(new QWidget());
    QVBoxLayout* root_layout = new QVBoxLayout();
    // on the root we have a container (with more stuff) and the bottom bar
    QWidget* container = new QWidget();
    QHBoxLayout* container_layout = new QHBoxLayout();
    // the container shall contain our world view
    _world_view = new viewer::Box2DWorldView(500, 500);
    if (!_world.expired()) {
        _world_view->setBox2DWorld(_world.lock());
        _simulation_controller = std::unique_ptr<viewer::Box2DSimulationController>(new viewer::Box2DSimulationController(_world.lock()));
    } else {
        throw std::logic_error("[sim_env::Box2DWorldViewer::createUI] Attempted to create a view on a non-existant Box2D world");
    }
    // create the side group (for displaying state information)
    container_layout->addWidget(_world_view);
    QWidget* side_panel = createSideBar();
    container_layout->addWidget(side_panel);
    container->setLayout(container_layout);

    // bottom widget (simulation control)
    QWidget* bottom_panel = createBottomBar();
    root_layout->addWidget(container);
    root_layout->addWidget(bottom_panel);
    _root_widget->setLayout(root_layout);
}

QWidget* sim_env::Box2DWorldViewer::createBottomBar() {
    QTabWidget* tab_widget = new QTabWidget(_root_widget.get());
    QGroupBox* bottom_group = new QGroupBox("Dynamics model control");
    tab_widget->addTab(bottom_group, "Simulation control");
    QHBoxLayout* bottom_group_layout = new QHBoxLayout();
    QPushButton* control_button = new QPushButton("Run simulation", bottom_group);
    control_button->setCheckable(true);
    bottom_group_layout->addWidget(control_button);
    bottom_group->setLayout(bottom_group_layout);
    QObject::connect(control_button, SIGNAL(clicked(bool)), _simulation_controller.get(), SLOT(triggerSimulation(bool)));
    return tab_widget;
}

QWidget* sim_env::Box2DWorldViewer::createSideBar() {
    // TODO Unfortunately this doesn't work with a scroll area (the layout can not be changed on the fly)
    // QScrollArea* scroll_view = new QScrollArea();
    viewer::Box2DObjectStateView* state_view = new viewer::Box2DObjectStateView();
    QObject::connect(_world_view, SIGNAL(objectSelected(sim_env::ObjectWeakPtr)),
            state_view, SLOT(setCurrentObject(sim_env::ObjectWeakPtr)));
    QObject::connect(state_view, SIGNAL(valuesChanged()), _world_view,
              SLOT(refreshView()));
    return state_view;
}
