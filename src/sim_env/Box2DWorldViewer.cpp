//
// Created by joshua on 6/26/17.
//

// our includes
#include <sim_env/Box2DWorldViewer.h>
#include <sim_env/Box2DController.h>
// stl includes
#include <cstring>
#include <random>
#include <chrono>
#include <memory>
// QT includes
#include <QtGui/QVBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include <QWheelEvent>
#include <QTimer>
#include <QtGui/QRadioButton>
#include <QtCore/QSize>

//////////////////////////////////////// utils /////////////////////////////////////////////
int sim_env::viewer::utils::toTickValue(float value, float min, float max) {
    return (int)(std::floor((value - min) / (max - min) * 100.0f));
}

float sim_env::viewer::utils::fromTickValue(int tick, float min, float max) {
    return (float)(tick) / 100.0f * (max - min) + min;
}

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
        // we can safely ignore the warning of link_view being unused here
        // the link view is automatically added as child to this view
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

//////////////////////////////////////// LineEditChangeDetector ////////////////////////////////////////
sim_env::viewer::LineEditChangeDetector::LineEditChangeDetector(QObject *parent):QObject(parent) {
}

sim_env::viewer::LineEditChangeDetector::~LineEditChangeDetector() {
}

bool sim_env::viewer::LineEditChangeDetector::eventFilter(QObject *qobject, QEvent *event) {
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
    QLineEdit* line_edit = dynamic_cast<QLineEdit*>(qobject);
    if (line_edit) {
        // notify all listeners
        emit(valueChanged(line_edit));
    }
    return false;
}
//////////////////////////////////////// Box2DObjectStateView ////////////////////////////////////////
const std::string sim_env::viewer::Box2DObjectStateView::LINE_EDIT_TYPE_PROPERTY_KEY = "LINE_EDIT_TYPE_KEY";
const std::string sim_env::viewer::Box2DObjectStateView::LINE_EDIT_DOF_PROPERTY_KEY = "LINE_EDIT_DOF_KEY";

sim_env::viewer::Box2DObjectStateView::Box2DObjectStateView(QWidget *parent):QGroupBox(parent) {
    setTitle("Selected object state");
    _form_layout = new QFormLayout();
    _line_edit_change_detector = new LineEditChangeDetector(this);
    QObject::connect(_line_edit_change_detector, SIGNAL(valueChanged(QLineEdit*)),
                     this, SLOT(lineEditChange(QLineEdit*)));
    setLayout(_form_layout);
    _mode_button = new QPushButton("View");
    _mode_button->setCheckable(true);
    _form_layout->addRow("View Mode:", _mode_button);
    _collision_display = new QLabel("");
    _collision_display->setStyleSheet("QLabel {background-color: green;}");
    _form_layout->addRow("Colliding objects:", _collision_display);
    QObject::connect(_mode_button, SIGNAL(clicked(bool)), this, SLOT(setViewMode(bool)));
}

sim_env::viewer::Box2DObjectStateView::~Box2DObjectStateView() {
}


void sim_env::viewer::Box2DObjectStateView::setCurrentObject(sim_env::ObjectWeakPtr object) {
    _current_object = object;
    synchView();
    _mode_button->setChecked(false);
    setViewMode(false);
    showValues(true);
}

void sim_env::viewer::Box2DObjectStateView::sliderChange(int value) {
    if (_current_object.expired()) {
        return;
    }
    ObjectPtr object = _current_object.lock();
    for (size_t i = 0; i < _joint_position_sliders.size(); ++i) {
        if (_joint_position_sliders.at(i) == QObject::sender()) {
            Eigen::VectorXi indices(1);
            indices[0] = (int) (i + object->getNumBaseDOFs());
            Eigen::ArrayX2f limits = object->getDOFPositionLimits(indices);
            Eigen::VectorXf configuration(1);
            configuration[0] = utils::fromTickValue(value, limits(0, 0), limits(0, 1));
            auto logger = object->getWorld()->getLogger();
            std::stringstream ss;
            ss << "Current joint position: " << object->getDOFPositions(indices).transpose();
            ss << "New joint position: " << configuration.transpose();
            logger->logDebug(ss.str());
            object->setDOFPositions(configuration, indices);
            emit newUserState();
            return;
        }
    }
}

void sim_env::viewer::Box2DObjectStateView::lineEditChange(QLineEdit *line_edit) {
    // we do not need to do anything, if we do not have an object
    ObjectPtr object = _current_object.lock();
    if (!object) {
        return;
    }
    // now we are sure we have a valid object, so check which text box is the source
    LoggerPtr logger = object->getWorld()->getLogger();
    bool conversion_ok = true;
    QVariant type_property = line_edit->property(LINE_EDIT_TYPE_PROPERTY_KEY.c_str());
    int type_property_int =  type_property.toInt(&conversion_ok);
    if (not conversion_ok
        or (type_property_int != LINE_EDIT_OBJECT_POSE_TYPE
            and type_property_int != LINE_EDIT_OBJECT_VEL_TYPE
            and type_property_int != LINE_EDIT_JOINT_VEL_TYPE)) {
        throw std::logic_error("[sim_env::viewer::Box2DObjectStateView::lineEditChange]"
                               " Could not determine type of line edit. This should not happen and indicates corruption of this widget.");
    }
    QVariant dof_property = line_edit->property(LINE_EDIT_DOF_PROPERTY_KEY.c_str());
    int dof_property_int = dof_property.toInt(&conversion_ok);
    if (not conversion_ok or dof_property_int < 0 or dof_property_int > object->getNumDOFs()) {
        throw std::logic_error("[sim_env::viewer::Box2DObjectStateView::lineEditChange]"
                               " Could not determine dof index of line edit. This indicates a serious corruption of this widget.");
    }
    // read the floating number value in line_edit
    float value = line_edit->text().toFloat(&conversion_ok);
    if (not conversion_ok) {
        LoggerPtr logger = object->getWorld()->getLogger();
        logger->logErr("Could not parse value. Invalid floating point number.",
                       "[sim_env::viewer::Box2DObjectStateView::lineEditChange]");
        return;
    }
    // now check the type of the line edit
    if (type_property_int == LINE_EDIT_OBJECT_POSE_TYPE) {
        // if it is a pose value set the respective coordinate
        Eigen::Affine3f tf = object->getTransform();
        if (dof_property_int == 0 or dof_property_int == 1) { // x or y change
            tf.translation()(dof_property_int) = value;
            object->setTransform(tf);
        } else { // theta change
            assert(dof_property_int == 2);
            float x = tf.translation()(0);
            float y = tf.translation()(1);
            tf = Eigen::Translation3f(x, y, 0.0);
            tf.rotate(Eigen::AngleAxisf(value, Eigen::Vector3f::UnitZ()));
            object->setTransform(tf);
        }
    } else {
        // if it is a velocity, set the respective velocity
        Eigen::VectorXi index(1);
        Eigen::VectorXf value_vector(1);
        index[0] = dof_property_int;
        value_vector[0] = value;
        object->setDOFVelocities(value_vector, index);
    }
    emit newUserState();
}

void sim_env::viewer::Box2DObjectStateView::setViewMode(bool enable_edit) {
    ObjectPtr object = _current_object.lock();
    if (not object) {
        auto logger = DefaultLogger::getInstance();
        logger->logWarn("[sim_env::viewer::Box2DObjectStateView::setViewMode] Could not set view mode"
                                "as there is no object information available.");
        return;
    }
    LoggerPtr logger = object->getWorld()->getLogger();
    for (auto& line_edit : _object_pose_edits) {
        line_edit->setEnabled(enable_edit);
    }
    for (auto& line_edit : _object_velocity_edits) {
        line_edit->setEnabled(enable_edit);
    }
    assert(_joint_position_sliders.size() >= object->getNumDOFs() - object->getNumBaseDOFs());
    assert(_joint_velocity_edits.size() >= object->getNumDOFs() - object->getNumBaseDOFs());
    for (unsigned int i = object->getNumBaseDOFs(); i < object->getNumDOFs(); ++i) {
        _joint_position_sliders.at(i - object->getNumBaseDOFs())->setEnabled(enable_edit);
        _joint_velocity_edits.at(i - object->getNumBaseDOFs())->setEnabled(enable_edit);
    }
    if (enable_edit){
        _mode_button->setText("Edit");
    } else {
        _mode_button->setText("View");
    }
}

void sim_env::viewer::Box2DObjectStateView::stateUpdate() {
    showValues(not _mode_button->isChecked());
}

void sim_env::viewer::Box2DObjectStateView::createBaseDOFEdits(ObjectPtr object) {
    assert (_object_velocity_edits.size() == _object_pose_edits.size());
    if (_object_pose_edits.size() != 3) {
        // create edit for x value
        QLineEdit* x_edit = new QLineEdit();
        x_edit->installEventFilter(_line_edit_change_detector);
        _form_layout->addRow("x:", x_edit);
        x_edit->setProperty(LINE_EDIT_TYPE_PROPERTY_KEY.c_str(), QVariant(LINE_EDIT_OBJECT_POSE_TYPE));
        x_edit->setProperty(LINE_EDIT_DOF_PROPERTY_KEY.c_str(), QVariant(0));
        _object_pose_edits.push_back(x_edit);
        // create edit for x velocity value
        QLineEdit* x_vel_edit = new QLineEdit();
        x_vel_edit->installEventFilter(_line_edit_change_detector);
        _form_layout->addRow("x_vel:", x_vel_edit);
        x_vel_edit->setProperty(LINE_EDIT_TYPE_PROPERTY_KEY.c_str(), QVariant(LINE_EDIT_OBJECT_VEL_TYPE));
        x_vel_edit->setProperty(LINE_EDIT_DOF_PROPERTY_KEY.c_str(), QVariant(0));
        _object_velocity_edits.push_back(x_vel_edit);
        // create edit for y value
        QLineEdit* y_edit = new QLineEdit();
        y_edit->installEventFilter(_line_edit_change_detector);
        _form_layout->addRow("y:", y_edit);
        y_edit->setProperty(LINE_EDIT_TYPE_PROPERTY_KEY.c_str(), QVariant(LINE_EDIT_OBJECT_POSE_TYPE));
        y_edit->setProperty(LINE_EDIT_DOF_PROPERTY_KEY.c_str(), QVariant(1));
        _object_pose_edits.push_back(y_edit);
        // create edit for y velocity value
        QLineEdit* y_vel_edit = new QLineEdit();
        y_vel_edit->installEventFilter(_line_edit_change_detector);
        _form_layout->addRow("y_vel:", y_vel_edit);
        y_vel_edit->setProperty(LINE_EDIT_TYPE_PROPERTY_KEY.c_str(), QVariant(LINE_EDIT_OBJECT_VEL_TYPE));
        y_vel_edit->setProperty(LINE_EDIT_DOF_PROPERTY_KEY.c_str(), QVariant(1));
        _object_velocity_edits.push_back(y_vel_edit);
        // create edit for theta value
        QLineEdit* theta_edit = new QLineEdit();
        theta_edit->installEventFilter(_line_edit_change_detector);
        _form_layout->addRow("theta:", theta_edit);
        theta_edit->setProperty(LINE_EDIT_TYPE_PROPERTY_KEY.c_str(), QVariant(LINE_EDIT_OBJECT_POSE_TYPE));
        theta_edit->setProperty(LINE_EDIT_DOF_PROPERTY_KEY.c_str(), QVariant(2));
        _object_pose_edits.push_back(theta_edit);
        // create edit for theta velocity value
        QLineEdit* theta_vel_edit = new QLineEdit();
        theta_vel_edit->installEventFilter(_line_edit_change_detector);
        _form_layout->addRow("theta_vel:", theta_vel_edit);
        theta_vel_edit->setProperty(LINE_EDIT_TYPE_PROPERTY_KEY.c_str(), QVariant(LINE_EDIT_OBJECT_VEL_TYPE));
        theta_vel_edit->setProperty(LINE_EDIT_DOF_PROPERTY_KEY.c_str(), QVariant(2));
        _object_velocity_edits.push_back(theta_vel_edit);
    }
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
    logger->logDebug("Synchronizing state view.", prefix);
    // show name in title bar
    QString title("State of object ");
    title.append(object->getName().c_str());
    setTitle(title);
    // ensure we have a line edit item for x,y,theta
    createBaseDOFEdits(object);
    // enable them
    for (auto& edit_item : _object_pose_edits) {
        edit_item->setEnabled(true);
    }
    // enable velocity only for non-static objects
    for (auto& edit_item : _object_velocity_edits) {
        edit_item->setEnabled(not object->isStatic());
    }
    // now ensure we have sufficient sliders for joint values
    int pose_dofs = object->getNumBaseDOFs();
    int num_joints = (int)object->getNumDOFs() - pose_dofs;
    // add sliders if we need more
    assert(_joint_velocity_edits.size() == _joint_position_sliders.size());
    for (int i = (int)_joint_position_sliders.size(); i < num_joints; ++i) {
        QSlider* slider = new QSlider(Qt::Orientation::Horizontal);
        QLineEdit* vel_edit = new QLineEdit();
        vel_edit->setProperty(LINE_EDIT_TYPE_PROPERTY_KEY.c_str(), QVariant(LINE_EDIT_JOINT_VEL_TYPE));
        vel_edit->setProperty(LINE_EDIT_DOF_PROPERTY_KEY.c_str(), QVariant(i + pose_dofs));
        QString label("Joint %1");
        QString vel_label("Joint %1 vel");
        label = label.arg(i);
        vel_label = vel_label.arg(i);
        slider->setTickInterval(100);
        _form_layout->addRow(label, slider);
        _form_layout->addRow(vel_label, vel_edit);
        _joint_position_sliders.push_back(slider);
        _joint_velocity_edits.push_back(vel_edit);
        QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderChange(int)));
        vel_edit->installEventFilter(_line_edit_change_detector);
    }
    // disable sliders that we do not need and ensure existing sliders are enabled
    for (int i = 0; i < (int)_joint_position_sliders.size(); ++i) {
        _joint_position_sliders.at((size_t)i)->setEnabled(i < num_joints);
        _joint_velocity_edits.at((size_t)i)->setEnabled(i < num_joints);
    }
    logger->logDebug("View synchronized", prefix);
}

void sim_env::viewer::Box2DObjectStateView::setSliderValue(QSlider* slider,
                                                           float value,
                                                           float min_value,
                                                           float max_value) {
    int tick_value = utils::toTickValue(value, min_value, max_value);
    slider->blockSignals(true);
    slider->setValue(tick_value);
    slider->blockSignals(false);
}

void sim_env::viewer::Box2DObjectStateView::showValues(bool update_input_fields) {
    if (_current_object.expired()) {
        auto logger = DefaultLogger::getInstance();
//        logger->logDebug("[sim_env::viewer::Box2DObjectStateView::showValues] Could not synchronize view "
//                                "as there is no object information available.");
        return;
    }
    ObjectPtr object = _current_object.lock();

    if (update_input_fields) {
        Eigen::VectorXi dof_indices = object->getDOFIndices();
        Eigen::VectorXf configuration = object->getDOFPositions(dof_indices);
        Eigen::VectorXf velocities = object->getDOFVelocities(dof_indices);
        Eigen::ArrayX2f position_limits = object->getDOFPositionLimits(dof_indices);
        Eigen::ArrayX2f velocity_limits = object->getDOFVelocityLimits(dof_indices);

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
            value.setNum(0.0);
            _object_velocity_edits.at(i)->setText(value);
        }
        // now read the rest of the configuration
        for (int i = 0; i < object->getNumBaseDOFs(); ++i) {
            // if the object is not static, the pose is part of the configuration
            // and we care for its velocity in these DOFs
            QString value("%L1");
            value = value.arg(velocities[i]);
            _object_velocity_edits.at(i)->setText(value);
        }

        for (unsigned int i = object->getNumBaseDOFs(); i < configuration.size(); ++i) {
            QSlider* pos_slider = _joint_position_sliders.at(i - object->getNumBaseDOFs());
            setSliderValue(pos_slider, configuration[i], position_limits(i, 0), position_limits(i, 1));
            QString value("%L1");
            QLineEdit* vel_edit = _joint_velocity_edits.at(i - object->getNumBaseDOFs());
            value = value.arg(velocities[i]);
            vel_edit->setText(value);
        }
    }
//    std::vector<sim_env::Contact> contacts;
//    if (object->checkCollision(contacts)) {
//        _collision_display->setStyleSheet("QLabel {background-color: red};");
//        QString text("Found %L1 contacts");
//        text = text.arg(contacts.size());
//        _collision_display->setText(text);
//    } else {
//        _collision_display->setStyleSheet("QLabel {background-color: green};");
//        _collision_display->setText("No collision");
//    }
}


//////////////////////////////////////// Box2DControllerView ////////////////////////////////////////

sim_env::viewer::Box2DControllerView::Box2DControllerView(QWidget *parent) : QGroupBox(parent) {
    initView();
}

sim_env::viewer::Box2DControllerView::~Box2DControllerView() {
}

void sim_env::viewer::Box2DControllerView::sliderChange(int value) {
    updateTarget();
}

void sim_env::viewer::Box2DControllerView::lineEditChange(QLineEdit *line_edit) {
    updateTarget();
}

void sim_env::viewer::Box2DControllerView::triggerController(bool enable) {
    setController();
}

void sim_env::viewer::Box2DControllerView::triggerControllerTypeChange(bool toggled) {
    updateView();
    setController();
}

void sim_env::viewer::Box2DControllerView::setCurrentObject(sim_env::ObjectWeakPtr object) {
    if (object.expired()) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Provided weak pointer is not referring to a valid object anymore.",
                       "[sim_env::viewer::Box2DControllerView::setSelectedObject]");
        return;
    }
    sim_env::ObjectPtr object_ptr = object.lock();
    if (object_ptr->getType() == sim_env::EntityType::Robot) {
        sim_env::RobotPtr robot = std::dynamic_pointer_cast<sim_env::Robot>(object_ptr);
        _current_robot = robot;
        // ensure we have a velocity controller for this robot
        auto iter_velocity = _velocity_controllers.find(robot->getName());
        if (iter_velocity == _velocity_controllers.end()) {
            sim_env::Box2DRobotPtr box2d_robot = std::dynamic_pointer_cast<Box2DRobot>(robot);
            _current_velocity_controller = std::make_shared<sim_env::Box2DRobotVelocityController>(box2d_robot);
            _velocity_controllers[robot->getName()] = _current_velocity_controller;
        } else {
            _current_velocity_controller = iter_velocity->second;
        }
        // ensure we have a position controller for this robot
        auto iter_postion = _position_controllers.find(robot->getName());
        if (iter_postion == _position_controllers.end()) {
            _current_position_controller = std::make_shared<sim_env::RobotPositionController>(robot, _current_velocity_controller);
            _position_controllers[robot->getName()] = _current_position_controller;
        } else {
            _current_position_controller = iter_postion->second;
        }
        updateView();
    }
}

void sim_env::viewer::Box2DControllerView::initView() {
    QGridLayout* grid_layout = new QGridLayout();
    setLayout(grid_layout);
    _position_button = new QRadioButton("Position control");
    grid_layout->addWidget(_position_button, 0, 0, 1, 2, Qt::AlignHCenter);
    QObject::connect(_position_button, SIGNAL(clicked(bool)), this, SLOT(triggerControllerTypeChange(bool)));
    _velocity_button = new QRadioButton("Velocity control");
    grid_layout->addWidget(_velocity_button, 1, 0, 1, 2, Qt::AlignHCenter);
    QObject::connect(_velocity_button, SIGNAL(clicked(bool)), this, SLOT(triggerControllerTypeChange(bool)));
    _enable_button = new QPushButton("Enable controller");
    _enable_button->setCheckable(true);
    grid_layout->addWidget(_enable_button, 0, 2, 2, 2);
    QObject::connect(_enable_button, SIGNAL(clicked(bool)), this, SLOT(triggerController(bool)));
    QLabel* target_label = new QLabel("Targets");
    grid_layout->addWidget(target_label, 2, 0);
    _x_edit = new QLineEdit();
    _y_edit = new QLineEdit();
    _theta_edit = new QLineEdit();
    grid_layout->addWidget(_x_edit, 2, 1);
    grid_layout->addWidget(_y_edit, 2, 2);
    grid_layout->addWidget(_theta_edit, 2, 3);
    setEnabled(false);
    _line_edit_change_detector = new LineEditChangeDetector(this);
    QObject::connect(_line_edit_change_detector, SIGNAL(valueChanged(QLineEdit*)),
                     this, SLOT(lineEditChange(QLineEdit*)));
    _x_edit->installEventFilter(_line_edit_change_detector);
    _y_edit->installEventFilter(_line_edit_change_detector);
    _theta_edit->installEventFilter(_line_edit_change_detector);
}

void sim_env::viewer::Box2DControllerView::updateView() {
    QLayout* layout = this->layout();
    QGridLayout* grid_layout = dynamic_cast<QGridLayout*>(layout);
    if (!grid_layout) {
        throw std::logic_error("[sim_env::viewer::Box2DControllerView] Unkown layout encountered. Should be QGridLayout");
    }
    if (_current_robot.expired()) {
        setEnabled(false);
        return;
    }
    setEnabled(true);
    if (not _position_button->isChecked() and not _velocity_button->isChecked()) {
        _position_button->setChecked(true);
    }

    sim_env::RobotPtr robot = _current_robot.lock();
    int dof_offest = 3;
    if (robot->isStatic()) {
        _x_edit->setEnabled(false);
        _y_edit->setEnabled(false);
        _theta_edit->setEnabled(false);
        dof_offest = 0;
    }
    Eigen::VectorXi active_dofs = robot->getActiveDOFs();
    // run over all dofs and synch view
    for (int idx = 0; idx < active_dofs.size(); ++idx) {
        int dof_idx = active_dofs[idx]; // the index of the actual DOF
        if (dof_idx < 3 and not robot->isStatic()) {
            // it's a base position DOF (x,y or theta)
            synchTextValue(dof_idx, robot);
        } else {
            // it's a joint DOF
            JointConstPtr joint = robot->getConstJointFromDOFIndex(dof_idx);
            int slider_idx = idx - dof_offest;
            QSlider* slider = nullptr;
            if (slider_idx >= _sliders.size()) {
                // add a new slider
                slider = new QSlider(Qt::Orientation::Horizontal);
                QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderChange(int)));
                slider->setTickInterval(100);
                _sliders.push_back(slider);
                grid_layout->addWidget(slider);
            } else {
                slider = _sliders.at(slider_idx);
            }
            synchSliderValue(slider, joint);
        }
    }
    // optionally deactivate unused sliders
    for (int slider_idx = (int) (active_dofs.size()) - dof_offest; slider_idx < (int)_sliders.size(); ++slider_idx) {
        _sliders.at(slider_idx)->setEnabled(false);
    }
}

void sim_env::viewer::Box2DControllerView::synchSliderValue(QSlider *slider, sim_env::JointConstPtr joint) {
    Eigen::Array2f limits;
    float value;
    if (_position_button->isChecked()) {
        // position control mode
        limits = joint->getPositionLimits();
        value = joint->getPosition();
    } else {
        limits = joint->getVelocityLimits();
        value = joint->getVelocity();
    }
    if (isinf(limits[0])) {
        limits[0] = -utils::LARGE_FLOATING_NUMBER;
    }
    if (isinf(limits[1])) {
        limits[1] = utils::LARGE_FLOATING_NUMBER;
    }
    int tick_value = utils::toTickValue(value, limits[0], limits[1]);
    slider->blockSignals(true);
    slider->setValue(tick_value);
    slider->blockSignals(false);
}

void sim_env::viewer::Box2DControllerView::synchTextValue(int dof_idx, sim_env::RobotPtr robot) {
    LoggerPtr logger = DefaultLogger::getInstance();
    float value;
    Eigen::VectorXi indices(1);
    indices[0] = dof_idx;
    if (_position_button->isChecked()) {
        // position mode
        Eigen::VectorXf config = robot->getDOFPositions(indices);
        value = config[0];
    } else {
        // velocity mode
        Eigen::VectorXf velo = robot->getDOFVelocities(indices);
        value = velo[0];
    }
    QString text("%L1");
    text = text.arg(value);
    switch(dof_idx) {
        case 0: {
            _x_edit->setText(text);
            break;
        }
        case 1: {
            _y_edit->setText(text);
            break;
        }
        case 2: {
            _theta_edit->setText(text);
            break;
        }
        default: {
            logger->logErr("Unknown dof index encountered.", "[sim_env::viewer::Box2DControllerView::synchTextValue]");
            break;
        }
    }
}

void sim_env::viewer::Box2DControllerView::setController() {
    if (_current_robot.expired()) {
        sim_env::LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Could not access robot - shared pointer expired.",
                       "[sim_env::viewer::Box2DControllerView::setController]");
        // TODO we need to delete controllers for this expired robot (would need some kind of delete listener)
        return;
    }
    sim_env::RobotPtr robot = _current_robot.lock();
    if (_enable_button->isChecked()) {
        using namespace std::placeholders;
        if (_position_button->isChecked()) {
            // set position controller
            sim_env::Robot::ControlCallback callback = std::bind(&RobotPositionController::control,
                                                                 _current_position_controller,
                                                                 _1, _2, _3, _4, _5);
            robot->getWorld()->getLogger()->logDebug("Setting position controller",
                                                     "[sim_env::viewer::Box2DControllerView::setController]");
            robot->setController(callback);
        } else {
            // set velocity controller
            sim_env::Robot::ControlCallback callback = std::bind(&Box2DRobotVelocityController::control,
                                                                 _current_velocity_controller,
                                                                 _1, _2, _3, _4, _5);
            robot->getWorld()->getLogger()->logDebug("Setting velocity controller",
                                                     "[sim_env::viewer::Box2DControllerView::setController]");
            robot->setController(callback);
        }
    } else {
        sim_env::Robot::ControlCallback empty_callback;
        robot->setController(empty_callback);
    }
}

void sim_env::viewer::Box2DControllerView::updateTarget() {
    sim_env::RobotPtr robot = _current_robot.lock();
    if (not robot) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Could not access selected robot - weak pointer expired.",
                       "[sim_env::viewer::Box2DControllerView::updateTarget]");
        return;
    }
    // If we have a robot we should always have controllers
    assert(_current_position_controller);
    assert(_current_velocity_controller);
    // get the logger from the robot (it may have a different level set)
    LoggerPtr logger = robot->getWorld()->getLogger();
    // we set the target's only for the active dofs
    Eigen::VectorXi active_dofs = robot->getActiveDOFs();
    Eigen::VectorXf target(active_dofs.size());
    // run over all dofs an retrieve the values
    for (size_t i = 0; i < active_dofs.size(); ++i) {
        int dof = active_dofs[i];
        if (dof < robot->getNumBaseDOFs()) { // it's a pose dof, i.e., x, y or theta
            QLineEdit* text_edit;
            switch(dof) {
                case 0: {
                    text_edit = _x_edit;
                    break;
                }
                case 1: {
                    text_edit = _y_edit;
                    break;
                }
                case 2: {
                    text_edit = _theta_edit;
                    break;
                }
                default: { // something is seriously wrong if this is the case
                    throw std::logic_error("[sim_env::viewer::Box2DControllerView::updateTarget]"
                                           "Encountered invalid dof index. Base dofs should be 0 <= i < 3.");
                }
            }
            // convert text value to float
            bool conversion_ok = false;
            float value = text_edit->text().toFloat(&conversion_ok);
            if (conversion_ok) {
                target[i] = value;
            } else {
                logger->logErr("Could not retrieve target position. Failed to convert to floating point.",
                               "[sim_env::viewer::Box2DController::View::updateTarget]");
                Eigen::VectorXi tmp_index(1);
                tmp_index[0] = dof;
                Eigen::VectorXf tmp_value(1);
                if (_velocity_button->isChecked()) {
                    tmp_value = robot->getDOFVelocities(tmp_index);
                } else {
                    tmp_value = robot->getDOFPositions(tmp_index);
                }
                target[i] = tmp_value[0];
            }
        } else {
            JointPtr joint = robot->getJointFromDOFIndex(dof);
            int joint_idx = joint->getJointIndex();
            Eigen::Array2f limits = joint->getPositionLimits();
            if (_velocity_button->isChecked()) {
                limits = joint->getVelocityLimits();
            }
            target[i] = utils::fromTickValue(_sliders.at((unsigned long) joint_idx)->value(), limits[0], limits[1]);
        }
    }
    if (_velocity_button->isChecked()) {
        _current_velocity_controller->setTargetVelocity(target);
    } else {
        _current_position_controller->setTargetPosition(target);
    }
}

//////////////////////////////////////// Box2DWorldView ////////////////////////////////////////
sim_env::viewer::Box2DWorldView::Box2DWorldView(float pw, float ph, int width, int height, QWidget *parent):QGraphicsView(parent) {
    _scene = new QGraphicsScene();
    _refresh_timer = nullptr;
    _width = width;
    _height = height;
    _rel_width = pw;
    _rel_height = ph;
    setScene(_scene);
    setRenderHint(QPainter::RenderHint::Antialiasing, true);
    // qt has it's y axis pointing downwards, so let's revert that axis
    setTransform(QTransform(1, 0, 0, 0, -1, 0, 0, 0, 1));
//    setTransformationAnchor(QGraphicsView::ViewportAnchor::AnchorUnderMouse);
}

sim_env::viewer::Box2DWorldView::~Box2DWorldView() {
}

void sim_env::viewer::Box2DWorldView::setBox2DWorld(sim_env::Box2DWorldPtr world) {
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
    Box2DWorldPtr world = _world.lock();
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
    // add a rectangle showing the world bounds
    Eigen::VectorXf world_bounds = world->getWorldBounds();
    QRect qrect(world_bounds[0],
                world_bounds[1],
                world_bounds[2] - world_bounds[0],
                world_bounds[3] - world_bounds[1]);
    _scene->addRect(qrect);
    if (not _refresh_timer) {
        _refresh_timer = new QTimer(this);
    }
    // TODO we probably don't need this. Instead only force redraws during propagation demo
    // TODO and when modifications on the model occurred.
    connect(_refresh_timer, SIGNAL(timeout()), this, SLOT(refreshView()));
    _refresh_timer->setSingleShot(false);
    _refresh_timer->start(40); // 25Hz
    // TODO BUG sometimes objects aren't drawn until I zoom in and out again
}

sim_env::WorldViewer::Handle sim_env::viewer::Box2DWorldView::drawFrame(const Eigen::Affine3f& frame, float length, float width) {
    QGraphicsItem* item = new Box2DFrameView(frame, length, width);
    return addDrawing(item);
}

sim_env::WorldViewer::Handle sim_env::viewer::Box2DWorldView::drawBox(const Eigen::Vector3f& pos,
                                                                      const Eigen::Vector3f& extent,
                                                                      const Eigen::Vector4f& color,
                                                                      bool solid,
                                                                      float edge_width)
{
    QPen pen;
    pen.setWidthF(edge_width);
    QColor q_color;
    q_color.setRgbF(color[0], color[1], color[2], color[3]);
    pen.setColor(q_color);
    QBrush brush;
    brush.setColor(q_color);
    if (solid) {
        brush.setStyle(Qt::BrushStyle::SolidPattern);
    } else {
        brush.setStyle(Qt::BrushStyle::NoBrush);
    }
    auto *rect = new QGraphicsRectItem(pos[0], pos[1], extent[0], extent[1]);
    rect->setBrush(brush);
    rect->setPen(pen);
    return addDrawing(rect);
}

sim_env::WorldViewer::Handle sim_env::viewer::Box2DWorldView::drawLine(const Eigen::Vector3f& start,
                                                                       const Eigen::Vector3f& end,
                                                                       const Eigen::Vector4f& color,
                                                                       float width) {
    QColor q_color;
    q_color.setRgbF(color[0], color[1],color[2], color[3]);
    QPen pen;
    pen.setWidthF(width);
    pen.setColor(q_color);
    Eigen::Vector3f dummy_end(end);
    if ((start - end).norm() == 0.0f) {
       dummy_end = end + Eigen::Vector3f(0.001f, 0.001f, 0.001f);
    }
    auto* line = new QGraphicsLineItem(start[0], start[1], dummy_end[0], dummy_end[1]);
    line->setPen(pen);
    return addDrawing(line);
}

sim_env::WorldViewer::Handle sim_env::viewer::Box2DWorldView::drawCircle(const Eigen::Vector3f& center,
                                                                         float radius,
                                                                         const Eigen::Vector4f& color,
                                                                         float width) {
    QColor q_color;
    q_color.setRgbF(color[0], color[1], color[2], color[3]);
    QPen pen;
    pen.setWidthF(width);
    pen.setColor(q_color);
    auto* circle = new QGraphicsEllipseItem(center[0] - radius, center[1] - radius, 2.0 * radius, 2.0 * radius);
    circle->setPen(pen);
    return addDrawing(circle);
}

void sim_env::viewer::Box2DWorldView::removeDrawing(const WorldViewer::Handle& handle)
{
    std::lock_guard<std::recursive_mutex> lock(_mutex_to_remove);
    auto iter = _drawings.find(handle.getID());
    if (iter != _drawings.end()) {
        _items_to_remove.push(iter->second);
        _drawings.erase(iter);
    } else {
        sim_env::LoggerPtr logger = getLogger();
        logger->logWarn("Requested to remove drawing with invalid handle.",
                        "[sim_env::viewer::Box2DWorldView::removeDrawing]");
    }
}

void sim_env::viewer::Box2DWorldView::removeAllDrawings() {
    std::lock_guard<std::recursive_mutex> lock(_mutex_to_remove);
    for (auto& entry : _drawings) {
        _items_to_remove.push(entry.second);
    }
    _drawings.clear();
}

QSize sim_env::viewer::Box2DWorldView::sizeHint() const {
    auto* parent = parentWidget();
    if (parent) {
        QSize parent_size = parent->size();
        return {(int)(_rel_width * parent_size.width()),
                (int)(_rel_height * parent_size.height())};
    }
    return {_width, _height};
}

void sim_env::viewer::Box2DWorldView::wheelEvent(QWheelEvent *event) {
    // TODO limit total scene size somhow to bounding box (i.e. we do not want the scrollbars to show
    // TODO when we see all object
    scaleView(pow(2.0, -event->delta() / 240.0));
}

void sim_env::viewer::Box2DWorldView::refreshView() {
//    auto logger = DefaultLogger::getInstance();
//    logger->logDebug("REFRESHING WORLD VIEW");
    // first add new items to the scene if we have any
    {
        std::lock_guard<std::recursive_mutex> add_lock(_mutex_to_add);
        while (not _items_to_add.empty()) {
            _scene->addItem(_items_to_add.front());
            _items_to_add.pop();
        }
    }
    // next remove items from the scene if there were requests
    {
        std::lock_guard<std::recursive_mutex> rm_lock(_mutex_to_remove);
        while (not _items_to_remove.empty()) {
            QGraphicsItem* item = _items_to_remove.front();
            _scene->removeItem(item);
            delete item;
            _items_to_remove.pop();
        }
    }
    _scene->update();
    update();
    emit refreshTick();
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

sim_env::WorldViewer::Handle sim_env::viewer::Box2DWorldView::addDrawing(QGraphicsItem* item) {
    sim_env::WorldViewer::Handle handle;
    std::lock_guard<std::recursive_mutex> lock(_mutex_to_add);
    // force drawings to be in the back
    item->setZValue(-1.0);
    _items_to_add.push(item);
    _drawings[handle.getID()] = item;
    return handle;
}

sim_env::LoggerPtr sim_env::viewer::Box2DWorldView::getLogger() const
{
    Box2DWorldPtr world = _world.lock();
    if (world) {
        return world->getLogger();
    } else {
        return sim_env::DefaultLogger::getInstance();
    }
}
/////////////////////////////////// Box2DSimulationController ///////////////////////////////////
sim_env::viewer::Box2DSimulationController::Box2DSimulationController(Box2DWorldPtr world) {
    _simulation_thread.is_running = false;
    _simulation_thread.world = world;
}

sim_env::viewer::Box2DSimulationController::~Box2DSimulationController() {
    // TODO there is crash happening here
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

void sim_env::Box2DWorldViewer::show(int argc, const char* const* argv) {
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

sim_env::WorldViewer::Handle sim_env::Box2DWorldViewer::drawFrame(const Eigen::Affine3f &transform, float length, float width) {
    return _world_view->drawFrame(transform, length, width);
}

sim_env::WorldViewer::Handle sim_env::Box2DWorldViewer::drawBox(const Eigen::Vector3f &pos,
                                                                const Eigen::Vector3f &extent,
                                                                const Eigen::Vector4f &color,
                                                                bool solid,
                                                                float edge_width) {
    return _world_view->drawBox(pos, extent, color, solid, edge_width);
}

sim_env::WorldViewer::Handle sim_env::Box2DWorldViewer::drawLine(const Eigen::Vector3f& start,
                                                                 const Eigen::Vector3f& end,
                                                                 const Eigen::Vector4f& color,
                                                                 float width) {
    return _world_view->drawLine(start, end, color, width);
}
sim_env::WorldViewer::Handle sim_env::Box2DWorldViewer::drawSphere(const Eigen::Vector3f& center, float radius,
                                                                   const Eigen::Vector4f& color,
                                                                   float width) {
    return _world_view->drawCircle(center, radius, color, width);
}

void sim_env::Box2DWorldViewer::removeDrawing(const sim_env::WorldViewer::Handle &handle)
{
    _world_view->removeDrawing(handle);
}

void sim_env::Box2DWorldViewer::removeAllDrawings() {
    _world_view->removeAllDrawings();
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
    _world_view = new viewer::Box2DWorldView(0.8f, 0.8f);
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
    container->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));

    // bottom widget (simulation control)
    createBottomBar();
    root_layout->addWidget(container);
    root_layout->addWidget(_bottom_tab_widget);
    _root_widget->setLayout(root_layout);
}

void sim_env::Box2DWorldViewer::createBottomBar() {
    _bottom_tab_widget = new QTabWidget(_root_widget.get());
    // build simulation control view
    QGroupBox* simulation_control_view = new QGroupBox("Dynamics model control");
    _bottom_tab_widget->addTab(simulation_control_view, "Simulation control");
    QHBoxLayout* sim_control_layout = new QHBoxLayout();
    QPushButton* sim_control_button = new QPushButton("Run simulation", simulation_control_view);
    sim_control_button->setCheckable(true);
    sim_control_layout->addWidget(sim_control_button);
    simulation_control_view->setLayout(sim_control_layout);
    QObject::connect(sim_control_button, SIGNAL(clicked(bool)), _simulation_controller.get(), SLOT(triggerSimulation(bool)));
    // create robot controller view
    viewer::Box2DControllerView* robot_control_view = new viewer::Box2DControllerView();
    QObject::connect(_world_view, SIGNAL(objectSelected(sim_env::ObjectWeakPtr)),
                     robot_control_view, SLOT(setCurrentObject(sim_env::ObjectWeakPtr)));
    _bottom_tab_widget->addTab(robot_control_view, "Robot control");
    _bottom_tab_widget->setMaximumSize(1980, 300);
}

QWidget* sim_env::Box2DWorldViewer::createSideBar() {
    // TODO Unfortunately this doesn't work with a scroll area (the layout can not be changed on the fly)
    // QScrollArea* scroll_view = new QScrollArea();
    auto* state_view = new viewer::Box2DObjectStateView();
    QObject::connect(_world_view, SIGNAL(objectSelected(sim_env::ObjectWeakPtr)),
            state_view, SLOT(setCurrentObject(sim_env::ObjectWeakPtr)));
    QObject::connect(state_view, SIGNAL(newUserState()), _world_view,
              SLOT(refreshView()));
    QObject::connect(_world_view, SIGNAL(refreshTick()), state_view, SLOT(stateUpdate()));
    state_view->setMaximumSize(200, 1080);
    return state_view;
}

void sim_env::Box2DWorldViewer::addCustomWidget(QWidget *widget, const std::string& name) {
    _bottom_tab_widget->addTab(widget, name.c_str());
}


