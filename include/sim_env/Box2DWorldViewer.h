//
// Created by joshua on 6/26/17.
//

#ifndef BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H
#define BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H

#include <sim_env/Box2DWorld.h>
#include <sim_env/Box2DController.h>
// Qt includes
#include <QtGui/QApplication>
#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsItem>
#include <QtGui/QColor>
#include <QtGui/QGroupBox>
#include <QtGui/QLineEdit>
#include <QtGui/QRadioButton>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QFormLayout>
#include <QtGui/QLabel>
#include <QtGui/QTabWidget>
// stl includes
#include <thread>

namespace sim_env {
    namespace viewer {
      class Box2DWorldView;
        namespace utils {
            int toTickValue(float value, float min, float max);
            float fromTickValue(int tick, float min, float max);
            constexpr float LARGE_FLOATING_NUMBER = 100000.0f;
        }

        ////////////////////// VIEWS OF WOLRD COMPONENTS, I.E. VISUAL ITEMS /////////////////////////
        class Box2DObjectView : public QGraphicsItem {
        public:
            Box2DObjectView(Box2DObjectPtr object, Box2DWorldView* world_view);
            ~Box2DObjectView();
            QRectF boundingRect() const override;
            void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
        protected:
            void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
            Box2DObjectWeakPtr _object;
        private:
            Box2DWorldView* _world_view; // raw pointer, but this view is destroyed
                                        // when the parent view is desrtroyed
        };

        class Box2DRobotView : public QGraphicsItem {
        public:
            Box2DRobotView(Box2DRobotPtr robot, Box2DWorldView* world_view);
            ~Box2DRobotView();
            QRectF boundingRect() const override;
            void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
        protected:
            void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
            Box2DRobotWeakPtr _robot;
        private:
            Box2DWorldView* _world_view; // raw pointer, but this view is destroyed
                                        // when the parent view is desrtroyed
        };

        class Box2DLinkView : public QGraphicsItem {
        public:
            Box2DLinkView(Box2DLinkConstPtr link, QGraphicsItem *parent = 0);
            QRectF boundingRect() const override;
            void setColors(const QColor& fill_color, const QColor& border_color);
            void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
        protected:
            Box2DLinkConstWeakPtr _link;
        private:
            std::vector<QPolygonF> _polygons;
            QRectF _bounding_rect;
            QColor _border_color;
            QColor _fill_color;
        };

        class Box2DJointView : public QGraphicsItem {
        public:
            Box2DJointView(Box2DJointConstPtr joint, QGraphicsScene *scene, QGraphicsItem *parent = 0);
//            ~Box2DJointView();
        protected:
            Box2DJointConstWeakPtr _joint;
        };

        class Box2DFrameView : public QGraphicsItem {
        public:
            Box2DFrameView(const Eigen::Affine3f& frame, float length, float width, QGraphicsItem* parent=0);
            QRectF boundingRect() const override;
            void paint(QPainter* painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
        private:
            QLineF _x_axis;
            QLineF _y_axis;
            float _width;
        };

        ////////////////////// QT Widgets ////////////////////////
        class LineEditChangeDetector : public QObject {
            Q_OBJECT
        public:
            LineEditChangeDetector(QObject* parent=0);
            ~LineEditChangeDetector();
        signals:
            void valueChanged(QLineEdit* line_edit);
        protected:
            bool eventFilter(QObject* qobject, QEvent* event);
        };

        class Box2DObjectStateView : public QGroupBox {
            Q_OBJECT
        public:
            Box2DObjectStateView(QWidget* parent=0);
            virtual ~Box2DObjectStateView();

        public slots:
            // called by robot/object view on click, sets the object for which the state is to be shown
            void setCurrentObject(sim_env::ObjectWeakPtr object);
            // called when a slider is changed
            void sliderChange(int value);
            // called when a line edit is changed
            void lineEditChange(QLineEdit* line_edit);
            // called when enable button changes its state
            void setViewMode(bool enable_edit);
            // called when the world state has changed, i.e. the current's object state may be different
            void stateUpdate();
        signals:
            void newUserState();
        protected:
            static const std::string LINE_EDIT_TYPE_PROPERTY_KEY;
            static const std::string LINE_EDIT_DOF_PROPERTY_KEY;
            constexpr static int LINE_EDIT_OBJECT_POSE_TYPE = 0;
            constexpr static int LINE_EDIT_OBJECT_VEL_TYPE = 1;
            constexpr static int LINE_EDIT_JOINT_VEL_TYPE = 2;
        private:
            sim_env::ObjectWeakPtr _current_object;
            std::vector<QLineEdit*> _object_pose_edits;
            std::vector<QSlider*> _joint_position_sliders;
            std::vector<QLineEdit*> _object_velocity_edits;
            std::vector<QLineEdit*> _joint_velocity_edits;
            QFormLayout* _form_layout;
            QPushButton* _mode_button;
            LineEditChangeDetector* _line_edit_change_detector;
            QLabel* _collision_display;

            void createBaseDOFEdits(ObjectPtr object);
            void synchView();
            void setSliderValue(QSlider* slider, float value, float min_value, float max_value);
            void showValues(bool update_input_fields);
        };

        class Box2DControllerView : public QGroupBox {
            Q_OBJECT
        public:
            Box2DControllerView(QWidget* parent=0);
            ~Box2DControllerView();

        public slots:
            void setCurrentObject(sim_env::ObjectWeakPtr object); // called by world view, if an object was selected
            void sliderChange(int value); // called when a slider is changed
            void lineEditChange(QLineEdit* line_edit); // called when a line edit text was changed
            void triggerController(bool enable);
            void triggerControllerTypeChange(bool toggled); // called when user selects position/velocity control
        private:
            // member functions
            void initView();
            void updateView();
            void updateTarget();
            void synchSliderValue(QSlider* slider, sim_env::JointConstPtr joint);
            void synchTextValue(int dof_idx, sim_env::RobotPtr robot);
            void setController();
            // member variables
            sim_env::RobotWeakPtr _current_robot;
            sim_env::RobotPositionControllerPtr _current_position_controller;
            sim_env::Box2DRobotVelocityControllerPtr _current_velocity_controller;
            // TODO in case robots can be deleted and new robots with the same names can be readded
            // TODO to the scene, this map may contain controllers for non-existing robots
            std::map<std::string, sim_env::RobotPositionControllerPtr> _position_controllers;
            std::map<std::string, sim_env::Box2DRobotVelocityControllerPtr> _velocity_controllers;
            QPushButton* _enable_button;
            QRadioButton* _position_button;
            QRadioButton* _velocity_button;
            QLineEdit* _x_edit;
            QLineEdit* _y_edit;
            QLineEdit* _theta_edit;
            std::vector<QSlider*> _sliders;
            LineEditChangeDetector* _line_edit_change_detector;
        };

        class Box2DWorldView : public QGraphicsView {
        Q_OBJECT
            friend class Box2DRobotView;
            friend class Box2DObjectView;
        public:
            Box2DWorldView(float pw, float ph, int width=800, int height=900, QWidget *parent = 0);
            ~Box2DWorldView();
            void setBox2DWorld(Box2DWorldPtr world);
            /**
             * Repopulates the visualized scene by recreating all child views based on the currently
             * set Box2D world.
             */
            void repopulate();
            WorldViewer::Handle drawFrame(const Eigen::Affine3f &frame, float length=1.0f, float width=0.01f);
            WorldViewer::Handle drawBox(const Eigen::Vector3f& pos,
                                        const Eigen::Vector3f& extent,
                                        const Eigen::Vector4f& color=Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
                                        bool solid=true,
                                        float edge_width=0.1f);
            WorldViewer::Handle drawLine(const Eigen::Vector3f& start,
                                         const Eigen::Vector3f& end,
                                         const Eigen::Vector4f& color=Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
                                         float width=0.1f);
            WorldViewer::Handle drawCircle(const Eigen::Vector3f& center,
                                           float radius,
                                           const Eigen::Vector4f& color=Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
                                           float width=0.1f);
            void removeDrawing(const WorldViewer::Handle& handle);
            void removeAllDrawings();
            virtual QSize sizeHint() const override;

        public slots:
            void refreshView();
        signals:
            void objectSelected(sim_env::ObjectWeakPtr object);
            void refreshTick();
        protected:
            void setSelectedObject(sim_env::ObjectWeakPtr object);
            void wheelEvent(QWheelEvent *event) override;
            void scaleView(double scale_factor);
            WorldViewer::Handle addDrawing(QGraphicsItem* item);
            LoggerPtr getLogger() const;
            // Variables
            QGraphicsScene *_scene;
            QTimer* _refresh_timer;
            int _width;
            int _height;
            float _rel_width;
            float _rel_height;
            Box2DWorldWeakPtr _world;
            sim_env::ObjectWeakPtr _currently_selected_object;
            std::vector<Box2DObjectView *> _object_views;
            std::vector<Box2DRobotView *> _robot_views;
            std::map<unsigned int, QGraphicsItem*> _drawings;
        };

        class Box2DSimulationController : public QObject {
            Q_OBJECT
        public:
            Box2DSimulationController(Box2DWorldPtr world);
            ~Box2DSimulationController();
            void startSimulation();
            void stopSimulation();
        public slots:
            void triggerSimulation(bool run);
        private:
            void terminateThread();
            // Simulation thread for running box2d world.
            struct SimulationThread {
                std::thread simulation_thread;
                bool is_running;
                Box2DWorldWeakPtr world;
                void run();
            };
            SimulationThread _simulation_thread;
        };
    }

    class Box2DWorldViewer : public sim_env::WorldViewer {
    public:
        Box2DWorldViewer(Box2DWorldPtr world);
        ~Box2DWorldViewer();
        /**
         * Executes the GUI-loop. This function is blocking until the application is terminated.
         * It must be executed in the main loop. Hence, any planning algorithm needs to be run in
         * a separate thread. The drawing functions provided by this viewer are thread-safe.
         * @warning You must call show(...) before calling this function.
         */
        int run();

        /**
         * Creates a Qt Application and shows the visualizer.
         * @param argc - number of arguments stored in argv
         * @param argv - pointer to array of c-style strings. These parameters are forwarded to QApplication.
         */
        void show(int argc = 0, const char* const* argv = nullptr);
        Handle drawFrame(const Eigen::Affine3f &transform,
                         float length=1.0f,
                         float width=0.01f) override;
        Handle drawBox(const Eigen::Vector3f& pos,
                       const Eigen::Vector3f& extent,
                       const Eigen::Vector4f& color=Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
                       bool solid=false,
                       float edge_width=0.1f) override;
        Handle drawLine(const Eigen::Vector3f& start,
                        const Eigen::Vector3f& end,
                        const Eigen::Vector4f& color=Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
                        float width=0.1f) override;
        Handle drawSphere(const Eigen::Vector3f& center,
                          float radius,
                          const Eigen::Vector4f& color=Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
                          float width=0.1f) override;
        void removeDrawing(const Handle& handle) override;
        void removeAllDrawings() override;
        void addCustomWidget(QWidget* widget, const std::string& name);

    protected:
        void log(const std::string& msg, const std::string& prefix,
                 Logger::LogLevel level=Logger::LogLevel::Info) const;
        void deleteArgs();

    private:
        Box2DWorldWeakPtr _world;
        std::unique_ptr<QApplication> _app;
        std::unique_ptr<QWidget> _root_widget;
        QTabWidget* _bottom_tab_widget;
        viewer::Box2DWorldView* _world_view;
        // this is a qt object, so we keep a pointer.
        std::unique_ptr<viewer::Box2DSimulationController> _simulation_controller;
        int _argc;
        char** _argv;
        bool _is_showing;

        // private methods
        void createUI();
        void createBottomBar();
        QWidget* createSideBar();
    };
}

#endif //BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H
