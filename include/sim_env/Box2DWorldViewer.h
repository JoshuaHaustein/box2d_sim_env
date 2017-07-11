//
// Created by joshua on 6/26/17.
//

#ifndef BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H
#define BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H

#include <QtGui/QApplication>
#include <sim_env/Box2DWorld.h>
#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsItem>
#include <QtGui/QColor>
#include <QtGui/QGroupBox>

class QLineEdit;
class QSlider;
class QFormLayout;

namespace sim_env {
    namespace viewer {
      class Box2DWorldView;

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

        class Box2DWorldView : public QGraphicsView {
            Q_OBJECT
            friend class Box2DRobotView;
            friend class Box2DObjectView;
        public:
            Box2DWorldView(int width, int height, QWidget *parent = 0);
            ~Box2DWorldView();
            void setBox2DWorld(Box2DWorldConstPtr world);
            /**
             * Repopulates the visualized scene by recreating all child views based on the currently
             * set Box2D world.
             */
            void repopulate();
            void drawFrame(const Eigen::Affine3f &frame, float length=1.0f, float width=0.01f);
            virtual QSize sizeHint() const override;
//            virtual QSize minimumSizeHint() const override;

        public slots:
            void refreshView();
        signals:
            void objectSelected(sim_env::ObjectWeakPtr object);
        protected:
            void setSelectedObject(sim_env::ObjectWeakPtr object);
            void wheelEvent(QWheelEvent *event) override;
            void scaleView(double scale_factor);
            // Variables
            QGraphicsScene *_scene;
            QTimer* _refresh_timer;
            int _width;
            int _height;
            Box2DWorldConstWeakPtr _world;
            sim_env::ObjectWeakPtr _currently_selected_object;
            std::vector<Box2DObjectView *> _object_views;
            std::vector<Box2DRobotView *> _robot_views;
        };

        class Box2DObjectStateView : public QGroupBox {
            Q_OBJECT
        public:
            Box2DObjectStateView(QWidget* parent=0);
            virtual ~Box2DObjectStateView();

        public slots:
            void setCurrentObject(sim_env::ObjectWeakPtr object); // called by robot/object view on click
            void setObjectState(); // called by 'set state' button
            void sliderChange(int value); // called when a slider is changed
        signals:
            void valuesChanged();
        private:
            sim_env::ObjectWeakPtr _current_object;
            std::vector<QLineEdit*> _object_pose_edits;
            std::vector<QSlider*> _joint_position_sliders;
            QFormLayout* _form_layout;
            bool _ignore_input_signals;

            void synchView();
            void showValues();
            int toTickValue(float value, float min, float max);
            float fromTickValue(int tick, float min, float max);

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
        void show(int argc = 0, char** argv = nullptr);
        void drawFrame(const Eigen::Affine3f &transform, float length=1.0f, float width=0.01f) override;
    protected:
        void log(const std::string& msg, const std::string& prefix,
                 Logger::LogLevel level=Logger::LogLevel::Info) const;
        void deleteArgs();

    private:
        Box2DWorldWeakPtr _world;
        std::unique_ptr<QApplication> _app;
        std::unique_ptr<QWidget> _root_widget;
        viewer::Box2DWorldView* _world_view;
        int _argc;
        char** _argv;

        void createUI();
        QWidget* createSideBar();
    };
}

#endif //BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H
