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

namespace sim_env {
    namespace viewer {

        class Box2DObjectView : public QGraphicsItem {
        public:
            Box2DObjectView(Box2DObjectConstPtr object);
            ~Box2DObjectView();
            QRectF boundingRect() const;
            void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
        protected:
            Box2DObjectConstWeakPtr _object;
        };

        class Box2DRobotView : public QGraphicsItem {
        public:
            Box2DRobotView(Box2DRobotConstPtr robot);
            ~Box2DRobotView();
            QRectF boundingRect() const;
            void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
        protected:
            Box2DRobotConstWeakPtr _robot;
        };

        class Box2DLinkView : public QGraphicsItem {
        public:
            Box2DLinkView(Box2DLinkConstPtr link, QGraphicsItem *parent = 0);
            QRectF boundingRect() const;
            void setColors(const QColor& fill_color, const QColor& border_color);
            void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
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

        class Box2DWorldView : public QGraphicsView {
        public:
            Box2DWorldView(int width, int height, QWidget *parent = 0);
            ~Box2DWorldView();
            void setBox2DWorld(Box2DWorldConstPtr world);
            /**
             * Repopulates the visualized scene by recreating all child views based on the currently
             * set Box2D world.
             */
            void repopulate();
        protected:
            void wheelEvent(QWheelEvent *event);
            void scaleView(double scale_factor);
            // Variables
            QGraphicsScene *_scene;
            int _width;
            int _height;
            Box2DWorldConstWeakPtr _world;
            std::vector<Box2DObjectView *> _object_views;
            std::vector<Box2DRobotView *> _robot_views;
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
        void drawFrame(const Eigen::Vector3f &transform) override;
    protected:
        void log(const std::string& msg, const std::string& prefix,
                 Logger::LogLevel level=Logger::LogLevel::Info) const;

    private:
        Box2DWorldWeakPtr _world;
        std::unique_ptr<QApplication> _app;
        std::unique_ptr<viewer::Box2DWorldView> _world_view;
    };
}

#endif //BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H
