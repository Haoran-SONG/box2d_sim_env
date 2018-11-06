//
// Created by joshua on 6/26/17.
//

#ifndef BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H
#define BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H

#include <sim_env/Box2DController.h>
#include <sim_env/Box2DWorld.h>
// Qt includes
#include <QRectF>
#include <QTimer>
#include <QtGui/QApplication>
#include <QtGui/QColor>
#include <QtGui/QFormLayout>
#include <QtGui/QGraphicsItem>
#include <QtGui/QGraphicsSceneWheelEvent>
#include <QtGui/QGraphicsView>
#include <QtGui/QGroupBox>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QTabWidget>
// stl includes
#include <queue>
#include <thread>

namespace sim_env {
namespace viewer {
    class Box2DScene;
    namespace utils {
        int toTickValue(float value, float min, float max);
        float fromTickValue(int tick, float min, float max);
        constexpr float LARGE_FLOATING_NUMBER = 100000.0f;
    } // namespace utils

    ////////////////////// VIEWS OF WOLRD COMPONENTS, I.E. VISUAL ITEMS /////////////////////////
    class Box2DLinkView : public QGraphicsItem {
    public:
        Box2DLinkView(Box2DLinkConstPtr link, QGraphicsItem* parent = 0);
        QRectF boundingRect() const override;
        void setColors(const QColor& fill_color, const QColor& border_color, const QColor& ball_color);
        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    protected:
        Box2DLinkConstWeakPtr _link;

    private:
        std::vector<QPolygonF> _polygons;
        std::vector<sim_env::Ball> _balls;
        QRectF _bounding_rect;
        QColor _border_color;
        QColor _fill_color;
        QColor _ball_color;
    };

    class Box2DObjectView : public QGraphicsItem {
    public:
        Box2DObjectView(Box2DObjectPtr object, Box2DScene* world_scene);
        ~Box2DObjectView();
        QRectF boundingRect() const override;
        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
        void setColor(float r, float g, float b);
        void resetColor();

    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
        void wheelEvent(QGraphicsSceneWheelEvent* event) override;
        QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;
        Box2DObjectWeakPtr _object;

    private:
        Box2DScene* _world_scene; // raw pointer, but this view is destroyed
            // when the scene is destroyed
        std::vector<Box2DLinkView*> _link_views;
        QColor _default_color;
    };

    class Box2DRobotView : public QGraphicsItem {
    public:
        Box2DRobotView(Box2DRobotPtr robot, Box2DScene* world_scene);
        ~Box2DRobotView();
        QRectF boundingRect() const override;
        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
        void setColor(float r, float g, float b);
        void resetColor();

    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
        void wheelEvent(QGraphicsSceneWheelEvent* event) override;
        QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;
        Box2DRobotWeakPtr _robot;

    private:
        Box2DScene* _world_scene; // raw pointer, but this view is destroyed
            // when the parent view is desrtroyed
        std::vector<Box2DLinkView*> _link_views;
        QColor _default_color;
    };

    // class Box2DJointView : public QGraphicsItem {
    // public:
    //     Box2DJointView(Box2DJointConstPtr joint, QGraphicsItem* parent = 0);
    //     //            ~Box2DJointView();
    // protected:
    //     Box2DJointConstWeakPtr _joint;
    // };

    class Box2DFrameView : public QGraphicsItem {
    public:
        Box2DFrameView(const Eigen::Affine3f& frame, float length, float width, QGraphicsItem* parent = 0);
        QRectF boundingRect() const override;
        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    private:
        QLineF _x_axis;
        QLineF _y_axis;
        float _width;
    };

    class Box2DScene : public QGraphicsScene {
        Q_OBJECT
        friend class Box2DRobotView;
        friend class Box2DObjectView;

    public:
        Box2DScene(Box2DWorldPtr world);
        ~Box2DScene();
        WorldViewer::Handle drawFrame(const Eigen::Affine3f& frame, float length = 1.0f, float width = 0.01f);
        WorldViewer::Handle drawBox(const Eigen::Vector3f& pos,
            const Eigen::Vector3f& extent,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            bool solid = true,
            float edge_width = 0.1f);
        WorldViewer::Handle drawLine(const Eigen::Vector3f& start,
            const Eigen::Vector3f& end,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            float width = 0.1f);
        WorldViewer::Handle drawCircle(const Eigen::Vector3f& center,
            float radius,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            float width = 0.1f);
        /**
             * Draws the given voxel grid. Since Box2D is 2d, the visualization is 2D as well, hence
             * only grids with size of 1 in z are supported. If the given grid has size != 1, a logic
             * error is thrown.
             */
        WorldViewer::Handle drawVoxelGrid(const grid::VoxelGrid<float, Eigen::Vector4f>& grid,
            const WorldViewer::Handle& old_handle);
        bool renderImage(const std::string& filename, unsigned int width, unsigned int height, bool include_drawings);
        bool renderImage(const std::string& filename, unsigned int width, unsigned int height, bool include_drawings, const QRectF& render_region);
        void removeDrawing(const WorldViewer::Handle& handle);
        void removeAllDrawings();
        void setColor(const std::string& name, float r, float g, float b);
        void setColor(const std::string& name, const Eigen::Vector4f& color);
        void resetColor(const std::string& name);
        void setObjectVisible(const std::string& name, bool visible);
        // Return bounds of the world in scene coordinates
        QRectF getWorldBounds() const;
        // Return bounds so that all robots/objects (and optionally all other drawings) are within these bounds.
        QRectF getDrawingBounds(bool include_drawings) const;
        void synchronizeScene();

    signals:
        void objectSelected(sim_env::ObjectWeakPtr object);

    protected:
        /**
          * Repopulates the visualized scene by recreating all child views based on the currently
          * set Box2D world.
          */
        void repopulate();
        void setSelectedObject(sim_env::ObjectWeakPtr object);
        void clearScene(bool clear_drawings = true);
        WorldViewer::Handle addDrawing(QGraphicsItem* item);
        LoggerPtr getLogger() const;

    private:
        QRectF _world_bounds;
        Box2DWorldWeakPtr _world;
        sim_env::ObjectWeakPtr _currently_selected_object;
        std::map<std::string, Box2DObjectView*> _object_views;
        std::map<std::string, Box2DRobotView*> _robot_views;
        std::map<unsigned int, QGraphicsItem*> _drawings;
        // The following members are required to ensure we only add/remove QGraphicsItem in the main thread.
        std::recursive_mutex _mutex_modify_graphics_items;
        std::queue<QGraphicsItem*> _items_to_add;
        std::queue<QGraphicsItem*> _items_to_remove;
        WorldViewer::Handle _world_bounds_handle;
    };
    ////////////////////// QT Widgets ////////////////////////
    class LineEditChangeDetector : public QObject {
        Q_OBJECT
    public:
        LineEditChangeDetector(QObject* parent = 0);
        ~LineEditChangeDetector();
    signals:
        void valueChanged(QLineEdit* line_edit);

    protected:
        bool eventFilter(QObject* qobject, QEvent* event);
    };

    class Box2DObjectStateView : public QGroupBox {
        Q_OBJECT
    public:
        Box2DObjectStateView(QWidget* parent = 0);
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
        Box2DControllerView(QWidget* parent = 0);
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

    public:
        Box2DWorldView(float pw, float ph, int width = 800, int height = 900, QWidget* parent = 0);
        ~Box2DWorldView();
        void setBox2DWorld(sim_env::Box2DWorldPtr world);
        // call when the view is actually shown on the screen
        void showEvent(QShowEvent* event) override;
        QSize sizeHint() const override;
        void setRelativeSize(float width, float height);
        void centerCamera(bool include_drawings);
        void resetCamera();
        Box2DScene* getBox2DScene() const;
    public slots:
        void refreshView();
    signals:
        void refreshTick();

    protected:
        void wheelEvent(QWheelEvent* event) override;
        void scaleView(double scale_factor);
        // Variables
        Box2DScene* _scene;
        QTimer _refresh_timer;
        int _width;
        int _height;
        float _rel_width;
        float _rel_height;
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
} // namespace viewer

class Box2DWorldViewer : public sim_env::WorldViewer {
public:
    class Box2DImageRenderer : public sim_env::WorldViewer::ImageRenderer {
        /**
         * A Box2DImageRenderer allows you to render images showing a Box2DWorld. Use this if
         * you want to save images of your scene to disk or use an image of the scene for anything else
         * other than showing it on the screen. A Box2DImageRenderer is thread safe. If you intend to render
         * images in parallel, however, it is a good idea to generate multiple Box2DImageRenderer.
         * Every operation requires the executing thread to acquire a lock, so access by multiple threads is serialized.
         * 
         * NOTE: Before you can use a Box2DImageRenderer, you must call init(...) on a Box2DWorldViewer to initialize a
         * Qt context. This renderer also uses Qt to render images, and without calling this function, Qt does not work.
        */
    public:
        Box2DImageRenderer(Box2DWorldPtr world);
        ~Box2DImageRenderer();
        /**
         *  Position the camera such that all bodies in the scene are visible.
         * @param include_drawings - if true, it also ensures that all user drawings are visible
         */
        void centerCamera(bool include_drawings = false) override;
        /**
             * Draw a box at the provided position with the given extents.
             * The box spans from pos to pos + extents
             * @param pos - box position (with minimal coordinates)
             * @param extent - (width, depth, height)
             * @param color - rgba color (in range [0,1]^4)
             * @param solid - flag whether to draw a solid or non-solid box
             * @param edge_width - thickness of lines
             */
        Handle drawBox(const Eigen::Vector3f& pos, const Eigen::Vector3f& extent,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            bool solid = false, float edge_width = 0.1f) override;
        /**
         * Draw a coordinate frame.
         * @param transform - the transformation matrix from the target frame to world frame
         * @param length - length of arrows
         * @param widt - width of arrows.
         */
        Handle drawFrame(const Eigen::Affine3f& transform, float length = 1.0f, float width = 0.1f) override;
        /**
             * Draws a line from position start to position end.
             * @param start  - position where the line segment should start
             * @param end  - position where the line segment should end
             * @param color - rgba color (in range [0,1]^4)
             * @param width - width of the line
             * @return handle to delete the line again
             */
        Handle drawLine(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            float width = 0.1f) override;

        /**
             * Draws a sphere with the given radius centered at center.
             * @param center - center position of the sphere.
             * @param radius - radius of the sphere.
             * @param color - (optional) rbda color of the sphere
             * @param width - (optional) width of the line
             * TODO: some option to only draw a 2d circle
             * @return handle to delete this sphere again
             */
        Handle drawSphere(const Eigen::Vector3f& center, float radius,
            const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
            float width = 0.1f) override;

        /**
             * Draws the given voxel grid. The grid is assumed to store a color for each cell.
             * @param grid - a voxel grid that stores the color for each voxel
             * @param old_handle (optional) - a handle to a previously drawn instance of a voxel grid that is supposed to
             *      be replaced by this new drawing. Providing this may save resources, but you need to ensure that the new
             *      grid has the same dimensions as the previous one.
             * @return handle to delete this grid again
             */
        Handle drawVoxelGrid(const grid::VoxelGrid<float, Eigen::Vector4f>& grid, const Handle& old_handle = Handle(false)) override;

        /**
         *  Render the scene from the current camera view to an image and store this image under the given name.
         *  The underlying implementation guarantees that this method is thread-safe.
         *  @param filename - path + name of where to store the image. The image format may depend on the implementation.
         *  @param width - width in pixels of the image
         *  @param height - height in pixels
         *  @param include_drawings - if true, also render additional drawings in image, else not
         */
        bool renderImage(const std::string& filename, unsigned int width, unsigned int height, bool include_drawings = false) override;

        /**
         *  Resets the camera to default view.
         */
        void resetCamera() override;

        /**
         * Sets whether to show the object/robot with the given name when rendering images.
         * By default all objects/robots are shown.
         * @param name - name of the object to show or hide
         * @param visible - if true, show it, else hide. 
         */
        void setVisible(const std::string& name, bool visible) override;

        /**
         * Set the color of the object/robot with the given name.
         * By default all objects/robots are shown.
         * @param name - name of the object to show or hide
         * @param visible - if true, show it, else hide. 
         */
        void setColor(const std::string& name, const Eigen::Vector4f& color) override;

    private:
        viewer::Box2DScene _world_scene;
        QRectF _render_region;
        std::recursive_mutex _mutex;
    };

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
         * Creates a Qt Application - must be called before anything else can be done!
         * @param argc - number of arguments stored in argv
         * @param argv - pointer to array of c-style strings. These parameters are forwarded to QApplication.
         */
    void init(int argc = 0, char** argv = nullptr);
    /**
     *  Show the actual GUI.
     */
    void show();
    Handle drawFrame(const Eigen::Affine3f& transform,
        float length = 1.0f,
        float width = 0.01f) override;
    Handle drawBox(const Eigen::Vector3f& pos,
        const Eigen::Vector3f& extent,
        const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
        bool solid = false,
        float edge_width = 0.1f) override;
    Handle drawLine(const Eigen::Vector3f& start,
        const Eigen::Vector3f& end,
        const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
        float width = 0.1f) override;
    Handle drawSphere(const Eigen::Vector3f& center,
        float radius,
        const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
        float width = 0.1f) override;
    /**
         * Draws the given voxel grid. Since Box2D is 2d, the visualization is 2D as well, hence
         * only grids with size of 1 in z are supported. If the given grid has size != 1, a logic
         * error is thrown.
         */
    Handle drawVoxelGrid(const grid::VoxelGrid<float, Eigen::Vector4f>& grid,
        const WorldViewer::Handle& old_handle) override;
    ImageRendererPtr createImageRenderer() override;
    bool renderImage(const std::string& filename, unsigned int width, unsigned int height, bool include_drawings = false) override;
    void centerCamera(bool include_drawings = false) override;
    void resetCamera() override;
    WorldPtr getWorld() const override;
    void removeDrawing(const Handle& handle) override;
    void removeAllDrawings() override;
    void setVisible(const std::string& name, bool visible) override;
    /**
     * Set the color of the object/robot with the given name.
     * By default all objects/robots are shown.
     * @param name - name of the object to show or hide
     * @param visible - if true, show it, else hide. 
     */
    void setColor(const std::string& name, const Eigen::Vector4f& color) override;
    void resetColor(const std::string& name);
    void addCustomWidget(QWidget* widget, const std::string& name);
    viewer::Box2DWorldView* getWorldViewer();

protected:
    void log(const std::string& msg, const std::string& prefix,
        Logger::LogLevel level = Logger::LogLevel::Info) const;
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
} // namespace sim_env

#endif //BOX2D_SIM_ENV_BOX2DWORLDVIEWER_H
