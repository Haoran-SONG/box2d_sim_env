//
// Created by joshua on 6/26/17.
//

#ifndef BOX2D_SIM_ENV_BOX2DIMAGERENDERER_H
#define BOX2D_SIM_ENV_BOX2DIMAGERENDERER_H

#include <sim_env/Box2DWorld.h>
// CImg
#define cimg_display 0 // disable CImg GUI support
#define cimg_use_png 1 // enable png support
#define cimg_use_jpeg 1 // enable jpeg support
#include <CImg.h>

namespace sim_env {
class Box2DImageRenderer : public sim_env::WorldViewer::ImageRenderer {
    /**
     * A Box2DImageRenderer allows you to render images showing a Box2DWorld. Use this if
     * you want to save images of your scene to disk or use an image of the scene for anything else
     * other than showing it on the screen. A Box2DImageRenderer is thread safe. If you intend to render
     * images in parallel, however, it is a good idea to generate multiple Box2DImageRenderer.
     * Every operation requires the executing thread to acquire a lock, so access by multiple threads is serialized.
     * 
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
    WorldViewer::Handle drawBox(const Eigen::Vector3f& pos, const Eigen::Vector3f& extent,
        const Eigen::Vector4f& color = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f),
        bool solid = false, float edge_width = 0.1f) override;
    /**
     * Draw a coordinate frame.
     * @param transform - the transformation matrix from the target frame to world frame
     * @param length - length of arrows
     * @param widt - width of arrows.
     */
    WorldViewer::Handle drawFrame(const Eigen::Affine3f& transform, float length = 1.0f, float width = 0.1f) override;
    /**
         * Draws a line from position start to position end.
         * @param start  - position where the line segment should start
         * @param end  - position where the line segment should end
         * @param color - rgba color (in range [0,1]^4)
         * @param width - width of the line
         * @return handle to delete the line again
         */
    WorldViewer::Handle drawLine(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
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
    WorldViewer::Handle drawSphere(const Eigen::Vector3f& center, float radius,
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
    WorldViewer::Handle drawVoxelGrid(const grid::VoxelGrid<float, Eigen::Vector4f>& grid, const WorldViewer::Handle& old_handle = WorldViewer::Handle(false)) override;

    /**
     *  Render the scene from the current camera view to an image and store this image under the given name.
     *  The underlying implementation guarantees that this method is thread-safe.
     *  The image format is chosen based on the end of filename. png and jpeg are supported.
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

    /**
     * Remove the drawing with the given handle.
     * If there is no drawing for that handle, this is a no-op.
     */
    virtual void removeDrawing(const WorldViewer::Handle& handle) override;

    /**
     * Remove all drawings.
     */
    virtual void removeAllDrawings() override;

protected:
    struct ObjectInfo {
        bool visible;
        unsigned char color[3];
        float opacity;
        ObjectInfo()
            : visible(true)
            , color{ 0, 0, 255 }
            , opacity(1.0f)
        {
        }
    };

    struct Line {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector2f start;
        Eigen::Vector2f end;
        unsigned char color[3];
        float opacity;
        float width; // TODO not supported at the moment
        Line()
            : color{ 255, 255, 255 }
            , opacity(1.0f)
            , width(0.0f)
        {
            start.setZero();
            end.setZero();
        }
        Line(const Eigen::Vector3f& istart, const Eigen::Vector3f& iend, const Eigen::Vector4f& icolor, float iwidth)
        {
            start[0] = istart[0];
            start[1] = istart[1];
            end[0] = iend[0];
            end[1] = iend[1];
            color[0] = (unsigned char)(255 * icolor[0]);
            color[1] = (unsigned char)(255 * icolor[1]);
            color[2] = (unsigned char)(255 * icolor[2]);
            opacity = icolor[3];
            width = iwidth;
        }
    };

    struct Sphere {
        Eigen::Vector2f center;
        float radius;
        unsigned char color[3];
        float opacity;
        float width; // TODO not supported at the moment
        Sphere()
            : radius(0.0f)
            , color{ 255, 255, 255 }
            , opacity(1.0f)
            , width(0.0f)
        {
            center.setZero();
        }
        Sphere(const Eigen::Vector3f& icenter, const Eigen::Vector4f& icolor, float r, float w)
        {
            center[0] = icenter[0];
            center[1] = icenter[1];
            color[0] = (unsigned char)(255 * icolor[0]);
            color[1] = (unsigned char)(255 * icolor[1]);
            color[2] = (unsigned char)(255 * icolor[2]);
            opacity = icolor[3];
            width = w;
            radius = r;
        }
    };

    struct Box {
        Eigen::Vector2f pos;
        Eigen::Vector2f extent;
        unsigned char color[3];
        float opacity;
        float edge_width; // TODO not supported at the moment
        bool solid;
        Box(const Eigen::Vector3f& ipos, const Eigen::Vector3f& iextent, const Eigen::Vector4f& icolor, bool isolid, float ew)
        {
            pos[0] = ipos[0];
            pos[1] = ipos[1];
            extent[0] = iextent[0];
            extent[1] = iextent[1];
            color[0] = (unsigned char)(255 * icolor[0]);
            color[1] = (unsigned char)(255 * icolor[1]);
            color[2] = (unsigned char)(255 * icolor[2]);
            opacity = icolor[3];
            edge_width = ew;
            solid = isolid;
        }
        Box()
            : color{}
            , opacity(1.0f)
            , edge_width(0.0f)
            , solid(true)
        {
            pos.setZero();
            extent.setZero();
        }
    };

    struct Frame {
        Line x_axis;
        Line y_axis;
        Frame() {}
        Frame(const Eigen::Affine3f& tf, float length, float width)
        {
            Eigen::Matrix3f rotation_matrix = tf.rotation();
            auto translation = tf.translation();
            // start of both axis
            x_axis.start[0] = translation(0);
            x_axis.start[1] = translation(1);
            y_axis.start[0] = translation(0);
            y_axis.start[1] = translation(1);
            // end of both axis
            Eigen::Vector3f xdir = rotation_matrix * Eigen::Vector3f::UnitX();
            Eigen::Vector3f ydir = rotation_matrix * Eigen::Vector3f::UnitY();
            x_axis.end[0] = x_axis.start[0] + length * xdir[0];
            x_axis.end[1] = x_axis.start[1] + length * xdir[1];
            y_axis.end[0] = y_axis.start[0] + length * ydir[0];
            y_axis.end[1] = y_axis.start[1] + length * ydir[1];
            // set width and colors
            x_axis.width = width;
            y_axis.width = width;
            x_axis.color[0] = 255; // red for x
            x_axis.color[1] = 0;
            x_axis.color[2] = 0;
            y_axis.color[0] = 0; // green for y
            y_axis.color[1] = 255;
            y_axis.color[2] = 0;
        }
    };

private:
    Box2DWorldPtr _world;
    // maps obj_name to color
    std::map<std::string, ObjectInfo> _object_drawing_info;
    // maps handle id to sphere
    std::map<unsigned int, Sphere> _spheres;
    // maps handle id to line
    std::map<unsigned int, Line> _lines;
    // maps handle id to frame
    std::map<unsigned int, Frame> _frames;
    // maps handle id to box
    std::map<unsigned int, Box> _boxes;
    bool _camera_on_world; // if true, focus camera only on world, else ensure all drawings are rendered
    sim_env::BoundingBox _world_region; // region excluding drawings
    sim_env::BoundingBox _drawing_region; // region including drawings
    std::recursive_mutex _mutex;

    // extend drawing bounds to contain line
    void extendBounds(const Line& line);
    // render all objects and robots from Box2D world on the given image
    void renderObjects(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const;
    // render all spheres on the given image
    void renderSpheres(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, float scaling_factor) const;
    // render all lines on the given image
    void renderLines(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const;
    // render all frames on the given image
    void renderFrames(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const;
    // render all boxes on the given image
    void renderBoxes(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const;
    // render individual components
    void renderSphere(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Sphere& sphere, float scaling_factor) const;
    void renderLine(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Line& line) const;
    void renderFrame(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Frame& frame) const;
    void renderBox(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Box& box) const;
    void renderObject(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, Box2DObjectPtr object) const;
};
} // namespace sim_env

#endif