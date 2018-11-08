#include <sim_env/Box2DImageRenderer.h>
/////////////////////////////////////// Box2DImageRenderer ///////////////////////////////////////
sim_env::Box2DImageRenderer::Box2DImageRenderer(sim_env::Box2DWorldPtr world)
    : _world(world)
    , _camera_on_world(true)
{
    // compute extents of the world excluding any drawings
    // run over objects and get largest bounding box
    std::vector<sim_env::Box2DObjectPtr> objects;
    _world->getBox2DObjects(objects);
    sim_env::BoundingBox max_obj_size_box;
    for (auto object : objects) {
        auto laabb = object->getLocalAABB();
        max_obj_size_box.merge(laabb);
    }
    Eigen::Vector3f max_obj_size = max_obj_size_box.extents();
    float max_extent = std::max(max_obj_size[0], max_obj_size[1]);
    // compute bounding box for world
    Eigen::Vector4f world_bounds = _world->getWorldBounds();
    _world_region.min_corner[0] = world_bounds[0] - max_extent;
    _world_region.min_corner[1] = world_bounds[1] - max_extent;
    _world_region.max_corner[0] = world_bounds[2] + max_extent;
    _world_region.max_corner[1] = world_bounds[3] + max_extent;
    _drawing_region = _world_region;
    // set colors of robots
    std::vector<sim_env::RobotPtr> robots;
    _world->getRobots(robots);
    for (auto robot : robots) {
        setColor(robot->getName(), Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.0f));
    }
}

sim_env::Box2DImageRenderer::~Box2DImageRenderer() = default;

void sim_env::Box2DImageRenderer::centerCamera(bool include_drawings)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    _camera_on_world = not include_drawings;
}

void sim_env::Box2DImageRenderer::resetCamera()
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    _camera_on_world = true;
}

sim_env::WorldViewer::Handle sim_env::Box2DImageRenderer::drawBox(const Eigen::Vector3f& pos, const Eigen::Vector3f& extent,
    const Eigen::Vector4f& color, bool solid, float edge_width)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    sim_env::WorldViewer::Handle new_handle;
    Box new_box(pos, extent, color, solid, edge_width);
    _boxes.insert(std::make_pair(new_handle.getID(), new_box));
    _drawing_region.min_corner[0] = std::min(_drawing_region.min_corner[0], new_box.pos[0] - new_box.extent[0]);
    _drawing_region.min_corner[1] = std::min(_drawing_region.min_corner[1], new_box.pos[1] - new_box.extent[1]);
    _drawing_region.max_corner[0] = std::max(_drawing_region.max_corner[0], new_box.pos[0] + new_box.extent[0]);
    _drawing_region.max_corner[1] = std::max(_drawing_region.max_corner[1], new_box.pos[1] + new_box.extent[1]);
    return new_handle;
}

sim_env::WorldViewer::Handle sim_env::Box2DImageRenderer::drawFrame(const Eigen::Affine3f& transform, float length, float width)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    sim_env::WorldViewer::Handle new_handle;
    Frame new_frame(transform, length, width);
    _frames.insert(std::make_pair(new_handle.getID(), new_frame));
    extendBounds(new_frame.x_axis);
    extendBounds(new_frame.y_axis);
    return new_handle;
}

sim_env::WorldViewer::Handle sim_env::Box2DImageRenderer::drawLine(const Eigen::Vector3f& start,
    const Eigen::Vector3f& end,
    const Eigen::Vector4f& color,
    float width)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    sim_env::WorldViewer::Handle new_handle;
    Line new_line(start, end, color, width);
    _lines.insert(std::make_pair(new_handle.getID(), new_line));
    extendBounds(new_line);
    return new_handle;
}

sim_env::WorldViewer::Handle sim_env::Box2DImageRenderer::drawSphere(const Eigen::Vector3f& center, float radius,
    const Eigen::Vector4f& color,
    float width)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    sim_env::WorldViewer::Handle new_handle;
    Sphere new_sphere(center, color, radius, width);
    _spheres.insert(std::make_pair(new_handle.getID(), new_sphere));
    _drawing_region.min_corner[0] = std::min(_drawing_region.min_corner[0], center[0] - radius);
    _drawing_region.min_corner[1] = std::min(_drawing_region.min_corner[1], center[1] - radius);
    _drawing_region.max_corner[0] = std::max(_drawing_region.max_corner[0], center[0] + radius);
    _drawing_region.max_corner[1] = std::max(_drawing_region.max_corner[1], center[1] + radius);
    return new_handle;
}

sim_env::WorldViewer::Handle sim_env::Box2DImageRenderer::drawVoxelGrid(const grid::VoxelGrid<float, Eigen::Vector4f>& grid,
    const WorldViewer::Handle& old_handle)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    throw std::logic_error("[Box2DImageRenderer::drawVoxelGrid] This function is not implemented yet.");
}

bool sim_env::Box2DImageRenderer::renderImage(const std::string& filename, unsigned int width, unsigned int height,
    bool include_drawings)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    // 1. compute projection matrix
    sim_env::BoundingBox target_region = _drawing_region;
    if (_camera_on_world) {
        target_region = _world_region;
    }
    assert(target_region.getWidth() > 0);
    assert(target_region.getHeight() > 0);
    float scaling_factor = std::min(width / target_region.getWidth(), height / target_region.getHeight());
    Eigen::Affine2f to_image_tf;
    to_image_tf = Eigen::Scaling(scaling_factor) * Eigen::Translation<float, 2>(-target_region.min_corner[0], -target_region.min_corner[1]);
    // 2. create image
    cimg_library::CImg<unsigned char> image(width, height, 1, 3, 255);
    // 3. draw objects and robot
    renderObjects(image, to_image_tf);
    // 4. optionally draw additional drawings
    if (include_drawings) {
        renderSpheres(image, to_image_tf, scaling_factor);
        renderLines(image, to_image_tf);
        renderFrames(image, to_image_tf);
        renderBoxes(image, to_image_tf);
    }
    // 5. save image
    image.save(filename.c_str());
    return true;
}

void sim_env::Box2DImageRenderer::setVisible(const std::string& name, bool visible)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    auto iter = _object_drawing_info.find(name);
    if (iter == _object_drawing_info.end()) { // if not in map yet
        ObjectInfo info;
        info.visible = visible;
        _object_drawing_info.insert(std::make_pair(name, info));
    } else {
        iter->second.visible = visible;
    }
}

void sim_env::Box2DImageRenderer::setColor(const std::string& name, const Eigen::Vector4f& color)
{
    std::lock_guard<std::recursive_mutex> guard(_mutex);
    auto iter = _object_drawing_info.find(name);
    if (iter == _object_drawing_info.end()) { // if not in map yet
        ObjectInfo info;
        info.color[0] = (unsigned int)(255 * color[0]);
        info.color[1] = (unsigned int)(255 * color[1]);
        info.color[2] = (unsigned int)(255 * color[2]);
        info.opacity = color[3];
        _object_drawing_info.insert(std::make_pair(name, info));
    } else {
        iter->second.color[0] = (unsigned int)(255 * color[0]);
        iter->second.color[1] = (unsigned int)(255 * color[1]);
        iter->second.color[2] = (unsigned int)(255 * color[2]);
        iter->second.opacity = color[3];
    }
}

void sim_env::Box2DImageRenderer::removeDrawing(const WorldViewer::Handle& handle)
{
    { // spheres
        auto iter = _spheres.find(handle.getID());
        if (iter != _spheres.end()) {
            _spheres.erase(iter);
            return;
        }
    }
    { // boxes
        auto iter = _boxes.find(handle.getID());
        if (iter != _boxes.end()) {
            _boxes.erase(iter);
            return;
        }
    }
    { // lines
        auto iter = _lines.find(handle.getID());
        if (iter != _lines.end()) {
            _lines.erase(iter);
            return;
        }
    }
    { // frames
        auto iter = _frames.find(handle.getID());
        if (iter != _frames.end()) {
            _frames.erase(iter);
            return;
        }
    }
}

void sim_env::Box2DImageRenderer::removeAllDrawings()
{
    _spheres.clear();
    _lines.clear();
    _boxes.clear();
    _frames.clear();
}

void sim_env::Box2DImageRenderer::extendBounds(const Line& line)
{
    float minx = std::min(line.start[0], line.end[0]);
    float maxx = std::max(line.start[0], line.end[0]);
    float miny = std::min(line.start[1], line.end[1]);
    float maxy = std::max(line.start[1], line.end[1]);
    _drawing_region.min_corner[0] = std::min(_drawing_region.min_corner[0], minx);
    _drawing_region.min_corner[1] = std::min(_drawing_region.min_corner[1], miny);
    _drawing_region.max_corner[0] = std::max(_drawing_region.max_corner[0], maxx);
    _drawing_region.max_corner[1] = std::max(_drawing_region.max_corner[1], maxy);
}

void sim_env::Box2DImageRenderer::renderObjects(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const
{
    // first render objects
    std::vector<sim_env::Box2DObjectPtr> objects;
    _world->getBox2DObjects(objects);
    for (auto object : objects) {
        renderObject(cimg, to_image_frame, object);
    }
    // then render robots
    std::vector<sim_env::Box2DRobotPtr> robots;
    _world->getBox2DRobots(robots);
    for (auto robot : robots) {
        renderObject(cimg, to_image_frame, robot->getBox2DObject());
    }
}

// render all spheres on the given image
void sim_env::Box2DImageRenderer::renderSpheres(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, float scaling_factor) const
{
    for (auto& item : _spheres) {
        renderSphere(cimg, to_image_frame, item.second, scaling_factor);
    }
}
// render all lines on the given image
void sim_env::Box2DImageRenderer::renderLines(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const
{
    for (auto& item : _lines) {
        renderLine(cimg, to_image_frame, item.second);
    }
}
// render all frames on the given image
void sim_env::Box2DImageRenderer::renderFrames(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const
{
    for (auto& item : _frames) {
        renderFrame(cimg, to_image_frame, item.second);
    }
}
// render all boxes on the given image
void sim_env::Box2DImageRenderer::renderBoxes(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame) const
{
    for (auto& item : _boxes) {
        renderBox(cimg, to_image_frame, item.second);
    }
}

// render individual components
void sim_env::Box2DImageRenderer::renderSphere(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Sphere& sphere, float scaling_factor) const
{
    Eigen::Vector2f center = to_image_frame * sphere.center;
    int radius = (int)(scaling_factor * sphere.radius);
    cimg.draw_circle((int)center[0], cimg.height() - (int)center[1], radius, sphere.color, sphere.opacity);
}

void sim_env::Box2DImageRenderer::renderLine(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Line& line) const
{
    Eigen::Vector2f start = to_image_frame * line.start;
    Eigen::Vector2f end = to_image_frame * line.end;
    cimg.draw_line((int)start[0], cimg.height() - (int)start[1], (int)end[0], cimg.height() - (int)end[1], line.color, line.opacity);
}

void sim_env::Box2DImageRenderer::renderFrame(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Frame& frame) const
{
    renderLine(cimg, to_image_frame, frame.x_axis);
    renderLine(cimg, to_image_frame, frame.y_axis);
}

void sim_env::Box2DImageRenderer::renderBox(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, const Box& box) const
{
    Eigen::Vector2f min_corner = box.pos - box.extent;
    min_corner = to_image_frame * min_corner;
    Eigen::Vector2f max_corner = box.pos + box.extent;
    max_corner = to_image_frame * max_corner;
    if (!box.solid) {
        // TODO figure out what pattern to use
        cimg.draw_rectangle((int)min_corner[0], cimg.height() - (int)min_corner[1], (int)max_corner[0],
            cimg.height() - (int)max_corner[1], box.color, box.opacity, 0xFFFFFFFF);
    } else {
        cimg.draw_rectangle((int)min_corner[0], cimg.height() - (int)min_corner[1], (int)max_corner[0],
            cimg.height() - (int)max_corner[1], box.color, box.opacity);
    }
}

void sim_env::Box2DImageRenderer::renderObject(cimg_library::CImg<unsigned char>& cimg, const Eigen::Affine2f& to_image_frame, Box2DObjectPtr object) const
{
    std::vector<sim_env::Box2DLinkPtr> links;
    object->getBox2DLinks(links);
    auto info_iter = _object_drawing_info.find(object->getName());
    ObjectInfo drawing_info;
    if (info_iter != _object_drawing_info.end()) {
        drawing_info = info_iter->second;
    } // else use default info
    if (!drawing_info.visible) {
        return; // do not draw invisible objects
    }
    // run over all links and render them
    Eigen::Vector2f lpos;
    for (auto link : links) {
        // first prepare transformation matrix to world frame
        Eigen::Vector3f pose = link->getPose();
        lpos[0] = pose[0];
        lpos[1] = pose[1];
        Eigen::Affine2f to_world_frame;
        to_world_frame = Eigen::Translation<float, 2>(lpos) * Eigen::Rotation2Df(pose[2]);
        // now get geometry for this link
        std::vector<std::vector<Eigen::Vector2f>> local_geometry;
        link->getGeometry(local_geometry);
        // draw each polygon
        for (auto& polygon : local_geometry) {
            cimg_library::CImg<int> cimg_polygon(polygon.size(), 2);
            // translate polygon into CImg data structure
            for (unsigned int i = 0; i < polygon.size(); ++i) {
                Eigen::Vector2f tp = to_image_frame * to_world_frame * polygon[i];
                cimg_polygon(i, 0) = tp[0];
                cimg_polygon(i, 1) = cimg.height() - tp[1];
            }
            cimg.draw_polygon(cimg_polygon, drawing_info.color, drawing_info.opacity);
        }
    }
}