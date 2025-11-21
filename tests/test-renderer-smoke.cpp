/*
 * test-renderer-smoke.cpp - Minimal OSG renderer sanity test
 *
 * Renders a single colored quad, grabs a frame to PPM, and exits.
 */

#include <osg/ArgumentParser>
#include <osg/Camera>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/NodeCallback>
#include <osg/ShapeDrawable>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>

static osg::ref_ptr<osg::Geode>
create_colored_quad ()
{
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry ();

    osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array ();
    verts->push_back (osg::Vec3 (-1.0f, 0.0f, -1.0f));
    verts->push_back (osg::Vec3 ( 1.0f, 0.0f, -1.0f));
    verts->push_back (osg::Vec3 (-1.0f, 0.0f,  1.0f));
    verts->push_back (osg::Vec3 ( 1.0f, 0.0f,  1.0f));
    geom->setVertexArray (verts.get ());

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array ();
    colors->push_back (osg::Vec4 (1.0f, 0.0f, 1.0f, 1.0f)); // magenta
    colors->push_back (osg::Vec4 (0.0f, 1.0f, 0.0f, 1.0f)); // green
    colors->push_back (osg::Vec4 (0.0f, 0.0f, 1.0f, 1.0f)); // blue
    colors->push_back (osg::Vec4 (1.0f, 1.0f, 0.0f, 1.0f)); // yellow
    geom->setColorArray (colors.get (), osg::Array::BIND_PER_VERTEX);

    osg::ref_ptr<osg::DrawElementsUInt> indices =
        new osg::DrawElementsUInt (GL_TRIANGLE_STRIP);
    indices->push_back (0);
    indices->push_back (1);
    indices->push_back (2);
    indices->push_back (3);
    geom->addPrimitiveSet (indices.get ());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode ();
    geode->addDrawable (geom.get ());
    return geode;
}

int
main (int argc, char **argv)
{
    osg::ArgumentParser args (&argc, argv);
    int width = 640;
    int height = 480;
    args.read ("--width", width);
    args.read ("--height", height);

    osg::ref_ptr<osg::Geode> quad = create_colored_quad ();

    osg::ref_ptr<osg::MatrixTransform> root = new osg::MatrixTransform ();
    root->addChild (quad.get ());

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow (100, 100, width, height);
    viewer.setSceneData (root.get ());

    // Use an orthographic projection looking down -Y.
    osg::ref_ptr<osg::Camera> cam = viewer.getCamera ();
    cam->setClearColor (osg::Vec4 (0.0f, 0.0f, 0.0f, 1.0f));
    cam->setReferenceFrame (osg::Transform::ABSOLUTE_RF);
    cam->setViewMatrix (osg::Matrix::lookAt (
                            osg::Vec3 (0.0f, 4.0f, 0.0f), // eye
                            osg::Vec3 (0.0f, 0.0f, 0.0f), // center
                            osg::Vec3 (0.0f, 0.0f, 1.0f)  // up
                        ));
    cam->setProjectionMatrix (osg::Matrix::ortho (-2.0, 2.0, -2.0, 2.0, 1.0, 10.0));

    osg::ref_ptr<osg::Image> image = new osg::Image ();
    cam->attach (osg::Camera::COLOR_BUFFER, image.get ());

    viewer.realize ();
    viewer.frame (); // render one frame

    osgDB::writeImageFile (*image, "osg_renderer_smoke.ppm");

    return 0;
}
