#include <cnoid/SimpleController>
#include <cnoid/RangeSensor>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>
#include <cnoid/EigenUtil>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cnoid;

class CameraController : public SimpleController
{
    RangeCamera* knt;
    Joystick joystick;
    bool prevButtonState;
    int counter;
    std::ostream* os;
    BodyPtr ioBody;

public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        knt = io->body()->findDevice<RangeCamera>("Kinect");
        
        io->enableInput(knt);
        io->enableInput(knt->link(), LINK_POSITION);
        io->enableInput(io->body()->rootLink(), LINK_POSITION);
        prevButtonState = false;
        os = &io->os();
        ioBody = io->body();
        counter = 0;

        return true;
    }

    void savePCD()
    {
        const Image& imgData = knt->constImage();
        const unsigned char* pixels = imgData.pixels();

        const int width  = imgData.width();
        const int height = imgData.height();

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.width    = width;
        cloud.height   = height;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        size_t i = 0;
        size_t ci = 0;
        for(const auto& e: knt->constPoints()) {
            if (e[1]<2 and e[1]>-0.5)
            {
                cloud[i].x = e(0);
                cloud[i].y = e(1);
                cloud[i].z = e(2);
                cloud[i].r = pixels[3*ci + 0];
                cloud[i].g = pixels[3*ci + 1];
                cloud[i].b = pixels[3*ci + 2];
                ++i;
            }
            ++ci;
        }

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        Position pos = ioBody->rootLink()->position();
        const Vector3 t = -pos.translation();
        Vector3 r = rpyFromRot(pos.rotation());
        transform.translation() << t.y(), t.z(), t.x();
        transform.rotate (Eigen::AngleAxisf (r[2], Eigen::Vector3f::UnitY()));

        pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
        pcl::transformPointCloud (cloud, transformed_cloud, transform);

        pcl::io::savePCDFileBinaryCompressed ("kinect/cloud" + to_string(counter) + ".pcd", transformed_cloud);
        (*os) << "Saved a pcd file" << std::endl;
    }

    virtual bool control()
    {
        joystick.readCurrentState();
        bool currentState = joystick.getButtonState(1);
        if(currentState && !prevButtonState){            
            savePCD();
            counter++;
        }
        prevButtonState = currentState;
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)
