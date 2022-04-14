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
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cnoid;

class CameraController : public SimpleController
{
    RangeSensor* vlp;
    Joystick joystick;
    bool prevButtonState;
    int counter;
    std::ostream* os;
    BodyPtr ioBody;

public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        vlp = io->body()->findDevice<RangeSensor>("VLP-16");
        
        io->enableInput(vlp);
        io->enableInput(vlp->link(), LINK_POSITION);
        io->enableInput(io->body()->rootLink(), LINK_POSITION);
        prevButtonState = false;
        os = &io->os();
        ioBody = io->body();
        counter = 0;

        return true;
    }
    
    void constPCD(vector<double> const &src, vector<Vector3f>* pcld)
    {       
        const int numPitchSamples = vlp->numPitchSamples();
        const double pitchStep = vlp->pitchStep();
        const int numYawSamples = vlp->numYawSamples();
        const double yawStep = vlp->yawStep();
        
        for(int pitch=0; pitch < numPitchSamples; ++pitch){
            const double pitchAngle = pitch * pitchStep - vlp->pitchRange() / 2.0;
            const double cosPitchAngle = cos(pitchAngle);
            const int srctop = pitch * numYawSamples;
            
            for(int yaw=0; yaw < numYawSamples; ++yaw){
                const double distance = src[srctop + yaw];
                if(distance <= vlp->maxDistance()){
                    double yawAngle = yaw * yawStep - vlp->yawRange() / 2.0;
                    float x = distance *  cosPitchAngle * sin(-yawAngle);
                    float y  = distance * sin(pitchAngle);
                    float z  = -distance * cosPitchAngle * cos(yawAngle);
                    pcld->push_back(Vector3f(x, y, z));
                }
            }
        }
    }

    void savePCD()
    {
        const RangeSensor::RangeData& src = vlp->constRangeData();

        vector<Vector3f> pcld; 
        constPCD(src, &pcld);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width    = pcld.size();
        cloud.height   = 1;
        cloud.points.resize(cloud.width * cloud.height);

        size_t i = 0;
        for(const auto& e: pcld) {
            if (e[1]>0.1)
            {
                cloud[i].x = e(0);
                cloud[i].y = e(1);
                cloud[i].z = e(2);
                ++i;
            }
        }

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        Position pos = vlp->link()->position();
        const Vector3 t = pos.translation();

        Vector3 r = rpyFromRot(pos.rotation());
        transform.translation() << -(t.x()-sin(r[2])*0.24), -(t.y()+cos(r[2])*0.24), 0;
        transform.rotate (Eigen::AngleAxisf (r[2], Eigen::Vector3f::UnitY()));

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud (cloud, transformed_cloud, transform);

        pcl::io::savePCDFileBinaryCompressed ("vlp/cloud" + to_string(counter) + ".pcd", transformed_cloud);
        (*os) << "Saved a pcd file" << std::endl;
    }

    virtual bool control()
    {
        joystick.readCurrentState();
        bool currentState = joystick.getButtonState(0);
        if(currentState && !prevButtonState){            
            savePCD();
            counter++;
        }
        prevButtonState = currentState;
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)
