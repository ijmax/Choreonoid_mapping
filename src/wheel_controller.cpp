#include <cnoid/SimpleController>
#include <cnoid/Joystick>
#include <iostream>

using namespace std;
using namespace cnoid;

class AW_Controller : public SimpleController
{
    Link* trackL;
    Link* trackR;
    Joystick joystick;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        trackL = io->body()->link("L_WHEEL");
        trackR = io->body()->link("R_WHEEL");

        io->enableOutput(trackL, JointVelocity);
        io->enableOutput(trackR, JointVelocity);

        return true;
    }

    virtual bool control() override
    {

        static const int axisID[] = { 2, 3 };

        joystick.readCurrentState();

        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(axisID[i]);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        double k = 2.0;
        trackL->dq_target() = k * (-2.0 * pos[1] + pos[0]);
        trackR->dq_target() = k * (-2.0 * pos[1] - pos[0]);

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AW_Controller)
