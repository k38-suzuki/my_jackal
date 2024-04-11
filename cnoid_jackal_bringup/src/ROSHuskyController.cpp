/**
   ROS Husky Controller
   @author Kenta Suzuki
*/

#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

using namespace cnoid;

class ROSHuskyController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    geometry_msgs::Twist latestTwistState;
    std::mutex twistMutex;
    
    Link* wheel[4];

public:

    virtual bool configure(SimpleControllerConfig* config) override
    {
        node.reset(new ros::NodeHandle);
        return true;
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        std::ostream& os = io->os();
        Body* body = io->body();

        wheel[0] = body->link("FRONT_LEFT");
        wheel[1] = body->link("FRONT_RIGHT");
        wheel[2] = body->link("REAR_LEFT");
        wheel[3] = body->link("REAR_RIGHT");

        for(int i = 0; i < 4; ++i) {
            Link* joint = wheel[i];
            if(!joint) {
                os << "Wheel " << i << " is not found." << std::endl;
                return false;
            }
            joint->setActuationMode(Link::JointVelocity);
            io->enableIO(joint);
        }

        subscriber = node->subscribe("cmd_vel", 100, &ROSHuskyController::twistCallback, this);

        return true;
    }

    void twistCallback(const geometry_msgs::Twist& msg)
    {
        std::lock_guard<std::mutex> lock(twistMutex);
        latestTwistState = msg;
    }

    virtual bool control() override
    {
        geometry_msgs::Twist twist;
        {
            std::lock_guard<std::mutex> lock(twistMutex);
            twist = latestTwistState;
        }

        double pos[2];
        pos[0] = -1.0 * twist.linear.x / 0.1651;
        pos[1] = -1.0 * twist.angular.z / 0.1651 * 0.670 / 2.0;

        for(int i = 0; i < 2; ++i) {
            Link* wheelL = wheel[2 * i];
            Link* wheelR = wheel[2 * i + 1];
            wheelL->dq_target() = pos[0] - pos[1];
            wheelR->dq_target() = pos[0] + pos[1];
        }       

        return true;
    }

    virtual void stop() override
    {
        subscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ROSHuskyController)
