#include "basic_function.hpp"

void uavos::printNavigationPose(const common_msgs::NavigationState& navigation_state, int level_of_detail){
    ros::Duration time_since_update = ros::Time::now() - navigation_state.header.stamp;

    double roll, pitch, yaw;
    quaternion2RPY(navigation_state.pose.orientation, roll, pitch, yaw);

    if(level_of_detail==0){
        std::printf("NWU Pose: [%8.2f]   x:%-8.2f y:%-8.2f z:%-8.2f c:%-8.2f\n",
                    time_since_update.toSec(),
                    navigation_state.pose.position.x,
                    navigation_state.pose.position.y,
                    navigation_state.pose.position.z,
                    yaw);
    } else if(level_of_detail==1){
        std::printf("NWU Pose: [%6.2f]   x:%-8.2f y:%-8.2f z:%-8.2f c:%-8.2f vx:%-8.2f vy:%-8.2f vz:%-8.2f vc:%-8.2f\n",
                    time_since_update.toSec(),
                    navigation_state.pose.position.x,
                    navigation_state.pose.position.y,
                    navigation_state.pose.position.z,
                    yaw,
                    navigation_state.velocity.linear.x,
                    navigation_state.velocity.linear.y,
                    navigation_state.velocity.linear.z,
                    navigation_state.velocity.angular.z);
    } else{
        std::printf("NWU Pose: [%6.2f]   x:%-8.2f y:%-8.2f z:%-8.2f c:%-8.2f vx:%-8.2f vy:%-8.2f vz:%-8.2f vc:%-8.2f ax:%-8.2f ay:%-8.2f az:%-8.2f ac:%-8.2f\n",
                    time_since_update.toSec(),
                    navigation_state.pose.position.x,
                    navigation_state.pose.position.y,
                    navigation_state.pose.position.z,
                    yaw,
                    navigation_state.velocity.linear.x,
                    navigation_state.velocity.linear.y,
                    navigation_state.velocity.linear.z,
                    navigation_state.velocity.angular.z,
                    navigation_state.acceleration.linear.x,
                    navigation_state.acceleration.linear.y,
                    navigation_state.acceleration.linear.z,
                    navigation_state.acceleration.angular.z);
    }

}



void uavos::quaternion2RPY(const geometry_msgs::Quaternion q, double &roll, double &pitch, double &yaw){
    tf::Quaternion q_tf(q.x, q.y, q.z, q.w);
    quaternion2RPY(q_tf, roll, pitch, yaw);
}

void uavos::quaternion2RPY(const tf::Quaternion q, double &roll, double &pitch, double &yaw){
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    yaw = INPI<double>(yaw);
    pitch = INPI<double>(pitch);
    roll = INPI<double>(roll);
}
void uavos::navigation2RPY(const common_msgs::NavigationState state, double &roll, double &pitch, double &yaw){
    quaternion2RPY(state.pose.orientation, roll, pitch, yaw);
}


// vector to NavigationState
common_msgs::NavigationState uavos::vector2Navigation(double x, double y, double z, double c,
                                               double vx, double vy, double vz, double vc,
                                               double ax, double ay, double az, double ac){
    common_msgs::NavigationState navigation_state;

    navigation_state.pose.position.x = x;
    navigation_state.pose.position.y = y;
    navigation_state.pose.position.z = z;
    navigation_state.pose.orientation = tf::createQuaternionMsgFromYaw(INPI<double>(c));

    navigation_state.velocity.linear.x = vx;
    navigation_state.velocity.linear.y = vy;
    navigation_state.velocity.linear.z = vz;
    navigation_state.velocity.angular.x = 0;
    navigation_state.velocity.angular.y = 0;
    navigation_state.velocity.angular.z = vc;

    navigation_state.acceleration.linear.x = ax;
    navigation_state.acceleration.linear.y = ay;
    navigation_state.acceleration.linear.z = az;
    navigation_state.acceleration.angular.x = 0;
    navigation_state.acceleration.angular.y = 0;
    navigation_state.acceleration.angular.z = ac;

    return navigation_state;
}

common_msgs::NavigationState uavos::navigationNWU2NED(common_msgs::NavigationState state_NWU){
    common_msgs::NavigationState state_NED;

    state_NED.pose.position.x = state_NWU.pose.position.x;
    state_NED.pose.position.y = state_NWU.pose.position.y * -1;
    state_NED.pose.position.z = state_NWU.pose.position.z * -1;
    state_NED.pose.orientation.x = state_NWU.pose.orientation.x;
    state_NED.pose.orientation.y = state_NWU.pose.orientation.y * -1;
    state_NED.pose.orientation.z = state_NWU.pose.orientation.z * -1;
    state_NED.pose.orientation.w = state_NWU.pose.orientation.w;

    state_NED.velocity.linear.x = state_NWU.velocity.linear.x;
    state_NED.velocity.linear.y = state_NWU.velocity.linear.y * -1;
    state_NED.velocity.linear.z = state_NWU.velocity.linear.z * -1;
    state_NED.velocity.angular.x = state_NWU.velocity.angular.x;
    state_NED.velocity.angular.y = state_NWU.velocity.angular.y * -1;
    state_NED.velocity.angular.z = state_NWU.velocity.angular.z * -1;

    state_NED.acceleration.linear.x = state_NWU.acceleration.linear.x;
    state_NED.acceleration.linear.y = state_NWU.acceleration.linear.y * -1;
    state_NED.acceleration.linear.z = state_NWU.acceleration.linear.z * -1;
    state_NED.acceleration.angular.x = state_NWU.acceleration.angular.x;
    state_NED.acceleration.angular.y = state_NWU.acceleration.angular.y * -1;
    state_NED.acceleration.angular.z = state_NWU.acceleration.angular.z * -1;

    return state_NED;
}

common_msgs::NavigationState uavos::navigationNED2NWU(common_msgs::NavigationState state_NED){
    return navigationNWU2NED(state_NED);
}

common_msgs::Reference uavos::referenceNWU2NED(common_msgs::Reference ref_NWU){
    common_msgs::Reference ref_NED = ref_NWU;
    ref_NED.navigation = navigationNWU2NED(ref_NWU.navigation);

    return ref_NED;
}

common_msgs::Reference uavos::referenceNED2NWU(common_msgs::Reference ref_NED){
    return referenceNWU2NED(ref_NED);
}
