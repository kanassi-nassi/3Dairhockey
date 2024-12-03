#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bouncing_object_control");
    ros::NodeHandle nh;
    ros::service::waitForService("/gazebo/set_model_state");
    ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient get_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    ros::Publisher cable_length_pub = nh.advertise<std_msgs::Float64MultiArray>("cable_lengths", 10);

    gazebo_msgs::ModelState model_state;
    model_state.model_name = "bouncing_object";
    model_state.pose.position.x = 1.0;
    model_state.pose.position.y = 1.0;
    model_state.pose.position.z = 1.0;

    // 初期速度を設定 (30度方向、斜め下)
    double angle = M_PI / 6; // 30度
    double speed = 0.5; // 速度を速くする
    double velocity_x = speed * cos(angle);
    double velocity_y = speed * sin(angle);
    double velocity_z = -speed * sin(angle);

    // 固定点のモデル名
    std::vector<std::string> fixed_point_names = {
        "fixed_point1", "fixed_point2", "fixed_point3", "fixed_point4",
        "fixed_point5", "fixed_point6", "fixed_point7", "fixed_point8"
    };

    ros::Rate rate(10);

    while (ros::ok())
    {
        // 位置を更新
        model_state.pose.position.x += velocity_x;
        model_state.pose.position.y += velocity_y;
        model_state.pose.position.z += velocity_z;

        // 壁に衝突したら跳ね返る
        if (abs(model_state.pose.position.x) > 3.4)
        {
            velocity_x = -velocity_x;
        }
        if (abs(model_state.pose.position.y) > 3.4)
        {
            velocity_y = -velocity_y;
        }
        if (model_state.pose.position.z > 7.4 || model_state.pose.position.z < 0.6)
        {
            velocity_z = -velocity_z;
        }

        // ケーブルの長さを計算
        std_msgs::Float64MultiArray cable_lengths;
        cable_lengths.data.clear();
        for (const auto& point_name : fixed_point_names)
        {
            gazebo_msgs::GetModelState get_model_state;
            get_model_state.request.model_name = point_name;
            if (get_state_client.call(get_model_state))
            {
                double length = std::sqrt(
                    std::pow(get_model_state.response.pose.position.x - model_state.pose.position.x, 2) +
                    std::pow(get_model_state.response.pose.position.y - model_state.pose.position.y, 2) +
                    std::pow(get_model_state.response.pose.position.z - model_state.pose.position.z, 2)
                );
                cable_lengths.data.push_back(length);
            }
            else
            {
                ROS_ERROR("Failed to call service get_model_state for %s", point_name.c_str());
            }
        }

        // ケーブルの長さをトピックとして出力
        cable_length_pub.publish(cable_lengths);

        // ケーブルの長さをコンソールに表示
        ROS_INFO("Cable lengths: [%f, %f, %f, %f, %f, %f, %f, %f]",
                 cable_lengths.data[0], cable_lengths.data[1], cable_lengths.data[2], cable_lengths.data[3],
                 cable_lengths.data[4], cable_lengths.data[5], cable_lengths.data[6], cable_lengths.data[7]);

        gazebo_msgs::SetModelState set_model_state;
        set_model_state.request.model_state = model_state;
        if (!set_state_client.call(set_model_state))
        {
            ROS_ERROR("Failed to call service set_model_state");
            return 1;
        }

        rate.sleep();
    }

    return 0;
}