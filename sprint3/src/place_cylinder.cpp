#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cstdlib>

class PlaceCylinderNode : public rclcpp::Node
{
public:
    PlaceCylinderNode() : Node("place_cylinder")
    {
        // Declare parameters for x and y coordinates
        this->declare_parameter<double>("x", 0.0);
        this->declare_parameter<double>("y", 0.0);

        // Spawn the cylinder
        spawnCylinder();
    }

private:
    void spawnCylinder()
    {
        // Retrieve x and y coordinates from the parameters
        double x, y;
        this->get_parameter("x", x);
        this->get_parameter("y", y);
        double z = 0.0; // Always 0 for z

        // Create a client to call the /spawn_entity service in Gazebo
        auto client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the /spawn_entity service to be available...");
        }

        // Create a request for spawning the cylinder
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "cylinder";
        request->xml = R"(
            <sdf version="1.6">
                <model name="cylinder">
                    <pose>0 0 0 0 0 0</pose>
                    <link name="link">
                        <inertial>
                            <mass>1.0</mass>
                        </inertial>
                        <collision name="collision">
                            <geometry>
                                <cylinder>
                                    <radius>0.15</radius> <!-- 30 cm diameter -->
                                    <length>1.0</length>  <!-- 1 m tall -->
                                </cylinder>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                                <cylinder>
                                    <radius>0.15</radius>
                                    <length>1.0</length>
                                </cylinder>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>
        )";

        // Set the pose for the cylinder (x, y, z)
        request->initial_pose.position.x = x;
        request->initial_pose.position.y = y;
        request->initial_pose.position.z = z;
        request->initial_pose.orientation.w = 1.0;

        // Call the service and wait for response
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Cylinder placed at (%f, %f, %f)", x, y, z);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to place the cylinder");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the node and set x and y coordinates as arguments
    auto node = std::make_shared<PlaceCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
