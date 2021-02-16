#include <system_model/system_model.hpp>

int32_t main(int32_t argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "system_model");

    // Get name of plugin.
    ros::NodeHandle private_node("~");
    std::string p_plugin_name = private_node.param<std::string>("plugin_path", "");

    // Load plugin.
    std::shared_ptr<system_model::system_model_t> system_model = system_model::system_model_t::load_plugin(p_plugin_name);

    // Check if plugin was loaded properly.
    if(!system_model)
    {
        ROS_FATAL_STREAM("failed to start system model");
        return 1;
    }

    // Run plugin.
    try
    {
        system_model->run();
    }
    catch(const std::exception& error)
    {
        ROS_FATAL_STREAM("run failed (" << error.what() << ")");
        return 1;
    }
    

    return 0;
}