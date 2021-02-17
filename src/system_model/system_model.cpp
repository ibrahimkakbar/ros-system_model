#include "system_model/system_model.hpp"

#include <system_model_msgs/variable.h>

#include <ros/console.h>
#include <ros/names.h>

#include <dlfcn.h>

using namespace system_model;

// CONSTRUCTORS
system_model_t::system_model_t(uint32_t n_variables, uint32_t n_observers)
    : ukf_t(n_variables, n_observers)
{
    // Initialize ROS node handle.
    system_model_t::m_node = std::make_unique<ros::NodeHandle>();

    // Read parameters.
    ros::NodeHandle private_node("~");
    
    // Set up default variable names.
    system_model_t::m_variable_names.reserve(n_variables);
    for(uint32_t i = 0; i < n_variables; ++i)
    {
        system_model_t::m_variable_names.push_back(std::to_string(i));
    }

    // Reserve space for state publishers.
    system_model_t::m_state_publishers.reserve(n_variables);

    // Set initial delta time.
    system_model_t::m_dt = 0;
}
std::shared_ptr<system_model_t> system_model_t::load_plugin(const std::string& plugin_path)
{
    // Check that path was provided (dl gets handle to program if empty)
    if(plugin_path.empty())
    {
        ROS_ERROR("attempted to load plugin with empty path");
        return nullptr;
    }

    // Open plugin shared object library.
    void* so_handle = dlopen(plugin_path.c_str(), RTLD_NOW);
    if(!so_handle)
    {
        ROS_ERROR_STREAM("failed to load model plugin (" << dlerror() << ")");
        return nullptr;
    }

    // Get a reference to the instantiate symbol.
    typedef system_model_t* (*instantiate_t)();
    instantiate_t instantiate = reinterpret_cast<instantiate_t>(dlsym(so_handle, "instantiate"));
    if(!instantiate)
    {
        ROS_ERROR_STREAM("failed to load model plugin (" << dlerror() << ")");
        dlclose(so_handle);
        return nullptr;
    }

    // Try to instantiate the plugin.
    system_model_t* plugin = nullptr;
    try
    {
        plugin = instantiate();
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("failed to instantiate model plugin (" << error.what() << ")");
        dlclose(so_handle);
        return nullptr;
    }

    // Return the plugin as a shared ptr with a custom deleter.
    return std::shared_ptr<system_model_t>(plugin,
                                           [so_handle](system_model_t* plugin){delete plugin; dlclose(so_handle);});
}

// METHODS
void system_model_t::set_variable_name(uint32_t index, const std::string& name)
{
    // Check if index is valid.
    if(!(index < system_model_t::m_variable_names.size()))
    {
        ROS_ERROR_STREAM("failed to set variable name (variable index " << index << " does not exist)");
        return;
    }

    // Check if name is valid as a ROS graph resource name.
    std::string error;
    if(!ros::names::validate(name, error))
    {
        ROS_ERROR_STREAM("failed to set variable name (variable name is invalid: " << error << ")");
        return;
    }

    // Update variable name.
    system_model_t::m_variable_names.at(index) = name;
}
void system_model_t::run()
{
    // Bring up state publishers.
    ros::NodeHandle private_node("~");
    for(uint32_t i = 0; i < system_model_t::n_variables(); ++i)
    {
        // Set up topic name.
        std::string topic_name = ros::names::remap(ros::names::append("state", system_model_t::m_variable_names.at(i)));
        // Publish topic.
        system_model_t::m_state_publishers.push_back(private_node.advertise<system_model_msgs::variable>(topic_name, 10));
    }

    // Set up the loop rate timer.
    double_t p_loop_rate = private_node.param<double_t>("loop_rate", 100);
    ros::Rate loop(p_loop_rate);

    // Initialize dt calculation.
    ros::Time update_timestamp = ros::Time::now();
    ros::Time current_timestamp;

    // Loop while ROS ok.
    while(ros::ok())
    {
        // Get current time and dt.
        current_timestamp = ros::Time::now();
        system_model_t::m_dt = (current_timestamp - update_timestamp).toSec();
        update_timestamp = current_timestamp;

        // Run callbacks to process observations.
        ros::spinOnce();

        // Run iteration.
        system_model_t::iterate();        

        // Publish current state.
        auto& state = system_model_t::state();
        auto& covariance = system_model_t::covariance();
        for(uint32_t i = 0; i < state.size(); ++i)
        {
            system_model_msgs::variable message;
            message.value = state(i);
            message.variance = covariance(i,i);
            system_model_t::m_state_publishers.at(i).publish(message);
        }

        // Sleep for remaining loop time.
        loop.sleep();
    }

    // Shutdown publishers.
    system_model_t::m_state_publishers.clear();
}
double_t system_model_t::dt() const
{
    return system_model_t::m_dt;
}