/// \file system_model/system_model.hpp
/// \brief Defines the system_model::system_model_t class.
#ifndef SYSTEM_MODEL___SYSTEM_MODEL_H
#define SYSTEM_MODEL___SYSTEM_MODEL_H

#include <kalman_filter/ukf.hpp>

#include <ros/node_handle.h>

#include <memory>

/// \brief Contains objects for implementing system models.
namespace system_model {

/// \brief A pure abstract base class for all system_model plugins.
class system_model_t
    : protected kalman_filter::ukf_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new system_model.
    /// \param n_variables The number of variables in the model's state.
    /// \param n_observers The number of observers of the model's state.
    system_model_t(uint32_t n_variables, uint32_t n_observers);
    /// \brief Loads a system_model plugin.
    /// \param plugin_path The path to the plugin's *.so library file.
    /// \returns A shared pointer to the loaded system model plugin. Returns NULLPTR if load failed.
    static std::shared_ptr<system_model_t> load_plugin(const std::string& plugin_path);
    
    // METHODS
    /// \brief Begins the model's update loop.
    void run();

protected:
    // VARIABLES
    /// \brief A unique pointer to the node's public handle.
    /// \note Use this node handle for subscribing to messages from sensors/observers.
    std::unique_ptr<ros::NodeHandle> m_node;

    // METHODS
    /// \brief Sets the name of a variable in the state.
    /// \param index The index of the state variable to name.
    /// \param name The name to give the specified variable (must be ROS graph name compliant).
    /// \note Variable name changes only take effect at a new call of run().
    void set_variable_name(uint32_t index, const std::string& name);
    /// \brief Gets the number of seconds that have elapsed since the last model update.
    /// \returns The elapsed seconds since the last model update.
    double_t dt() const;

private:
    // VARIABLES
    /// \brief The current number of seconds that have elapsed since the last model update.
    double_t m_dt;
    /// \brief The names of each variable in the state.
    std::vector<std::string> m_variable_names;
    /// \brief The collection of publishers for individual variable messages.
    std::vector<ros::Publisher> m_state_publishers;
};

#define REGISTER_PLUGIN(class_name) extern "C" class_name* instantiate() {return new class_name();}

}

#endif