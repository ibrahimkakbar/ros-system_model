#ifndef SYSTEM_MODEL___SYSTEM_MODEL_H
#define SYSTEM_MODEL___SYSTEM_MODEL_H

#include <kalman_filter/ukf.hpp>

#include <ros/node_handle.h>

#include <memory>

namespace system_model {

class system_model_t
    : protected kalman_filter::ukf_t
{
public:
    // CONSTRUCTORS
    system_model_t(uint32_t n_variables, uint32_t n_observers);
    static std::shared_ptr<system_model_t> load_plugin(const std::string& plugin_path);
    
    void run();

protected:
    std::unique_ptr<ros::NodeHandle> m_node;

    /// \brief Sets the name of a variable in the state.
    /// \param index The index of the state variable to name.
    /// \param name The name to give the specified variable (must be ROS graph name compliant).
    /// \note Variable name changes only take effect at a new call of run().
    void set_variable_name(uint32_t index, const std::string& name);

    double_t dt() const;

private:
    double_t m_dt;

    std::vector<std::string> m_variable_names;
    std::vector<ros::Publisher> m_state_publishers;
};

#define REGISTER_PLUGIN(class_name) extern "C" class_name* instantiate() {return new class_name();}

}

#endif