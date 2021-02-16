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

    virtual std::vector<std::string> variable_names() const = 0;

    double_t dt() const;

private:
    double_t p_loop_rate;

    double_t m_dt;

    std::vector<ros::Publisher> m_state_publishers;
};

#define REGISTER_PLUGIN(class_name) extern "C" class_name* instantiate() {return new class_name();}

}

#endif