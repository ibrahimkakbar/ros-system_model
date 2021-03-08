#include <system_model/example_system_model.hpp>

using namespace system_model;

// An example state transition for the UKF filter where x_t+1 = f(x_t) is x_t+1 = I * x_t
// and I is the identity matrix
void example_system_model::state_transition(Eigen::VectorXd& xp, Eigen::VectorXd& x) const
{
    x = xp;
}

// An example observation model for the UKF filter where z_t = h(x_t) is a range model
// Assumes that the position observation is the 1st n indices and in a Euclidean space with Cartesian coordinates
// Assumes the state vector contains position and n_x > n_z
void example_system_model::observation(Eigen::VectorXd& x, Eigen::VectorXd& z) const
{
    // Get number of observations
    uint32_t n_o = example_system_model::m_observations.size();

    // Error if measurements less than 3
    if (n_o < 3)
        throw std::runtime_error("failed to perform an observation update, number of measurements less than observation dimensions")

    // Generate measured observation interface vector
    Eigen::VectorXd m_z(example_system_model::n_x);
    for (auto i = example_system_model::m_observations.begin(); i != example_system_model::m_obserations.end(); ++i)
    {
        if (m_i == 3)
            break;
        m_z(m_i++) = i->second;
    }

    // Compute L2 norm of distance
    z = (m_z - x).norm();
}

REGISTER_PLUGIN(example_system_model)