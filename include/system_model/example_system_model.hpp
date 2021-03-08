#include <system_model/system_model.hpp>

namespace system_model {

class example_system_model : protected system_model_t
{
    protected:
        void state_transition(Eigen::VectorXd& xp, Eigen::VectorXd& x) const override;
        void observation(Eigen::VectorXd& x, Eigen::VectorXd& z) const override;
};

}



