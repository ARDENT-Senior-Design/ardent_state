
#ifndef JOINT_H
#define JOINT_H

#include <tinyxml.h>
#include <urdf_model/joint.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <sensor_msgs/JointState.h>

namespace ardent_model {

class JointState
{
  public:

    /// Constructor
    JointState(std::string joint_id);

    /// Returns the safety limits given the current position, velocity, and effort.
    void constrain();

    /// Updates the measured values. 
    // THIS FUNCTION IS ONLY CALLED IF ODRIVE'S OR SOME OTHER EXTERNAL HARDWARE IS USED
    // TODO: Replace with callback
    void update();

    // ID of the joint
    std::string id_;

    // Flag to kill or stop the process
    bool violated_limits_;

    // rads, rads/s, and Nm
    double min_position_, max_position_;
    double max_abs_velocity_;
    double max_abs_effort_;


    /// A pointer to the corresponding urdf::Joint from the urdf::Model
    // boost::shared_ptr<const urdf::Joint> joint_;
    double measured_position_;
    double measured_velocity_;
    double measured_acceleration_;
    double measured_effort_;
    /// The effort the joint should apply in Nm or N (write-to variable)
    double commanded_position_;
    double commanded_velocity_;
    double commanded_acceleration_;
    double commanded_effort_;

    /// Bool to indicate if the joint has been calibrated or not.
    // Moves the joints to the starting position and makes sure they are good
    bool calibrated_;

    /// The position of the optical flag that was used to calibrate this joint
    double reference_position_;
  private:
    template <class T> inline int sgn(T v);

};

enum
{
  JOINT_NONE,
  JOINT_ROTARY,
  JOINT_CONTINUOUS,
  JOINT_PRISMATIC,
  JOINT_FIXED,
  JOINT_PLANAR,
  JOINT_TYPES_MAX
};

}

#endif /* JOINT_H */