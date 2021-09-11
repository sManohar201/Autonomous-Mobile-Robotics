#ifndef SENSOR_FUSION_FILTER_COMMON_H
#define SENSOR_FUSION_FILTER_COMMON_H


namespace SensorFusion {

  /**
   * @brief Enumeration that defines the state vector
   */
  enum StateMembers {
    StateMemberX = 0,
    StateMemberY,
    StateMemberZ,
    StateMemberRoll,
    StateMemberPitch,
    StateMemberYaw,
    StateMemberVx,
    StateMemberVy,
    StateMemberVz,
    StateMemberVroll,
    StateMemberVpitch,
    StateMemberVyaw,
    StateMemberAx,
    StateMemberAy,
    StateMemberAz
  };

  /**
   * @brief Enumeration that defines the control vector
   * 
   */
  // TODO: what exactly this is used for
  enum ControlMembers {
    ControlMemberVx,
    ControlMemberVy,
    ControlMemberVz,
    ControlMemberVroll,
    ControlMemberVpitch,
    ControlMemberVyaw
  };
  /**
   * @brief Global constants that define our state
   * vector size and offsets to groups of values
   * within that state.
   * 
   */
  // TODO: what are the offsets here
  const int STATE_SIZE = 15;
  const int POSITION_OFFSET = StateMemberX;
  const int ORIENTATION_OFFSET = StateMemberRoll;
  const int POSITION_V_OFFSET = StateMemberVx;
  const int ORIENTATION_V_OFFSET = StateMemberVroll;
  const int POSITION_A_OFFSET = StateMemberAx;

  /**
   * @brief Pose and twist messages each 
   * contain six variables.
   * 
   */
  // TODO: why not angular velocity size
  // FIXME: maybe infer angular velocity size from linear velocity size?
  const int POSE_SIZE = 6;
  const int TWIST_SIZE = 6;
  const int POSITION_SIZE = 3;
  const int ORIENTATION_SIZE = 3;
  const int LINEAR_VELOCITY_SIZE = 3;
  const int ACCELERATION_SIZE = 3;

  /**
   * @brief common variables
   * 
   */
  const double PI = 3.141592653589793;
  const double TAU = 6.283185307179587;

} // namespace SensorFusion

#endif
