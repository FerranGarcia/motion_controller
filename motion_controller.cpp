#include "motion_controller.h"
#include <chrono>
#include <cmath>

MotionController::MotionController(boost::shared_ptr<AL::ALBroker> broker,
                const std::string &name)  : AL::ALModule(broker, name), _last_velocity_tick(0), _last_stiffness(0.0), _running(true) {
  setModuleDescription("Motion controller providing low latency motion response");

  functionName("move", getName(), "Moves the robot base at a given velocity");
  addParam("linear_x", "Velocity along the x-axis in meters per second");
  addParam("linear_y", "Velocity along the y-axis in meters per second");
  addParam("angular_z", "Velocity around the z-axis in radians per second");
  BIND_METHOD(MotionController::move);
  
  functionName("set_linear_velocity", getName(), "Sets a given linear velocity");
  addParam("linear_vel", "Velocity along the x and y-axis in meters per second");
  BIND_METHOD(MotionController::set_linear_velocity);
  
  functionName("set_angular_velocity", getName(), "Sets a given angular velocity");
  addParam("angular_vel", "Velocity along the wz in meters per second");
  BIND_METHOD(MotionController::set_angular_velocity);
  
  functionName("set_acceleration", getName(), "Sets a given acceleration");
  addParam("acc", "Acceleration along the x y and wz axis in meters per second squared");
  BIND_METHOD(MotionController::set_acceleration);
  
  functionName("is_enabled", getName(), "Returns wether this node is enabled (received move commands)");
  addParam("", "");
  BIND_METHOD(MotionController::is_enabled);
};

void MotionController::init() {
  _dcm_proxy = AL::DCMProxy(getParentBroker());
  _memory_proxy = AL::ALMemoryProxy(getParentBroker());

  AL::ALValue commands;

  std::vector<std::string> keys = {
    "Device/SubDeviceList/WheelFL/Stiffness/Actuator/Value",
    "Device/SubDeviceList/WheelFL/Speed/Actuator/Value",
    "Device/SubDeviceList/WheelFR/Stiffness/Actuator/Value",
    "Device/SubDeviceList/WheelFR/Speed/Actuator/Value",
    "Device/SubDeviceList/WheelB/Stiffness/Actuator/Value",
    "Device/SubDeviceList/WheelB/Speed/Actuator/Value"
  };

  commands.arraySetSize(2);
  commands[0] = std::string("Wheels");
  commands[1] = AL::ALValue(keys);

  _dcm_proxy.createAlias(commands);
  
  last_move_command_ = std::chrono::system_clock::now();

  _update_thread = std::thread(&MotionController::update, this);
}
void MotionController::exit() {
  _running = false;
  AL::ALModule::exit();
}

void MotionController::move(const float& vx, const float& vy, const float& wz) {
  std::lock_guard<std::mutex> lock(_thread_lock);

  // calculate time since last execution
  is_enabled_ = true;
  last_move_command_ = std::chrono::system_clock::now();
  
  _desired_velocity = Vec3(
    std::fmax(-_linear_vel, std::fmin(_linear_vel, vx)),
    std::fmax(-_linear_vel, std::fmin(_linear_vel, vy)),
    std::fmax(-_angular_vel, std::fmin(_angular_vel, wz))
  );
}

Vec3 MotionController::get_desired_wheel_velocities() {
  std::lock_guard<std::mutex> lock(_thread_lock);

  Vec3 result(
    -2.5 * _desired_velocity.z,
    -2.5 * _desired_velocity.z,
    -2.5 * _desired_velocity.z
  );

  float vx = -5.2 * (_desired_velocity.x / 0.35);
  float vy = -5.2 * (_desired_velocity.y / 0.35);

  float velocity = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
  float drive_direction = std::atan2(vy, vx);

  drive_direction = !std::isnan(drive_direction) ? drive_direction : 0;

  //result.x += velocity * std::cos(2.61799 - drive_direction);
  //result.y += velocity * std::cos(0.523599 - drive_direction);
  //result.z += velocity * std::cos(4.71239 - drive_direction);
  
  result.x += velocity * std::cos(2.64 - drive_direction);
  result.y += velocity * std::cos(0.50 - drive_direction);
  result.z += velocity * std::cos(4.71 - drive_direction);

  return result;
}

Vec3 MotionController::get_current_wheel_velocities() {
  std::vector<std::string> keys = {
    "Device/SubDeviceList/WheelFL/Speed/Sensor/Value",
    "Device/SubDeviceList/WheelFR/Speed/Sensor/Value",
    "Device/SubDeviceList/WheelB/Speed/Sensor/Value"
  };
  return Vec3(_memory_proxy.getListData(AL::ALValue(keys)));
}

void MotionController::set_wheel_velocities(Vec3 velocities) {
  AL::ALValue commands;

  if (velocities.x == 0.0 && velocities.y == 0.0 && velocities.z == 0.0) {
    _last_velocity_tick += 1;
  } else {
    _last_velocity_tick = 0;
  }

  float stiffness = (_last_velocity_tick <= UNSET_STIFFNESS_TICKS) ? 1.0 : 0.0;

  if (_last_stiffness == stiffness && _last_stiffness == 0.0) {
    return;
  }

  commands.arraySetSize(4);
  commands[0] = std::string("Wheels");
  commands[1] = std::string("ClearAll");
  commands[2] = std::string("time-mixed");
  commands[3].arraySetSize(6);

  commands[3][0] = create_command(stiffness, 0);
  commands[3][1] = create_command(velocities.x, 0);
  commands[3][2] = create_command(stiffness, 0);
  commands[3][3] = create_command(velocities.y, 0);
  commands[3][4] = create_command(stiffness, 0);
  commands[3][5] = create_command(velocities.z, 0);

  _dcm_proxy.setAlias(commands);
  _last_stiffness = stiffness;
}

AL::ALValue MotionController::create_command(float value, int delta_time) {
  AL::ALValue result;
  result.arraySetSize(1);
  result[0].arraySetSize(2);
  result[0][0] = value;
  result[0][1] = _dcm_proxy.getTime(delta_time);
  return result;
}


void MotionController::update() {
  auto last_update = std::chrono::system_clock::now();

  while (_running) {

    auto current_time = std::chrono::system_clock::now();

    if (isInterrupted()) {
      last_update = current_time;
      _last_velocity_tick = UNSET_STIFFNESS_TICKS + 1;
      _last_stiffness = 0.0f;

      std::this_thread::sleep_for (std::chrono::milliseconds(30));
      continue;
    }

    float dt = ((float)std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update).count()) / 1000;

    Vec3 desired = get_desired_wheel_velocities();
    Vec3 current = get_current_wheel_velocities();

    Vec3 delta (
      desired.x - current.x,
      desired.y - current.y,
      desired.z - current.z
    );

    float delta_max = std::fmax(fabs(delta.x), std::fmax(fabs(delta.y), fabs(delta.z)));

    Vec3 result;

    if (delta_max > 0.5) {
        float max_accel = ((desired.x == 0 && desired.y == 0 && desired.z == 0) ? 100 : 50) * dt;

        max_accel = std::fmax(-_acce, std::fmin(_acce, max_accel));

        result.x = current.x + (max_accel * (delta.x / delta_max));
        result.y = current.y + (max_accel * (delta.y / delta_max));
        result.z = current.z + (max_accel * (delta.z / delta_max));
    } else {
        result.x = desired.x;
        result.y = desired.y;
        result.z = desired.z;
    }

    float seconds_since_last_move_command = ((float)std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_move_command_).count()) / 1000;
    if ( is_enabled_ ) {
        if ( seconds_since_last_move_command < 1. ) 
        {
            set_wheel_velocities(result);
        } else {
            result.x = 0;
            result.y = 0;
            result.z = 0;
            set_wheel_velocities(result);
        }
    }
    last_update = current_time;

    std::this_thread::sleep_for (std::chrono::milliseconds(30));
  }
}

bool MotionController::isInterrupted() {
  AL::ALValue is_awake = _memory_proxy.getData("robotIsWakeUp");

  if (!is_awake) {
    _desired_velocity = Vec3();
    return true;
  }

  std::vector<std::string> keys = {
    "Motion/Command/Velocity/WheelFL",
    "Motion/Command/Velocity/WheelFR",
    "Motion/Command/Velocity/WheelB",
  };

  Vec3 wheel_cmd = Vec3(_memory_proxy.getListData(AL::ALValue(keys)));

  if ((wheel_cmd.x + wheel_cmd.y + wheel_cmd.z) != 0) {
    _desired_velocity = Vec3();
    _last_velocity_tick = 0;
    return true;
  }

  return false;
}

void MotionController::set_linear_velocity(const float& linear)
{
    if (linear > 1.0f) {
        _linear_vel = 1.0f;
    }else{
        _linear_vel = linear;
    }
}

void MotionController::set_angular_velocity(const float& angular)
{
    if (angular > 1.0f) {
        _angular_vel = 1.0f;
    }else{
        _angular_vel = angular;
    }
}

void MotionController::set_acceleration(const float& acce)
{
    if (acce > 1.0f) {
        _acce = 1.0f;
    }else{
        _acce = acce;
    }
}

bool MotionController::is_enabled()
{
    return is_enabled_;
}
