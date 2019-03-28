#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alproxies/alproxies.h>

#include <mutex>
#include <thread>

#define UNSET_STIFFNESS_TICKS 5

namespace AL{
  class ALBroker;
}

struct Vec3 {
  float x, y, z;
  Vec3()
    : x(0), y(0), z(0) {
  }
  Vec3(float x, float y, float z)
    : x(x), y(y), z(z) {
  }
  Vec3(AL::ALValue data)
    : x(data[0]), y(data[1]), z(data[2]) {
  }
};


class MotionController : public AL::ALModule {
public:
  MotionController(boost::shared_ptr<AL::ALBroker> broker,
                  const std::string &name);

  virtual void init();
  virtual void exit();

  void move(const float& linear_x,
            const float& linear_y,
            const float& angular_z);
  
  void set_linear_velocity(const float& linear);
  
  void set_angular_velocity(const float& angular);
  
  void set_acceleration(const float& acce);

protected:
  Vec3 get_desired_wheel_velocities();
  Vec3 get_current_wheel_velocities();

  void set_wheel_velocities(Vec3 velocities);

  AL::ALValue create_command(float value, int delta_time);

  void update();

  bool isInterrupted();

private:
  AL::DCMProxy _dcm_proxy;
  AL::ALMemoryProxy _memory_proxy;

  Vec3 _desired_velocity;

  std::thread _update_thread;
  std::mutex _thread_lock;

  float _last_stiffness;

  int _last_velocity_tick;
  bool _running;
  
  float _linear_vel = 0.35f;
  float _angular_vel = 0.35f;
  float _acce = 0.5f;
};

#endif
