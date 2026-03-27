//copied from drivers/applications/h_bridge_demo.cpp

#include <h_bridge.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <bldc_servo.hpp>
#include <resource_list.hpp>


using namespace std::chrono_literals;
namespace sjsu::perseus {

// each rotation of the output shaft of the track servo is 8 mm of linear travel
// so 1 degree of rotation is 8mm / 360 = 0.0222 mm of linear travel
// 188:1 is for shoulder servo 5281.1 * 28
// 188:1 elbow 1-2 reduction. 5281.1 * 2

enum servo_address : hal::u16
{
  track_servo = 0x120,
  shoulder_servo = 0x121,
  elbow_servo = 0x122,
  wrist_pitch = 0x123,
  wrist_roll = 0x124,
  clamp = 0x125
};

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto console = resources::console();
  auto clock = resources::clock();
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  
  bldc_perseus servo(h_bridge, encoder);
  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
 
  bldc_perseus::PID_settings my_pid_values;

  my_pid_values.kp = 1.0;
  my_pid_values.ki = 0.0;
  my_pid_values.kd = 0.0; 

  servo_ptr->update_pid_position(my_pid_values);
  servo_ptr->set_target_position(90.0);

  hal::print(*console, "Pre-PID settings\n");
  while(true)
  {
     servo_ptr->update_position();
     hal::print<64>(*console, "%f\n", servo_ptr-> get_reading_position());
  }
}

}  // namespace sjsu::perseus