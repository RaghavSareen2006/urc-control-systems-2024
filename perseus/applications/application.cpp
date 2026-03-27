//copied from drivers/applications/h_bridge_demo.cpp

#include "../hardware_map.hpp"
#include <h_bridge.hpp>
#include <libhal-util/steady_clock.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {


void application()
{
<<<<<<< Updated upstream
  // perseus shoulder_motor(apram1, param2);
  // institialize can, hbridge etc.
=======
  using namespace std::chrono_literals;
  using namespace hal::literals;
  auto console = resources::console();
  auto clock = resources::clock();
  auto h_bridge = resources::h_bridge();
  auto encoder = resources::encoder();
  
  bldc_perseus servo(h_bridge, encoder);
  auto servo_ptr = hal::v5::make_strong_ptr<decltype(servo)>(resources::driver_allocator(), std::move(servo));
  hal::print(*console, "Pre-homing\n");
  while(true)
  {
     servo_ptr->set_power(0.3f);
  }
}
>>>>>>> Stashed changes

  

  // shoulder_motor.set_position(50.0_degrees);

  // on_can_receive(0x)
    //-> perform set_position task
}
}  // namespace sjsu::drivers