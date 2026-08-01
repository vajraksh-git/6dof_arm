#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>

using namespace std::chrono_literals;

// -----------------------------------------------------------------------
// Non-blocking single-keypress reader (raw terminal mode).
// -----------------------------------------------------------------------
class KeyboardReader
{
public:
  KeyboardReader()
  {
    tcgetattr(STDIN_FILENO, &orig_termios_);
    termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  ~KeyboardReader()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
  }

  // Returns 0 if no key was pressed since last call.
  char readKey()
  {
    char c = 0;
    int n = read(STDIN_FILENO, &c, 1);
    return (n > 0) ? c : 0;
  }

private:
  termios orig_termios_;
};

// -----------------------------------------------------------------------
// Publishes a Twist representing desired end-effector linear velocity.
// Sends zero the instant no relevant key is currently held, so motion
// stops immediately when the operator stops pressing keys (Stage 2
// requirement: "Motion stops immediately when operator input ceases").
// -----------------------------------------------------------------------
class TeleopKeyboard : public rclcpp::Node
{
public:
  TeleopKeyboard()
  : Node("teleop_keyboard")
  {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cartesian_velocity_cmd", 10);

    declare_parameter("linear_speed", 0.10);   // m/s
    linear_speed_ = get_parameter("linear_speed").as_double();

    timer_ = create_wall_timer(20ms, std::bind(&TeleopKeyboard::tick, this));

    RCLCPP_INFO(get_logger(),
      "Teleop ready. w/s: +/-X   a/d: +/-Y   q/e: +/-Z   x: stop   Ctrl+C: quit");
  }

private:
  void tick()
  {
    char key = reader_.readKey();

    geometry_msgs::msg::Twist twist;  // defaults to all-zero

    switch (key) {
      case 'w': twist.linear.x =  linear_speed_; break;
      case 's': twist.linear.x = -linear_speed_; break;
      case 'a': twist.linear.y =  linear_speed_; break;
      case 'd': twist.linear.y = -linear_speed_; break;
      case 'q': twist.linear.z =  linear_speed_; break;
      case 'e': twist.linear.z = -linear_speed_; break;
      case 'x': /* explicit stop, twist stays zero */ break;
      default:
        // No key this tick -> also publish zero. This is intentional:
        // teleop should only move while a key is actively held/repeating.
        break;
    }

    pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  KeyboardReader reader_;
  double linear_speed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopKeyboard>());
  rclcpp::shutdown();
  return 0;
}