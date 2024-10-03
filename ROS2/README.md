# ROS 2

## Daftar Isi

- [Definisi](#definisi)
- [Instalasi](#instalasi)
- [Konsep](#konsep)
  - [Topic](#topic)
  - [Service](#service)
  - [Action](#action)
  - [Interfaces](#interfaces)
- [Praktik](#implementasi)
  - [Setup Workspace](#setup-workspace)
  - [Topic](#topic-1)
  - [Service](#service-1)
  - [Action](#action-1)

## Definisi

ROS merupakan sebuah _framework_ yang dirancang khusus untuk membuat aplikasi robot. ROS memiliki banyak driver, algoritma, dan alat-alat lainnya yang dapat membantu _developer_ untuk membuat robot dengan _robust_. Tak hanya itu, ROS bersifat _open source_, sehingga selain adanya transparansi untuk para _developer_, ROS juga **gratis**.

<div align="center">
<img alt="ketika ada yang gratis" src="https://th.bing.com/th/id/OIP.gC_D-cCr9EsmXnc7XTNU4QAAAA?rs=1&pid=ImgDetMain" />
</div>

## Instalasi

Untuk contoh ini, kita akan melakukan instalasi ROS 2 distribusi Humble dalam sistem operasi Ubuntu / Lubuntu / Kubuntu / Xubuntu (pokoknya jangan Uwubuntu ataupun Winbuntu ataupun distro aneh2 lain) 22.04.

Pada awalnya, pastikan locale kalian mensupport UTF-8. Apabila tidak, masukkan perintah-perintah berikut ke terminal.

```shell
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Setelah itu, tambahkan repository Ubuntu Universe.

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Tambahkan GPG key ROS 2.

```shell
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Dan tambahkan repository ke _source list_.

```shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Setelah itu, install ROS 2 Humble.

```shell
sudo apt update
sudo apt upgrade
sudo apt install ros-$ROS_DISTRO-desktop
```

Dan lakukan setup ROS 2 Humble.

```shell
source /opt/ros/humble/setup.bash
```

Untuk memastikan ROS 2 Humble sudah terinstalasi, jalankan beberapa program ROS 2 Humble.

```shell
# Di satu terminal
ros2 run demo_nodes_cpp talker

# Di terminal lain
ros2 run demo_nodes_cpp listener
```

Apabila tidak terdapat error, selamat! Kalian sudah melakukan instalasi ROS 2 Humble dengan baik dan benar :+1:

## Konsep

### Node

Jaringan ROS 2 terdiri dari berbagai node. Setiap node ini biasanya menjalankan sebuah tugas tertentu, seperti mendapatkan data jarak dari LIDAR atau menggerakan roda robot. Setiap node dapat mengirim maupun mendapatkan data dari _topic_, _service_, maupun _action_.

![node](https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif)

Dalam ROS 2, sebuah _executable_ dapat memiliki satu atau lebih node. Untuk menjalankan sebuah node, kita dapat menggunakan perintah:

```shell
ros2 run <package> <executable>
```

### Topic

_Topic_ berfungsi sebagai sebuah bus dalam komunikasi ROS 2. Mekanisme komunikasi mereka gak kenal diskriminasi. Oleh sebab itu, berbagai node dapat mengirim maupun mendapat data dari 1 topik yang sama secara terus menerus. Mereka hanya perlu _publish_ maupun _subscribe_ terhadap _topic_ tersebut.

Namun, komunikasi mereka biasanya bersifat _one way_, dimana sebuah node dapat mendapat data dari sebuah topic namun tidak dapat mengirim kembali data tersebut lewat _topic_ yang sama.

![topic](https://docs.ros.org/en/humble/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

### Service

Berbeda dengan metode komunikasi _topic_, _service_ hanya memberi data yang diinginkan apabila diminta oleh sebuah _client_. Respons yang dikirimkan oleh sebuah _service_ hanya ditujukan kepada _client_ yang memberi permintaan.

![service](https://docs.ros.org/en/humble/_images/Service-MultipleServiceClient.gif)

### Action

_Action_ mirip dengan _service_, dimana mereka hanya memberikan respons kepada _client_ yang memberi sebuah permintaan. Namun, _service_ langsung mengakhiri layanannya setelah dipanggil sedangkan _action_ bertujuan untuk memberikan _feedback_ selama melakukan sebuah proses terhadap data yang diberikan.

![action](https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif)

### Interfaces

Dalam kehidupan nyata, data yang dilewatkan dalam sistem robot kompleks dan bervariasi. Untungnya, ROS 2 menyediakan _interfaces_ untuk membuat struktur data sesuai keinginan kita.

## Implementasi

Dalam implementasi konsep-konsep sebelumnya, ROS 2 memberikan _client library_ yang mempermudah hidup pembuat robot.

Akan tetapi, Banyu suka ribet. Untuk itu, kita menggunakan _client library_ C++ untuk contoh-contoh ini :moyai:.

### Setup Workspace

Workspace merupakan sebuah tempat dimana kita membuat _package_. Struktur sebuah workspace adalah seperti berikut.

```
ws/
| build/
| install/
| log/
+ src/
  | package1
  | package2
  + package3
```

Sebelum membuat sebuah aplikasi, kita perlu melakukan instalasi _build tool_. ROS 2 menggunakan `colcon` dalam melakukan _build_.

```shell
sudo apt update
sudo apt upgrade
sudo apt install python3-colcon-common-extensions
```

Dalam melakukan _build_, `colcon` akan menghasilkan 3 folder, yakni _build_, _install_, dan _log_.

-   _build_ berisi program-program hasil pembuatan.
-   _install_ berisi _package_ hasil pembuatan.
-   _log_ berisi catatan setiap pemanggilan `colcon`

Setelah melakukan instalasi `colcon`, kita dapat mulai membuat aplikasi robot :smile:.

### Topic

Di folder `src`, buatlah sebuah package.

```shell
ros2 pkg create pubsub --build-type ament_cmake --dependencies rclcpp std_msgs --license Apache-2.0
```

Setelah menjalankan perintah tersebut, sebuah folder bernama `pubsub` akan muncul. Folder tersebut juga berisi

```
pubsub
| src/
| LICENSE
| CMakeLists.txt
+ package.xml
```

Tambahkan file-file berikut ke dalam folder `pubsub/src`:

`pub.cpp`

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::chrono_literals;

class Publisher : public rclcpp::Node
{
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    void timer_callback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Halo dunia!";

        pub_->publish(msg);
    }

    public:
        Publisher() : Node("pub")
        {
            timer_ = this->create_wall_timer(
                500ms,
                std::bind(&Publisher::timer_callback, this)
            );

            pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}
```

`sub.cpp`

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), msg.data.c_str());
    }

    public:
        Subscriber() : Node("node")
        {
            sub_ = this->create_subscription<std_msgs::msg::String>(
                "topic",
                10,
                std::bind(&Subcriber::topic_callback, this, _1)
            );
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}
```

Dan menambahkan _executable_ dalam `CMakeLists.txt`.

```cmake
add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp std_msgs)

add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp std_msgs)

install(
    TARGETS pub sub
    DESTINATION lib/${PROJECT_NAME}
)
```

Lalu, lakukan _build_ dengan `colcon`.

```shell
colcon build
```

Apabila tidak terdapat error, jalankan perintah berikut.

```shell
ros2 run pubsub pub # Di satu terminal
ros2 run pubsub sub # Di terminal lain
```

### Service

Buatlah sebuah _package_ baru bernama `interfaces` dalam folder `src`. _Package_ ini akan berisi semua `msg`, `srv`, `action` buatan sendiri.

```shell
ros2 pkg create interfaces --build-type ament_cmake --dependencies std_msgs rosidl_default_generators --license Apache-2.0
```

Dalam `interfaces/src`, buatlah sebuah folder bernama `srv`. Dalam folder tersebut, buatlah sebuah file bernama `Add.srv` yang berisi:

```srv
int16 a
int16 b
---
int16 sum
```

Dalam `package.xml`, tambahkan:

```xml
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Dalam `CMakeLists.txt`, tambahkan:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Add.srv"
    DEPENDENCIES std_msgs
)
```

Lakukan _build_.

```shell
colcon build
```

Dan source installasi.

```shell
. install/setup.bash
```

Buatlah sebuah _package_ baru bernama `calculator` dalam folder `src`.

```shell
ros2 pkg create calculator --build-type ament_cmake --dependencies rclcpp interfaces --license Apache-2.0
```

Setelah itu tambahkan file-file berikut ke dalam folder `calculator/src`

`server.cpp`

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/add.hpp"

void add(const std::shared_ptr<interfaces::srv::Add::Request> req,
         std::shared_ptr<interfaces::srv::Add::Response> res)
{
    res->sum = req->a + req->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requested %d + %d, Responded %d", req->a, req->b, res->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> server = std::make_shared<rclcpp::Node>("add_server");

    rclcpp::Service<interfaces::srv::Add>::SharedPtr adder = node->create_service<interfaces::srv::Add>("add", &add);

    rclcpp::spin(server);
    rclcpp::shutdown();

    return 0;
}
```

`client.cpp`

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/add.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("client");
    rclcpp::Client<interfaces::srv::Add>::SharedPtr client = node->create_client<interfaces::srv::Add>("add");

    auto req = std::make_shared<interfaces::srv::Add::Request>();
    req->a = atoi(argv[1]);
    req->b = atoi(argv[2]);

    while (client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Terminated");
            return -1;
        }

        RCLCPP_INFO(node->get_logger(), "Waiting for service");
    }

    auto res = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node, res) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(node->get_logger(), "Sum: %d", res->sum);
    else
        RCLCPP_ERROR(node->get_logger(), "Failed to call service");

    rclcpp::shutdown();

    return 0;
}
```

Dalam `CMakeLists.txt`, tambahkan:

```cmake
add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp interfaces)

install(
    TARGETS server client
    DESTINATION lib/${PROJECT_NAME}
)
```

Jalan perintah berikut dalam 

### Action

Dalam package _interfaces_, buatlah folder baru bernama `action`. Dalam folder tersebut, tambahkan file `Fibonnaci.action` yang berisi:

```action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

Buatlah sebuah _package_ baru bernama `fibonnaci` dengan menjalankan perintah berikut:
```shell
ros2 pkg create fibonnaci --build-type ament_cmake --dependencies rclcpp rclcpp_action rclcpp_components interfaces --license Apache2.0
```

Setelah itu, tambahkan file-file berikut:

`include/fibonnaci/visibility_control.h`
```cpp
#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
  #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_TUTORIALS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
```

`src/server.cpp`

```cpp
#include <functional>
#include <memory>
#include <thread>

#include "interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      sequence.push_back(sequence[i] + sequence[i - 1]);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
```

`src/client.cpp`

```cpp
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
```

Lalu tambahkan ini dalam `CMakeLists.txt`

```cmake
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(action_client SHARED
  src/client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

Lalu jalankan perintah ini di direktori workspace:
```shell
colcon build
```

Setelah proses _build_ selesai, jalankan:
```shell
ros2 run fibonacci server # Di satu terminal
ros2 run fibonacci client # Di terminal lain
```

## Tugas

Buatlah sebuah _package_ bernama `controller` yang berisi sebuah node dengan tujuan mendapatkan input joystick XBox dan mengirimkan perintah pergerakan `x`, `y`, `depth`, dan `yaw`. Namun kalian juga dibebaskan untuk menambahkan perintah-perintah yang lain. Perintah ini akan dikirmkan ke topic bernama `/cmd_vel`.

Untuk mendapat input dari joystick XBox, kalian dapat menggunakan package [joy](http://wiki.ros.org/joy).

## Referensi

https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html
