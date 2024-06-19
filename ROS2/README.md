# ROS 2

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

_Topic_ berfungsi sebagai sebuah bus dalam komunikasi ROS 2. Mekanisme komunikasi mereka gak kenal diskriminasi. Oleh sebab itu, berbagai node dapat mengirim maupun mendapat data dari 1 topik yang sama secara terus menerus. Mereka hanya perlu _publish_ maupun _subsribe_ terhadap _topic_ tersebut.

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
ws
    build
    install
    log
    src
        package1
        package2
        package3
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
    src/
    LICENSE
    CMakeLists.txt
    package.xml
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
# Di satu terminal
ros2 run pubsub pub

# Di terminal lain
ros2 run pubsub sub
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

### Action

Dalam package _interfaces_, buatlah folder baru bernama `action`. Dalam folder tersebut, tambahkan file `Prime.action` yang berisi:

```action
int16 num
---
bool is_prime
---
bool partial_prime
```

Buatlah sebuah _package_ baru bernama ``

`server.cpp`

```cpp
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/action/prime.hpp"

using namespace std::placeholders;

class IsPrimeServer : public rclcpp::Node
{
    rclcpp_action::Server<IsPrime>::SharedPtr server;

    void execute(const std::shared_ptr<GoalHandleIsPrime> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<IsPrime::Feedback>();
        auto result = std::make_shared<IsPrime::Result>();

        if (goal->num < 2 && rclcpp::ok())
        {
            result->is_prime = false;
            goal_handle->succeed(result);
            return;
        }

        int i;

        for (i = 2; i < goal && rclcpp::ok(); i++)
        {
            feedback->partial_prime = !(goal->num % i);
            goal_handle->publish_feedback(feedback);
        }

        if (rclcpp::ok())
        {
            result->is_prime = !(goal->num % i);
            goal_handle->succeed(result);
        }
    }

    public:

        using IsPrime = interfaces::action::Prime;
        using GoalHandleIsPrime = rclcpp_action::ServerGoalHandle<IsPrime>;

        explicit IsPrimeServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("is_prime_server")
        {
            auto handle_goal = [this](
                const rclcpp_action::GoalHandle::GoalUUID &uuid,
                std::shared_ptr<const IsPrime::Goal> goal)
            {
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandleIsPrime> goal_handle)
            {
                (void)goal_handle;
                return rclcpp_action::GoalResponse::ACCEPT;
            }

            auto handle_accepted = [this](const std::shared_ptr<GoalHandleIsPrime> goal_handle)
            {
                auto execute_in_thread = [this, goal_handle](){ return this->execute(goal_handle) };
                std::thread(execute_in_thread).detach();
            }

            this->server = this->create_server<IsPrime>(
                this,
                "is_prime",
                &handle_goal,
                &handle_cancel,
                &handle_accepted
            );
        }
};

RCLCPP_COMPONENTS_REGISTER_NODE(IsPrimeServer)
```

`client.cpp`

```cpp
#include <functional>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/action/prime.hpp"

using namespace std::placeholders;

class IsPrimeClient : public rclcpp::Node
{
    rclcpp_action::Client<IsPrime>::SharedPtr client;

    void goal_response_callback(const GoalHandleIsPrime::SharedPtr &goal_handle)
    {
        if (!goal_handle)
            RCLCPP_ERROR(this->get_logger(), "Goal denied");
        else
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }

    void feedback_callback(
        GoalHandleIsPrime::SharedPtr /*goal_handle*/, 
        const std::shared_ptr<const IsPrime::Feedback> feedback
    )
    {
        RCLCPP_INFO(this->get_logger(), feedback->partial_prime ? "prime" : "not prime");
    }

    void result_callback(const GoalHandleIsPrime::WrappedResult &result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Aborted");
                return;
            case rclcpp_action::ResultCode::CANCELLED:
                RCLCPP_ERROR(this->get_logger(), "Cancelled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Final result: %s", result.is_prime ? "prime" : "not prime");
        rclcpp::shutdown();
    }

    public:

        using IsPrime = interfaces::action::Prime;
        using GoalHandleIsPrime = rclcpp_action::ServerGoalHandle<IsPrime>;

        explicit FibonacciActionClient(const rclcpp::NodeOptions &options) : Node("is_prime_client", options)
        {
            this->client = rclcpp_action::create_client<IsPrime>(
                this,
                "is_prime"
            );
        }

        void send_goal()
        {
            auto goal = IsPrime::Goal();
            goal.num = 50;

            auto send_goal_options = rclcpp_action::Client<IsPrime>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&IsPrimeClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&IsPrimeClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&IsPrimeClient::result_callback, this, _1);

            this->client->async_senc_goal(goal, send_goal_options);
        }
};

RCLCPP_COMPONENTS_REGISTER_NODE(IsPrimeClient)
```

## Tugas

Buatlah sebuah node controller yang mendapatkan input joystick XBox dan mengirimkan perintah pergerakan `x`, `y`, `depth`, dan `yaw`. Namun kalian juga dibebaskan untuk menambahkan perintah-perintah yang lain.

Untuk mendapat input dari joystick XBox, kalian dapat menggunakan package [joy](http://wiki.ros.org/joy).

## Referensi

https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html
