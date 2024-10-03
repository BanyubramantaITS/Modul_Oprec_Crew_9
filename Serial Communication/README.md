# Serial Communication (ASIO)

## Daftar Isi

- [Konsep](#konsep)
- [Instalasi](#instalasi)
- [Praktik](#praktik)
- [Tugas](#tugas)

## Konsep

_Serial Communication_ merupakan metode komunikasi yang mengirimkan bit-bit data secara sekuensial, yaitu mengirimkan bitnya secara satu per satu.

ASIO merupakan sebuah _library_ C++ untuk pemrograman jaringan.

### Instalasi

Untuk melakukan instalasi pada ASIO, jalankan perintah ini pada perintah:
```shell
sudo apt-get update
sudo apt-get install libasio-dev
```

## Praktik

Tambahkan header file:
```cpp
#include <string>
#include <asio.hpp>
```

Lalu konfigurasi port yang akan digunakan untuk komunikasi:
```cpp
const std::string PORT = "/dev/ttyACM0";

asio::io_service io_;
asio::serial_port port_(io_, PORT);
port_.set_option(asio::serial_port_base::baud_rate(115200));
port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::software));
port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
port_.set_option(asio::serial_port_base::character_size(8));
```

Setelah itu, kalian dapat mulai mengirim data yang ingin dikirimkan
```cpp
std::string buf = "Hello World!";
asio::write(port_, asio::buffer(buf));

std::cout << "Sent: " << buf << std::endl;
```

Untuk melihat apa yang telah dikirim di port tertentu, tambahkan:
```cpp
const int READ_SIZE = 100;

char response[READ_SIZE];
size_t len = asio::read(port_, asio::buffer(response, READ_SIZE));

std::cout << "Got: "<< std::string(response, READ_SIZE) << std::endl;
```

## Tugas

Buatlah sebuah _package_ yang berisikan _subscriber_ di ROS yang akan mendengar topik `/cmd_vel` dan mengeluarkan isinya ke port `/dev/ttyACM0`.

## Referensi

https://think-async.com/Asio/asio-1.30.2/doc/