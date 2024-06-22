# Daftar Isi
- [Daftar Isi](#daftar-isi)
- [OpenCV](#opencv)
  - [Instalasi](#instalasi)
  - [Konsep](#konsep)
    - [Basic Operations](#basic-operations)
    - [Color Spaces](#color-spaces)
- [YOLO](#yolo)
  - [Convolutional Neural Network (CNN)](#convolutional-neural-network-cnn)
  - [Pengenalan YOLO](#pengenalan-yolo)
  - [Instalasi](#instalasi-1)
  - [Konsep](#konsep-1)
  - [Integrasi dengan ROS2](#integrasi-dengan-ros2)

# OpenCV

Apa itu OpenCV? OpenCV merupakan sebuah library yang bisa kita gunakan untuk melakukan pengolahan gambar, video, atau **pengolahan dari kamera secara realtime**. OpenCV bersifat open-source, dan bisa digunakan untuk mengolah citra yang dikonversi dari analog ke digital sehingga kita bisa melakukan operasi-operasi pengolahan citra. Pemrosesan gambar bisa membantu kita untuk melakukan perbaikan kualitas gambar, menghilangkan noise, identifikasi gambar, deteksi warna, dan lain-lain.

## Instalasi

**Python**

Untuk OpenCV versi python bisa langsung diinstall memakai package manager apt, seperti berikut:

```bash
sudo apt install python3-opencv -y
```

**C++**

Untuk OpenCV C++ bisa memakai library ataupun install dari source. Untuk memudahkan kita hanya akan cover bagian library.

```bash
sudo apt install libopencv-dev -y
```

## Konsep

### Basic Operations

### Color Spaces

# YOLO

## Convolutional Neural Network (CNN)

## Pengenalan YOLO

## Instalasi

**Prerequisite**
```bash
sudo apt update
sudo apt install python3
sudo apt install python3-pip
```

**YOLOv5**
```bash
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt 
```

**YOLOv8**
```bash
pip install opencv-python
pip install supervision
pip install ultralytics
```

## Konsep

## Integrasi dengan ROS2