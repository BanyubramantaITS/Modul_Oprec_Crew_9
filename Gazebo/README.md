# Gazebo

## Definisi

Gazebo adalah aplikasi simulasi realistis khusus untuk simulasi robot. Gazebo menyediakan berbagai jenis plugin dan sensor sehingga kami dapat melakukan simulasi dan robot yang sesuai dengan lingkungan nyata dan robot asli.

## Instalasi

Gazebo memiliki 3 versi, yaitu:

### Gazebo Classic

Gazebo Classic merupakan versi Gazebo tertua dan sudah mendekati EOL. Meski begitu, integrasinya dengan ROS (khususnya ROS 1) tidak dapat dipungkiri. 

Untuk melakukan instalasi, kalian dapat install Debian package di [website Gazebo Classic](https://classic.gazebosim.org/).

### Gazebo Ignition

Gazebo Ignition bisa dikatakan seperti middle child dalam keluarga Gazebo ini. Gazebo Ignition awalnya berupa versi baru dari Gazebo Classic dengan _physics engine_ lebih baik, sensor dan plugin lebih banyak, dan integrasi dengan ROS 2. Namun setelah Gazebo Sim 6, nama Gazebo Ignition diubah menjadi Gazebo Sim.

Untuk melakukan instalasi versi terbaru Gazebo Ignition, kalian dapat jalankan:
```
sudo apt install gazebo-fortress
```

Untuk memastikan apabila Gazebo Ignition telah terinstalasi, kalian dapat menjalankan:
```
ign gazebo
```

### Gazebo Sim

## Konsep

Dalam Gazebo, terdapat berbagai konsep yang perlu dipahami untuk membuat simulasi. Berikut konsep-konsep tersebut.

### World

World merupakan lingkungan yang akan digunakan untuk simulasi robot. Secara default, world kalian akan berisi _plane_ besar saja.

### Model

Dalam Gazebo, setiap objek merupakan sebuah model. Model dapat berupa robot kalian, sebuah kotak

#### Link

Link merupakan komponen dari setiap model. Misalkan model kita berupa robot Narumusa, link yang terdapat pada model tersebut dapat berupa thruster, camera, badan, dan banyak lagi.

Dalam sebuah link, terdapat 3 komponen penting, yaitu:

##### Visual

Visual mendefinisikan bagaimana bentuk visual sebuah link.

##### Collision

Collision mendefinisikan bagaimana sebuah link berinteraksi secara fisik dalam sebuah simulasi.

##### Inertia

Inertia mendefinisikan distribusi massa pada sebuah link. Hal ini sangat penting untuk simulasi robot yang realistis.

### Plugin

Plugin dapat diibaratkan seperti library dalam sebuah file. Library tersebut dapat memberi fungsi tambahan kepada file tersebut. Dalam kasus ini, plugin dapat menambah fitur kepada simulasi kita. Contohnya berupa plugin hidrodinamika dan buoyancy untuk simulasi air serta plugin thruster untuk mengubah sebuah link menjadi thruster robot.

### Format Deskripsi

Untuk "mendeskripsikan" berbagai world dan model kami, terdapat berbagai XML format yang tersedia, yaitu:

#### SDF

SDF (Simulation Description Format) merupakan format simulasi robot 

Pros:
- Dapat mendeskripsikan robot secara detail

Cons:
- Nguli
- Nguli
- Nguli

#### URDF

URDF (Unified Robot Description Format) merupakan format

Namun mirisnya, Gazebo hanya dapat membaca format SDF saja. Untuk itu, kita perlu mengubah format URDF menjadi format SDF. Untuk mengubah formatnya, kita hanya perlu menjalankan:
```
gz sdf -p myrobot.urdf > myrobot.sdf	# Gazebo Classic
ign sdf -p myrobot.urdf > myrobot.sdf 	# Ignition dan seterusnya
```

Pros:
- Memiliki integrasi ROS yang baik

Cons:
- Masih nguli :moyai:

Apabila kalian memiliki CAD model dari robot kalian, kalian dapat ubah CAD model tersebut menjadi URDF lewat cara-cara berikut:

- [PTC Creo](https://github.com/icub-tech-iit/cad-libraries/wiki/Prepare-PTC-Creo-Mechanism-for-URDF)
- [SolidWorks](http://wiki.ros.org/sw_urdf_exporter)
- [Fusion](https://github.com/syuntoku14/fusion2urdf)
- [OnShape](https://onshape-to-robot.readthedocs.io/en/latest/)
- [Blender](https://github.com/dfki-ric/phobos)

#### Xacro

Xacro (XML with macros) merupakan format XML yang menyediakan fitur pembuatan macro, sehingga kita dapat membuat variabel dan macro serta menyesuaikan nilai variabel-variabel tersebut dan menggunakan macro secara berulang untuk menentukan deskripsi yang sesuai. Hal tersebut sangat mempermudah pembuatan deskripsi dan menghilangkan pengulangan redundan.

Namun, mengingat bahwa Gazebo hanya dapat membaca format SDF, kita perlu mengubah format Xacro menjadi SDF. Akan tetapi, Xacro hanya dapat diubah menjadi format URDF saja. Oleh sebab itu, kita perlu mengubah Xacro menjadi URDF lalu URDF menjadi SDF. Hal tersebut dapat dilakukan dengan cara berikut:
```
xacro myrobot.xacro > myrobot.urdf
gz sdf -p
```

## Praktik