# Behavior Tree

Behavior Tree merupakan sebuah library yang menjadi framework yang mudah diintegrasikan dengan sistem kita, ROS2.

## Instalasi

**BehaviorTree.CPP Submodule**

Karena kalian sudah membuat workspace ROS, kita masuk kedalam workspace yang telah dibuat.

```bash
cd <WORKSPACE_KALIAN>/src
git submodule add https://github.com/BehaviorTree/BehaviorTree.CPP
cd BehaviorTree.CPP
mkdir build; cd build 
cmake .. 
make
sudo make install
```

Dipakai untuk development BT yang independent, karena BT sendiri merupakan framework yang terpisah dari ROS2.

**BehaviorTree.ROS2 Submodule**

```bash
cd <WORKSPACE_KALIAN>/src
git submodule add https://github.com/BehaviorTree/BehaviorTree.ROS2
cd ..
colcon build
```

Dipakai untuk development BT bersama ROS2.

**Groot2** - [Download](https://www.behaviortree.dev/groot/)

## Konsep

Apa itu Behavior Tree? Behavior Tree merupakan sebuah struktur yang berbentuk *tree*, yang berisi banyak *node* yang melakukan kontrol terhadap eksekusi task-task, atau dalam kasus kita, eksekusi sebuah *action*.


https://www.behaviortree.dev/docs/category/tutorials-basic
https://www.behaviortree.dev/docs/category/tutorials-advanced
https://www.behaviortree.dev/docs/category/nodes-library