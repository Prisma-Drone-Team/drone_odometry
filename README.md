# drone_odometry

## Launch

I launch inclusi sono i seguenti:

- **`zed_camera.launch.py`**  
  Copia del launch file della ZED dal package ufficiale, ma usa i parametri contenuti in questo package, così da poter essere modificati facilmente.  
  Viene incluso dagli altri due launch file.

- **`odom_leonardo.launch.py`**  
  Lancia tutto il necessario per far navigare il drone con VIO in frame `odom`, ovvero:  
  - la camera e i relativi TF  
  - il microagent  
  - il nodo `px4_tf_pub` per avere i topic lato ROS in ENU

- **`slam_leonardo.launch.py`**  
  Lancia tutto il necessario per far navigare il drone con VIO + SLAM in frame `map`:  
  in aggiunta a quanto lanciato da `odom_leonardo.launch.py`, avvia anche `rtabmap` per lavorare con la ZED e pubblicare la TF `map -> odom`.

## Build

Compilare preferibilmente con:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release