# attitude_estimation_walls
## System architecture
![ekf_architecture](https://user-images.githubusercontent.com/37431972/108962123-a4c0e900-76bb-11eb-9a49-ad67c9b18563.png)
## Usage
This is just a example.
```bash
$ roscd dnn_attitude_estimation/docker/nvidia_docker1_kinetic
$ ./camera_mle_inference.sh
```
Open another terminal.
```bash
$ roslaunch attitude_estimation_walls ekf_gyro_dgsphere_mle_real.launch
```
## Related repositories
- [ozakiryota/dnn_attitude_estimation](https://github.com/ozakiryota/dnn_attitude_estimation)
- [ozakiryota/image_to_gravity](https://github.com/ozakiryota/image_to_gravity)
- [ozakiryota/dataset_image_to_gravity](https://github.com/ozakiryota/dataset_image_to_gravity)
