ROS 2 containers using Fast DDS can see each other’s topics but fail to exchange data because Docker isolates shared memory, so you must either share IPC (ipc: host), disable Fast DDS shared memory (UDP-only), or switch to Cyclone DDS. 
```yaml
services:
  container1:
    ...
    network_mode: host
    ipc: host
    pid: host

  container2:
    ...
    network_mode: host
    ipc: host
    pid: host
```