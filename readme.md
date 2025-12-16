# Docker image for Vicon motion capture system
This repo contains dockerfile to build an image with ROS humble with vicon_bridge package. In order to build the docker image run
```
sudo docker build -t vicon_bridge .
```

## Configure the vicon ip
In order to configure the vicon ip, edit the file ```.env``` and input the ip of the vicon in ```VICON_IP```

## Run the Vicon
To run the vicon bridge 
```
docker compose up vicon_bridge -d 
```

## Enter in the Docker
```
docker compose run enter bash
```

## Rviz 
```
xhost +
docker compose up rviz 
```