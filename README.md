# **GStreamer LiDAR Streaming with SCReAM Integration (NOT READY)**

This repository provides a GStreamer-based LiDAR streaming pipeline integrated with the SCReAM congestion control algorithm. The project includes custom GStreamer plugins, a Docker setup for easy deployment, and necessary scripts to facilitate streaming.  

## **Repository Structure**  

```
├── docker/                     # Docker-related files
│   ├── Dockerfile              # Dockerfile for building the container
│   ├── entrypoint.sh           # Entrypoint script to initialize the container
│   ├── create_image.sh         # Script to build the Docker image
│   ├── run_container.sh        # Script to run the container
├── src/                        # Source code for the plugin
│   ├── build.rs                # Build script for the plugin
│   ├── env.sh                  # Environment variables for configuration
│   ├── gstlidar.rs             # 
│   ├── lib.rs                  #
│   ├── lidar_receiver.py       #
│   ├── lidar_receiver.sh       #
│   ├── lidar_sender.py         # Script to run the LiDAR streaming pipeline
│   ├── lidar_sender.sh         # Script to run the LiDAR streaming pipeline
│   ├── sender_utils.rs         #
│   ├── senders.rs              # 
├── README.md                    # Project documentation
```

## **Getting Started**

### **Build and Run with Docker**
This project includes a Docker setup for easy deployment.  

#### **Build the Docker Image**
Run the following script to build the Docker image:
```sh
cd docker/
./create_image.sh
```
This script will build the necessary dependencies  

#### **Run the Docker Container**
Once the image is built, start the container with:
```sh
./run_container.sh
```
This will:
- Run the container with the necessary mounts and environment variables  
- Compile the GStreamer SCReAM plugin  
- Set up the runtime environment  
- Initialize the LiDAR streaming pipeline  
