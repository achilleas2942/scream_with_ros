echo "--------------------------------------"
echo " building scream with ros docker image"
echo "--------------------------------------"

DOCKER_BUILDKIT=1 docker build -t ghcr.io/achilleas2942/scream-with-ros:latest .