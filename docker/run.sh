xhost +local:root > /dev/null 2>&1
xhost +local:docker

container_name=container

if [ "$( sudo docker container inspect -f '{{.State.Status}}' $container_name )" == "running" ] 
then 
echo "Connected to existed $container_name container"
docker exec -it $container_name /bin/bash
fi

network_name=host
docker run --rm -it --name $container_name -e DISPLAY --network $network_name --mount type=bind,source=/home/ss21mipt/Documents/starkit/StartUp/starkit_ik_walk,target=/root/tmp starkit/container
