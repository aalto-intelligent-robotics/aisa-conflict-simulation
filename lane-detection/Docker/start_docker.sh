# docker run -it --net=host -v [host_folder]:[docker_folder]  --name wasp_ros wasp_ros
docker run -it --net=host -v /mnt/vol2/code/aisa/wasp_ros:/home/aisa/code/src  --name wasp_ros wasp_ros
