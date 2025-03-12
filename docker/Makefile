build:
	docker build -t ego_swarm .

build-clone:
	docker build --build-arg DUMMY=`date +%s` -t ego_swarm .
run:
	docker run  -it \
			--gpus all \
			--env="NVIDIA_DRIVER_CAPABILITIES=all" \
			--privileged \
			-e DISPLAY=${DISPLAY} \
			--network=host \
			--env="DISPLAY" \
			--env="QT_X11_NO_MITSHM=1" \
			--rm ego_swarm
