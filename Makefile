SHELL := /bin/bash
BRANCH ?= main

PASSWORD ?= laff

install:
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt install curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add
	sudo apt update
	sudo apt install ros-noetic-ros-base
	sudo apt install python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential g++ libusb-1.0-0-dev
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo rosdep init
	rosdep update

	git clone https://github.com/naoki-mizuno/ds4drv --branch devel ~/ds4drv
	cd ~/ds4drv && python3 setup.py install && sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/

	git clone https://github.com/mcfrappe/laff-platooning ~/laff-platooning
	git config --global --add safe.directory /home/laff/laff-platooning
	cd ~/laff-platooning && git submodule update --init --recursive
	cd ~/laff-platooning && sudo pip install -r requirements.txt

	sudo cp ~/laff-platooning/src/pixy2_ros/pixy2_node/pixy2/src/host/linux/pixy.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger

	sudo cp ~/laff-platooning/orchestrator/laff.service /lib/systemd/system
	sudo systemctl enable laff.service
	sudo systemctl start laff.service
	sudo loginctl enable-linger laff

	echo 'laff ALL=(ALL) NOPASSWD:ALL' | sudo EDITOR='tee -a' visudo


update:
	git config --global --add safe.directory /home/laff/laff-platooning
	cd /home/laff/laff-platooning
	git reset --hard
	git pull
	git checkout $(BRANCH)
	git pull
	rm -rf build
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "cd /home/laff/laff-platooning; source devel/setup.bash; catkin_make"

cat:
	rm -rf build
	catkin_make

run:
	source devel/setup.bash
	roslaunch joyit vehicle.launch

run_pi:
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "cd /home/laff/laff-platooning; source	devel/setup.bash; roslaunch joyit vehicle.launch --screen --pid /tmp/laff.pid"