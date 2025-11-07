obj-m += main.o

BUILD_DIR := build

all:
	clear
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD)/$(BUILD_DIR) src=$(PWD) modules
	@echo

clean:
	clear
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD)/$(BUILD_DIR) src=$(PWD) clean
	@echo

add: build/main.ko
	@if lsmod | grep -q "^e1000"; then \
		sudo rmmod e1000; \
	fi
	sudo insmod build/main.ko
	sudo ip addr add 10.2.163.229/23 dev mustafa0
	sudo ip link set mustafa0 up
	make debug

add_min: build/main.ko
	@if lsmod | grep -q "^e1000"; then \
		sudo rmmod e1000; \
	fi
	sudo insmod build/main.ko
	sudo ip link set mustafa0 up
	make debug


remove: build/main.ko
	@if lsmod | grep -q "^main"; then \
		sudo rmmod build/main.ko; \
	fi
	sudo modprobe e1000
	sudo ip addr add 10.2.163.229/23 dev enp0s3
	sudo ip link set enp0s3 up
	make debug

ip:
	sudo dhclient -v mustafa0

check:
	@echo -n "Proc: "; if [ -e /proc/mustafa_driver ]; then echo T; else echo F; fi
	@echo -n "Dev:  "; if [ -e /dev/mustafa_driver ]; then echo T; else echo F; fi

write:
	clear
	sudo python3 test.py $(wordlist 2, 99, $(MAKECMDGOALS))
	@echo

debug:
	clear
	sudo dmesg --color=always | tail -n 45
	@echo

redo: 
	make remove
	make clean
	make
	make add 

ping:
	ping -c 10 10.2.163.224 

host:
	sudo arp -s 10.2.163.224 c4:cb:e1:25:48:f9

vm:
	sudo arp -s 10.2.163.229 b6:f8:6c:e6:fa:70

%:
	@:
