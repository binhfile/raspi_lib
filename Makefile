all:
	arm-linux-gnueabihf-g++ gpio.c delay.c dht11.c dht11_test.c -o dht11 -static -O2
