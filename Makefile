CFLAGS=-Wall -O2 -static
dht11:
	arm-linux-gnueabihf-g++ $(CFLAGS) gpio.c delay.c dht11.c dht11_test.c -o dht11
ds18b20:
	arm-linux-gnueabihf-g++ $(CFLAGS) gpio.c delay.c ds18b20.c ds18b20_test.c -o ds18b20
clean:
	rm -rf dht11 ds18b20
format:
	clang-format -i *.c *.h
