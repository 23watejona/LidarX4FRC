%.o: %.cpp
	g++ -std=c++17 -c -g -I/usr/local/frc/include -o $@ $^

lidar.exe: lidar.o serial.o ntpublish.o
	g++ -L/usr/local/frc/lib -lpthread -lntcore -o $@ $^

clean:
	rm *.o *.exe

sample: sample.o ntpublish.o
	g++ -L/usr/local/frc/lib -lntcore -o $@ $^
