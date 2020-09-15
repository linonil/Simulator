CFLAGS = -Wall `pkg-config --cflags --libs opencv` -c -o
LFLAGS = -Wall `pkg-config --cflags --libs opencv` -o

genTest: genTest.o square.o ball.o
	g++ $^ $(LFLAGS) $@

genTest.o: genTest.cpp
	g++ $< $(CFLAGS) $@

square.o: square.cpp square.hpp
	g++ $< $(CFLAGS) $@

ball.o: ball.cpp ball.hpp
	g++ $< $(CFLAGS) $@

clean:
	rm *.o

run:
	./genTest refImage.png t_0_0_500.txt k1.txt 640x480
