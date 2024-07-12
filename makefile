CC = gcc
CFLAGS = -g -Wall
LDFLAGS = -lm
TARGET = gimbalTrajectory

all: $(TARGET)

$(TARGET): gimbalTrajectory.c
	$(CC) $(CFLAGS) -o $(TARGET) gimbalTrajectory.c $(LDFLAGS)

clean:
	rm $(TARGET)



