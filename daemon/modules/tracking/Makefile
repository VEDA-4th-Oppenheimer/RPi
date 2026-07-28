CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -lm

TARGET  = v_servo_test
SRCS    = v_servo_test.c visual_servo.c
OBJS    = $(SRCS:.c=.o)
DEPS    = visual_servo.h

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean