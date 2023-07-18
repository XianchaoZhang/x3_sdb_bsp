CFLAGS  := -O2
LIB_OBJECT = libimx415_linear.so
CROSS_COMPILE=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
SRCS = $(wildcard ./*.c)

LDFLAGS = -lpthread -fPIC -shared

OBJS = $(SRCS:.c=.o)

%.o:%.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -g -o $@ -fpic -c $<

.PHONY: all all_32 clean
all :$(LIB_OBJECT)
$(LIB_OBJECT): $(OBJS)
	$(CROSS_COMPILE)gcc $(LDFLAGS) -o $@ $(OBJS)

clean:
	rm -rf $(LIB_OBJECT) $(OBJS)
