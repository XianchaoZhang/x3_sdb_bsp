include  ${SRC_BUILD_DIR}/core/Makefile

LIB_OBJECT = j2cf
CROSS_COMPILE=
SRCS = $(wildcard ./*.c)

LIBS += -lpthread

OBJS = $(SRCS:.c=.o)

%.o:%.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -g -o $@ -fpic -c $<

.PHONY: all all_32 clean
all :$(LIB_OBJECT)
$(LIB_OBJECT): $(OBJS)
	$(CROSS_COMPILE)gcc $(LDFLAGS) $(LIBS) -static -o $@ $(OBJS)
	rm $(OBJS)

clean:
	rm -rf $(LIB_OBJECT) $(OBJS)
