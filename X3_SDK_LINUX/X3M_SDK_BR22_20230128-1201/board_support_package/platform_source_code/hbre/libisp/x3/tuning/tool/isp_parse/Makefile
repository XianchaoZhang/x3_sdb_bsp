#use this makefile need libxml2 and libz.so

CFLAGS := -O2
SRC = isp_json_parse.c cJSON.c
TAR_OBJ = isp_parse
INC = -I ./inc/ -I ./inc/libxml2/
LDFLAGS = -L ./lib -lxml2 -lz -lm -Wl,-rpath=../isp_parse/lib/ -Wl,-rpath=./lib/
OBJS = $(SRC:.c=.o)

%.o:%.c
	gcc $(CFLAGS) $(INC) -g -o $@ -fpic -c $<
all :$(TAR_OBJ)
$(TAR_OBJ): $(OBJS)
	gcc -o $@ $(OBJS) $(STATIC_LIB) $(LDFLAGS)
	cp $(TAR_OBJ) ../generate_lib/
	mkdir -p ../generate_lib/lib
	cp lib/libxml2.so.2 ../generate_lib/lib/
	ln -sf ./libxml2.so.2 ../generate_lib/lib/libxml2.so
clean:
	rm -rf $(TAR_OBJ) $(OBJS)
