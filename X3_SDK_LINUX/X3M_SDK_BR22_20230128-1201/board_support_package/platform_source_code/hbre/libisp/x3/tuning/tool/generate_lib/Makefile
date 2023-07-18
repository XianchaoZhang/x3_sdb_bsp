include  ${SRC_BUILD_DIR}/core/Makefile

J2CS = $(wildcard *_dynamic.json)
CSOURCE = $(subst _dynamic.json,_calibration.c,$(J2CS))
LNAME = $(subst _dynamic.json,,$(J2CS))
LIB_OBJECT = $(addsuffix .so,$(addprefix lib,$(LNAME)))
.PRECIOUS: %.c
ISP_PARSE_BIN = "IV009-SW-Control.bin"
ISP_PARSE_JSON = "settings.json"
ISP_PARSE_REG = "need_select_register.txt"
#ISP_PARSE_EN = "enable"
ISP_PARSE_LUT_IN = "8888.3dl"
ISP_PARSE_LUT_OUT = "test3dlut.txt"
#ISP_PARSE_LUT_EN = "enable"

LIBS += -lpthread -lm

OBJS = $(subst .c,.o,$(CSOURCE))

.PHONY: all all_32 clean
all all_32:$(LIB_OBJECT)
	tar -zcf $(shell date +%Y%m%d_%H%M)_lib.tar.gz * 

%.c:
	./j2cf -d "./$(subst _calibration.c,,$@)_dynamic.json" -s "./$(subst _calibration.c,,$@)_static.json" -n "$(subst _calibration.c,,$@)"

        ifeq ($(ISP_PARSE_EN), "enable")
	./isp_parse -m 0 -r $(ISP_PARSE_BIN) -s $(ISP_PARSE_JSON) -n $(ISP_PARSE_REG) -c $@ -e 1
        endif
        ifeq ($(ISP_PARSE_LUT_EN), "enable")
	./isp_parse -m 1 -l $(ISP_PARSE_LUT_IN) -o $(ISP_PARSE_LUT_OUT)
        endif

%.o:%.c
	$(CC) $(CFLAGS) -g -o $@ -fPIC -c $<

$(LIB_OBJECT):$(OBJS)
	$(CC) $(CFLAGS_DYNAMIC) $(LDFLAGS) -o $@ $(subst .so,_calibration.o,$(subst lib,, $@))
#rm $(subst .so,_calibration.o,$(subst lib,, $@))

clean:
	rm -rf $(LIB_OBJECT) $(OBJS)
