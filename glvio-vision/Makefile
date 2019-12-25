INC = -I./src 
SRC = $(wildcard src/*.c)
OBJ = $(patsubst %.c,%.o,$(SRC))

CUR_DIR=`pwd`
CROSS_TOOL = ${CUR_DIR}/../out/sun8iw8p1/linux/common/buildroot/external-toolchain/bin/arm-buildroot-linux-musleabi-


BIN=./glvio-vision

CC = $(CROSS_TOOL)gcc
STRIP = $(CROSS_TOOL)strip
CFLAGS += -Os -march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=softfp -ffast-math #-D__ARM_NEON__
CFLAGS += -pipe -Werror=format-security -fno-caller-saves -D_FORTIFY_SOURCE=1 -Wl,-z,now -Wl,-z,relro
CFLAGS += -Wall
# 注意库的引入顺序：被依赖的库放到后面！
LIB += -lpthread -lrt -lm

all : $(OBJ)
	$(CC) $(CFLAGS) $(INC) $(OBJ) $(LIB) -o $(BIN)
	@echo 
	@echo "build flow successfully"
	@echo 

%.o : %.c
	$(CC) $(CFLAGS) $(INC) -c $^ -o $@

clean:
	@rm -f src/*.o
	@rm -f $(BIN)

