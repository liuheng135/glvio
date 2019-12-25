INC = -I. 
SRC = $(wildcard ./*.c)
OBJ = $(patsubst %.c,%.o,$(SRC))

CUR_DIR=`pwd`
CROSS_TOOL=

BIN=./glvio-viewer

CC = $(CROSS_TOOL)gcc
STRIP = $(CROSS_TOOL)strip
CFLAGS += -Os 
CFLAGS += -pipe -Werror=format-security -fno-caller-saves -D_FORTIFY_SOURCE=1 -Wl,-z,now -Wl,-z,relro
CFLAGS += -Wall
# 注意库的引入顺序：被依赖的库放到后面！
LIB += -lrt -lm

all : $(OBJ)
	$(CC) $(CFLAGS) $(INC) $(OBJ) $(LIB) -o $(BIN)
	@echo 
	@echo "build flow successfully"
	@echo 

%.o : %.c
	$(CC) $(CFLAGS) $(INC) -c $^ -o $@

clean:
	@rm -f *.o
	@rm -f $(BIN)

