C_SOURCE=${wildcard *.c}
C_OBJS=${patsubst %.c, %.o, $(C_SOURCE)}

CPP_SOURCE=${wildcard *.cpp}
CPP_OBJS=${patsubst %.cpp, %.o, $(CPP_SOURCE)}

all:PRINT $(C_OBJS) $(CPP_OBJS)
PRINT:
	@echo
	@echo CFLAG = $(CFLAGS)
	@echo
	@echo INCLUDE    = $(INC)
	@echo
$(C_OBJS):%.o:%.c
	@echo CC $^ -o $@
	@$(CC) $(CFLAGS) $(INC) -c $^ -o $(PROJECT_DIR)$(OBJS_DIR)/$@

$(CPP_OBJS):%.o:%.cpp
	@echo CC $^ -o $@
	@$(CPP) $(CFLAGS) $(INC) -c $^ -o $(PROJECT_DIR)$(OBJS_DIR)/$@
