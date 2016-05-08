# DIRS
OBJ_DIR = $(PWD)/obj
BIN_DIR = $(PWD)/bin
LIB_DIR = $(PWD)/lib

# INCLUDE AND LIBRARY PATHS
INCLUDES = \
	-I/usr/include \
	-I/usr/include/eigen3 \
	-I/usr/include/flycapture \
	-I/usr/local/include \
	-I$(PWD)/include/ \
	-I$(PWD)/include/awesomo \
	-I$(PWD)/include/awesomo/navio

LIBS = \
	-L/usr/lib \
	-L/usr/local/lib \
	-L$(LIB_DIR) \
	-lawesomo \
	-lflycapture \
	-lEigen3\
	-lm

# C COMPILER
CC = g++
# STANDARD = -std=gnu99
DEBUG_FLAGS = -g
WARN_FLAGS = -Wall
CFLAGS = $(INCLUDES) $(STANDARD) $(DEBUG_FLAGS) $(WARN_FLAGS)

# ARCHIVER
AR = ar
ARFLAGS = rvs

# COMMANDS
MAKE_OBJ = \
	echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $(addprefix $(OBJ_DIR)/, $@);

MAKE_TEST = \
	echo "CC [$@.cpp]"; \
	$(CC) -c $@.c -o $(OBJ_DIR)/$@.o $(CFLAGS); \
	$(CC) $(OBJ_DIR)/$@.o -o $(BIN_DIR)/$@ $(LIBS);

MAKE_EXE = \
	echo "CC [$@.cpp]"; \
	$(CC) -c $@.c -o $(OBJ_DIR)/$@.o $(CFLAGS); \
	$(CC) $(OBJ_DIR)/$@.o -o $(BIN_DIR)/$@ $(LIBS);

MAKE_STATIC_LIB = \
	echo "AR [$@]"; \
	$(AR) $(ARFLAGS) $(LIB_DIR)/$@.a $(wildcard $(OBJ_DIR)/*.o);
