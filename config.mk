# DIRS
OBJ_DIR = $(PWD)/obj
BIN_DIR = $(PWD)/bin
LIB_DIR = $(PWD)/lib

# INCLUDE AND LIBRARY PATHS
INCLUDES = -I/usr/include -I/usr/local/include -I$(PWD)/include
LIBS = -L/usr/lib -L/usr/local/lib -L$(LIB_DIR) -lawesomo `pkg-config --libs --cflags opencv`

# C COMPILER
CC = g++
DEBUG_FLAGS = -g
WARN_FLAGS = -Wall
CFLAGS = $(INCLUDES) $(DEBUG_FLAGS) $(WARN_FLAGS)

# ARCHIVER
AR = ar
ARFLAGS = rvs

# COMMANDS
MAKE_OBJ = \
	echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $(addprefix $(OBJ_DIR)/, $@);

MAKE_TEST = \
	echo "CC [$@.c]"; \
	$(CC) -c $@.cpp -o $(OBJ_DIR)/$@.o $(CFLAGS); \
	$(CC) $(OBJ_DIR)/$@.o -o $(BIN_DIR)/$@ $(LIBS);

MAKE_EXE = \
	echo "CC [$@.c]"; \
	$(CC) -c $@.cpp -o $(OBJ_DIR)/$@.o $(CFLAGS); \
	$(CC) $(OBJ_DIR)/$@.o -o $(BIN_DIR)/$@ $(LIBS);

MAKE_STATIC_LIB = \
	echo "AR [$@]"; \
	$(AR) $(ARFLAGS) $(LIB_DIR)/$@.a $(wildcard $(OBJ_DIR)/*.o);
