# DIRS
BUILD_DIR = $(PWD)/build
BIN_DIR = $(BUILD_DIR)/bin
LIB_DIR = $(BUILD_DIR)/lib
LIB_BUILD_DIR = $(BUILD_DIR)/awesomo
TESTS_BIN_DIR = $(BIN_DIR)/tests
TESTS_BUILD_DIR = $(BUILD_DIR)/tests
CALIBRATION_BIN_DIR = $(BIN_DIR)/calibration
CALIBRATION_BUILD_DIR = $(BUILD_DIR)/calibration


# INCLUDE AND LIBRARY PATHS
INCLUDES = \
	-I/usr/include \
	-I/usr/include/eigen3 \
	-I/usr/include/flycapture \
	-I/usr/local/include/navio2 \
	-I$(PWD)/include/

LIBS = \
	-L/usr/lib \
	-L/usr/local/lib \
	-L$(LIB_DIR) \
	-lawesomo \
	-lflycapture \
	-lapriltags \
	-lyaml-cpp \
	-lnavio2 \
	-lm \
	`pkg-config --libs --cflags opencv`


# C COMPILER
CC = g++
STANDARD = -std=c++11
DEBUG_FLAGS = -g
WARN_FLAGS = -Wall -Wno-unknown-pragmas
CFLAGS = $(INCLUDES) $(STANDARD) $(DEBUG_FLAGS) $(WARN_FLAGS)


# ARCHIVER
AR = ar
ARFLAGS = rvs


# COMMANDS
MAKE_OBJ = \
	echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $@;

MAKE_TEST = \
	echo "TEST [$<]"; \
	$(CC) -c $< -o $@ $(CFLAGS); \
	$(CC) $@ -o $(addprefix $(TESTS_BIN_DIR)/, $(notdir $(@:.o=))) $(LIBS);

MAKE_CALIBRATION = \
	echo "EXE [$<]"; \
	$(CC) -c $< -o $@ $(CFLAGS); \
	$(CC) $@ -o $(addprefix $(CALIBRATION_BIN_DIR)/, $(notdir $(@:.o=))) $(LIBS);

MAKE_STATIC_LIB = \
	echo "AR [$@]"; \
	$(AR) $(ARFLAGS) $@ $(wildcard $(LIB_BUILD_DIR)/*.o);
