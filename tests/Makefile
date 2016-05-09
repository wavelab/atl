include $(PWD)/config.mk
FOLDER_NAME = tests
SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
OBJS := $(addprefix $(TESTS_BUILD_DIR)/, $(OBJS))

default: $(OBJS)

$(TESTS_BUILD_DIR)/%.o: %.cpp
	$(MAKE_TEST)
