include $(PWD)/config.mk
FOLDER_NAME = tools
SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
OBJS := $(addprefix $(TOOLS_BUILD_DIR)/, $(OBJS))

default: $(OBJS)

$(TOOLS_BUILD_DIR)/%.o: %.cpp $(LIB_DIR)/libawesomo.a
	$(MAKE_TOOL)
