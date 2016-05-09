include $(PWD)/config.mk
FOLDER_NAME = src

SRCS = $(wildcard *.cpp)
OBJS = $(addprefix $(LIB_BUILD_DIR)/, $(SRCS:.cpp=.o))

default: $(LIB_DIR)/libawesomo.a

$(LIB_BUILD_DIR)/%.o: %.cpp
	$(MAKE_OBJ)

$(LIB_DIR)/libawesomo.a: $(OBJS)
	$(MAKE_STATIC_LIB)
