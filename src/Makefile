include $(PWD)/config.mk
FOLDER_NAME = src

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)

default: libawesomo

%.o: %.cpp
	$(MAKE_OBJ)

libawesomo: $(OBJS)
	$(MAKE_STATIC_LIB)
