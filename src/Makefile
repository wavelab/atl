include $(PWD)/config.mk
FOLDER_NAME = src

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)

default: src libawesomo

%.o: %.cpp
	$(MAKE_OBJ)

src: $(OBJS)

libawesomo: $(OBJS)
	$(MAKE_STATIC_LIB)
