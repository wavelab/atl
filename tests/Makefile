include $(PWD)/config.mk

SRCS = $(wildcard *.cpp)
TESTS = $(SRCS:.cpp=)

default: $(TESTS)

%: %.cpp
	$(MAKE_TEST)
