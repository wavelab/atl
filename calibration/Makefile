include $(PWD)/config.mk
FOLDER_NAME = calibration
SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
OBJS := $(addprefix $(CALIBRATION_BUILD_DIR)/, $(OBJS))

default: $(OBJS)

$(CALIBRATION_BUILD_DIR)/%.o: %.cpp $(LIB_DIR)/libawesomo.a
	$(MAKE_CALIBRATION)
