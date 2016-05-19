include config.mk

CCACHE_EXISTS = $(shell ccache -V)
ifdef CCACHE_EXISTS
	CC := ccache $(CC)
	CXX := ccache $(CXX)
endif


default: all done

all: mkdirs libawesomo tests tools

libawesomo: mkdirs
	@make -s -C src

tests: mkdirs libawesomo
	@make -s -C tests

tools: mkdirs libawesomo
	@make -s -C tools

mkdirs:
	@mkdir -p $(LIB_DIR)
	@mkdir -p $(LIB_BUILD_DIR)
	@mkdir -p $(TESTS_BUILD_DIR)
	@mkdir -p $(TESTS_BIN_DIR)
	@mkdir -p $(TOOLS_BUILD_DIR)
	@mkdir -p $(TOOLS_BIN_DIR)

rmdirs:
	@rm -rf $(LIB_DIR)
	@rm -rf $(LIB_BUILD_DIR)
	@rm -rf $(TESTS_BUILD_DIR)
	@rm -rf $(TESTS_BIN_DIR)
	@rm -rf $(TOOLS_BUILD_DIR)
	@rm -rf $(TOOLS_BIN_DIR)

clean: rmdirs
	@echo "cleaning ..."
	@echo "done! :)"

imu_transmit:
	@./build/bin/tools/imu_visualizer ${IP}

<<<<<<< HEAD
imu_client:
=======
imu_visualize:
>>>>>>> 2125b8ae1ae741a09b52aa42615ba5d227a76505
	@python scripts/navio/imu_visualizer.py 7000 imu

done:
	@echo "done! :)"
