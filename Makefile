include config.mk

default: all done

all: mkdirs libawesomo tests calibration

libawesomo: mkdirs
	@make -s -C src

tests: mkdirs libawesomo
	@make -s -C tests

calibration: mkdirs libawesomo
	@make -s -C calibration

mkdirs:
	@mkdir -p $(LIB_DIR)
	@mkdir -p $(LIB_BUILD_DIR)
	@mkdir -p $(TESTS_BUILD_DIR)
	@mkdir -p $(TESTS_BIN_DIR)
	@mkdir -p $(CALIBRATION_BUILD_DIR)
	@mkdir -p $(CALIBRATION_BIN_DIR)

rmdirs:
	@rm -rf $(LIB_DIR)
	@rm -rf $(LIB_BUILD_DIR)
	@rm -rf $(TESTS_BUILD_DIR)
	@rm -rf $(TESTS_BIN_DIR)
	@rm -rf $(CALIBRATION_BUILD_DIR)
	@rm -rf $(CALIBRATION_BIN_DIR)

clean: rmdirs
	@echo "cleaning ..."
	@echo "done! :)"

done:
	@echo "done! :)"
