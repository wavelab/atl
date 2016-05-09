include config.mk

default: all done

all: mkdirs libawesomo tests

libawesomo: mkdirs
	@make -s -C src

tests: mkdirs
	@make -s -C tests

debug: mkdirs
	@make -C src
	@make -C tests

mkdirs:
	@mkdir -p $(LIB_DIR)
	@mkdir -p $(LIB_BUILD_DIR)
	@mkdir -p $(TESTS_BUILD_DIR)
	@mkdir -p $(TESTS_BIN_DIR)

rmdirs:
	@rm -rf $(LIB_DIR)
	@rm -rf $(LIB_BUILD_DIR)
	@rm -rf $(TESTS_BUILD_DIR)
	@rm -rf $(TESTS_BIN_DIR)

clean: rmdirs
	@echo "cleaning ..."
	@echo "done! :)"

done:
	@echo "done! :)"
