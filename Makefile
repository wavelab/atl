include config.mk

default: all done

all: rmdirs mkdirs libawesomo libawesomo_tests

libawesomo: mkdirs
	@make -s -C src

libawesomo_tests: mkdirs
	@make -s -C tests

debug: mkdirs
	@make -C src
	@make -C tests

mkdirs: rmdirs
	@mkdir bin
	@mkdir obj
	@mkdir lib

rmdirs:
	@rm -rf bin
	@rm -rf obj
	@rm -rf lib
	@rm -rf tests/*.dSYM

run_tests: all done
	@./tests/unittest_runner.py

clean: rmdirs
	@echo "cleaning ..."
	@echo "done! :)"

done:
	@echo "done! :)"
