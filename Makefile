include config.mk

default: all done

all: rmdirs mkdirs libawesomo

libawesomo: mkdirs
	@make -s -C src

tests: mkdirs
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

clean: rmdirs
	@echo "cleaning ..."
	@echo "done! :)"

done:
	@echo "done! :)"
