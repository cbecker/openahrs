#GCC_PREFIX	= avr32-linux-
GCC_PREFIX	=

include Makefile.paths

CXX=$(GCC_PREFIX)g++
AR=$(GCC_PREFIX)ar
CC=$(GCC_PREFIX)gcc

CXXFLAGS=-O3 -Wall

OPENIMU_PATH	= $(realpath ./openIMU)
OPENIMU_LIB		= $(OPENIMU_PATH)/openIMU.a
OPENIMU_INC		= $(OPENIMU_PATH)/include

## change that if you want to try floating point results
FLOAT_TYPE=double


## Prepare basic build variables
all:	openIMU/openIMU.a
	@echo	done.

openIMU/openIMU.a:	Makefile.build
	@echo ---=== Compiling openIMU ===---
	make -C openIMU
	
test-kal7: openIMU/openIMU.a Makefile.build
	@echo ---=== Building test-kal7 ===---
	make -C tests/test-kal7

test-eigen2: Makefile.build
	@echo ---=== Building test-eigen2 ===---
	make -C tests/test-eigen2

##prepare neccesary files for build
Makefile.build:	Makefile
	@echo EIGENPATH=$(realpath $(EIGENPATH)) > Makefile.build
	@echo CXXFLAGS += -DFT=$(FLOAT_TYPE)	>> Makefile.build
	@echo CXX=$(CXX) >> Makefile.build
	@echo AR=$(AR)	>> Makefile.build
	@echo CC=$(CC)	>> Makefile.build
	@echo CXXFLAGS +=$(CXXFLAGS) >> Makefile.build
	@echo OPENIMU_LIB = $(OPENIMU_LIB) >> Makefile.build
	@echo OPENIMU_INC = $(OPENIMU_INC) >> Makefile.build



clean:	Makefile.build
	make clean	-C tests/test-kal7
	make clean	-C tests/test-eigen2
	make clean 	-C openIMU
	rm Makefile.build

help:
	@echo
	@echo	___--=== openIMU Makefile ===--___
	@echo	
	@echo	CAUTION: Please setup the EIGENPATH variable in Makefile.paths first!!
	@echo
	@echo
	@echo	== To compile the main library just type \'make\' with no arguments.
	@echo
	@echo 	== To compile examples you can try \'make example_name\'
	@echo		where example_name is one of
	@echo
	@echo		test-eigen2
	@echo		test-kal7
	@echo


.PHONY: tests test-kal7 help
