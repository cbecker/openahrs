include ../Makefile.build

TARGET = openIMU.a

INC_PATH = ./include/ $(EIGENPATH)

CXXFLAGS += $(addprefix -I , $(INC_PATH) )

SRC = $(addprefix src/, \
			kalman/kalman7.cpp \
			util/util.cpp \
		)


SOURCES = $(SRC)

OBJS = $(SOURCES:.cpp=.o)

########################################################
############# RULES ###################################
########################################################

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING_C = Compiling C:
MSG_COMPILING_CPP = Compiling C++:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:


all:	$(TARGET)	Makefile	../Makefile.build
	@echo
	@echo $(MSG_END)

$(TARGET):	$(OBJS)		Makefile	../Makefile.build
	@echo
	@echo $(MSG_CREATING_LIBRARY)
	$(AR) rs $(TARGET) $(OBJS)

$(OBJS):	%.o:%.cpp	Makefile ../Makefile.build
	@echo
	@echo $(MSG_COMPILING_CPP) $<
	$(CXX) $(CXXFLAGS) -c $< -o $@

test:
	@echo $(EIGENPATH)

clean:
	@echo
	@echo $(MSG_CLEANING)
	rm -f $(OBJS)
	rm -f $(TARGET)
	rm -r -f .dep



# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

