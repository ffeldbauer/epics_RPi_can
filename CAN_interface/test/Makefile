CXX		= g++
ifneq ($(CROSS_COMPILE),)
	CXX=$(CROSS_COMPILE)g++
endif
REMOVE		= rm -f
REMOVEDIR	= rm -rf
COPY		= cp

EXTRAINCDIRS 	= 
EXTRALIBDIRS 	= 

OPT 		= 

CPPFLAGS 	+= -O$(OPT) -Wall
CPPFLAGS 	+= $(patsubst %,-I%,$(EXTRAINCDIRS))

LDFLAGS 	+= $(patsubst %,-L%,$(EXTRALIBDIRS))
LDFLAGS 	+= -lsocketcan

SRC 		= main.cxx
SRC 		+= CanTest.cxx

OBJ = $(SRC:%.cxx=%.o)

TARGET		= rpi_can_test

MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_CLEANING = Cleaning project:
MSG_COMPILING_CPP = Compiling C++:
MSG_LINKING = Linking:

## Build
all: begin gccversion build end

build: $(TARGET)

begin:
		@echo
		@echo $(MSG_BEGIN)

end:
		@echo $(MSG_END)
		@echo

# Display compiler version information.
gccversion: 
		@$(CXX) --version

%.o : %.cxx
	@echo
	@echo $(MSG_COMPILING_CPP) $<
	$(CXX) -c $(CPPFLAGS) $< -o $@ 

$(TARGET):	$(OBJ)
		@echo
		@echo $(MSG_LINKING) $@
		$(CXX) $^ $(CPPFLAGS) $(LDFLAGS) -o $@

install: begin install_do end

install_do:
	sudo $(COPY) $(TARGET) /usr/local/bin/.
	sudo chmod a+x /usr/local/bin/$(TARGET)

uninstall:
	@-sudo rm -f /usr/local/bin/$(TARGET)

clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET)
	$(REMOVE) $(OBJ)

# Create object files directory
$(shell mkdir $(OBJDIR) 2>/dev/null)

# Listing of phony targets.
.PHONY : all begin end gccversion build clean clean_list install install_do uninstall
