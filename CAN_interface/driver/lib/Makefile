CXX		= g++
REMOVE		= rm -f
REMOVEDIR	= rm -rf
COPY		= cp
SYMLINK		= ln

LIBPATH		= /usr/lib

LDNAME		= libpican.so
SONAME		= $(LDNAME).1
TARGET		= $(SONAME).0

EXTRAINCDIRS 	= 
EXTRALIBDIRS 	= 

OPT 		= 

CPPFLAGS	+= -fPIC -shared
CPPFLAGS 	+= -O$(OPT) -Wall -Wl,-soname,$(SONAME)
CPPFLAGS 	+= $(patsubst %,-I%,$(EXTRAINCDIRS))

LDFLAGS 	+= $(patsubst %,-L%,$(EXTRALIBDIRS))
LDFLAGS 	+= -lc

SRC 		= libpican.c

MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:

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

$(TARGET):	$(SRC)
		@echo
		@echo $(MSG_CREATING_LIBRARY) $@
		$(CXX) $< $(CPPFLAGS) $(LDFLAGS) -o $@

install: begin install_do end

install_do:
	sudo $(COPY) $(TARGET) $(LIBPATH)/$(TARGET)
	sudo $(SYMLINK) -sf $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	sudo $(SYMLINK) -s $(LIBPATH)/$(SONAME) $(LIBPATH)/$(LDNAME)
	sudo $(COPY) libpican.h /usr/include/libpican.h
	sudo chmod 644 /usr/include/libpican.h
	sudo /sbin/ldconfig

uninstall:
	@-sudo rm -f $(LIBPATH)/$(TARGET)
	@-sudo rm -f $(LIBPATH)/$(SONAME) $(LIBPATH)/$(LDNAME)
	@-sudo rm -f /usr/include/libpican.h
	sudo /sbin/ldconfig


clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET)

# Create object files directory
$(shell mkdir $(OBJDIR) 2>/dev/null)

# Listing of phony targets.
.PHONY : all begin end gccversion build clean clean_list install install_do uninstall
