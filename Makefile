#Variables
DOXYGEN = doxygen 
ECHO	= /bin/echo
#Project root location
PROJECT_ROOT=./
SRCDIR=$(PROJECT_ROOT)/src
OBJDIR=$(PROJECT_ROOT)/myobjs
#External Location
EXTERNAL=$(PROJECT_ROOT)/external

# Library Paths
BOX2D_ROOT=$(EXTERNAL)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

#Compiler and linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include#729fcf
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
LDFLAGS_N=$(LDFLAGS) -L $(PROJECT_ROOT)/mylibs -Wl,-R$(PROJECT_ROOT)/mylibs

OK="[OK]"
ERR="[ERRORS]"
WARN="[WARNINGs]"

TARGET=cs296_24_exe

SHARED_LIB=FALSE

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

NEW_OBJS := $(filter-out $(PROJECT_ROOT)/myobjs/main.o,$(OBJS))

EXELIB=$(PROJECT_ROOT)/mybins/cs296_24_exelib

.PHONY: all setup exe doc distclean clean

all: setup

setup :
	@mkdir -p myobjs mylibs mybins
	@if test -e $(EXTERNAL)/include/Box2D -a -e $(EXTERNAL)/lib/Box2D;\
	then echo "Box2D has already been installed";\
	else tar xzf $(PROJECT_ROOT)/external/src/Box2D.tgz -C $(EXTERNAL)/src \
	&& printf "Installing Box2D" \
	&& cd $(EXTERNAL)/src/Box2D \
	&& mkdir -p build296 \
	&& cd build296 \
	&& cmake ../ \
	&& make	\
	&& make install \
	&& cd .. && cd .. && cd .. && cd .. ;\
	fi;

$(OBJS) : $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@printf "Compiling: %25s" "$(notdir $<)"
	@g++ -fPIC $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then printf "%30s\n" $(ERR) && cat temp.log; \
	elif test -s temp.log;\
	then printf "%30s\n" $(WARN) && cat temp.log; \
	else printf "%30s\n" "[OK]"; \
	fi;
	@rm -f temp.log temp.err

exe: $(OBJS)
	@printf "Creating Executable now.."
	@g++ -o $(PROJECT_ROOT)/mybins/$(TARGET) $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then printf "%30s\n" $(ERR) && cat temp.log; \
	elif test -s temp.log;\
	then printf "%30s\n" $(WARN) && cat temp.log; \
	else printf "%30s\n" "[OK]"; \
	fi;
	@rm -f temp.log temp.err

staticl:
	@if test $(SHARED_LIB) = FALSE; \
	then ar cq $(PROJECT_ROOT)/mylibs/libCS296test.a $(NEW_OBJS); \
	else make dynamicl; \
	fi;

dynamicl:
	@g++ -shared -Wl,-soname,$(PROJECT_ROOT)/mylibs/libCS296test.so \
	-o $(PROJECT_ROOT)/mylibs/libCS296test.so $(NEW_OBJS)

exelib: $(OBJS) staticl
	@g++ -o $(EXELIB) $(LDFLAGS_N) $(PROJECT_ROOT)/myobjs/main.o -lCS296test $(LIBS)

doc:
	@echo "Generating Doxygen Documentation ...  "
	@rm -rf doc/html
	@$(DOXYGEN) $(PROJECT_ROOT)/doc/Doxyfile 2 > /dev/null
	@echo "Done"


clean:
	@rm -rf mylibs/* myobjs/* mybins/*
	
distclean:
	@rm -rf mylibs myobjs mybins
	@rm -rf $(EXTERNAL)/src/Box2D
	@rm -rf $(EXTERNAL)/lib/Box2D 
	@rm -rf $(EXTERNAL)/include/Box2D

report:
	@cd  $(PROJECT_ROOT)/doc \
	&& latex  cs296_report_24 \
	&& bibtex  cs296_report_24 \
	&& latex  cs296_report_24 \
	&& latex cs296_report_24 \
	&& dvipdfm cs296_report_24.dvi \
	&& rm  -rf  cs296_report_24.bbl cs296_report_24.blg cs296_report_24.aux cs296_report_24.log cs296_report_24.dvi;\


