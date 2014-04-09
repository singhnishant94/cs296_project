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
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing -p
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include#729fcf
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib -p
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
	&& cmake -DCMAKE_BUILD_TYPE=Release ../ \
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

profile: setup exe
	@perf record -g -- $(PROJECT_ROOT)/mybins/$(TARGET) auto 50000 >/dev/null
	@perf script | python gprof2dot.py -f perf | dot -Tpng -o prof_graph.png
	



clean:
	@rm -rf cs296_24_project
	@rm -rf g24_project
	@rm -rf mylibs/* myobjs/* mybins/*
	@rm -rf data
	@rm -rf plots
	@rm -rf ./scripts/temp1.dat
	@rm -rf fit.log load
	@rm -rf ./scripts/temp1.dat
	@rm -rf g24_release_prof.dat
	@rm -rf g24_debug_prof.dat
	@rm -rf release.png
	@rm -rf debug.png
	@rm -rf gmon.out
	@rm -rf ./doc/*.aux
	@rm -rf ./doc/*.log
	@rm -rf ./doc/*.pdf
	@rm -rf ./doc/*.html
	
distclean: clean
	@rm -rf mylibs myobjs mybins
	@rm -rf $(EXTERNAL)/src/Box2D
	@rm -rf $(EXTERNAL)/lib/Box2D 
	@rm -rf $(EXTERNAL)/include/Box2D
	@rm -rf cs296_g24_project

dist: distclean
	@tar -cf temp.tar *
	@mkdir g24_project
	@tar -xf temp.tar -C g24_project
	@mkdir -p cs296_g24_project
	@mv g24_project cs296_g24_project
	@rm -rf temp.tar
	@tar czf cs296_g24_project.tar.gz cs296_g24_project
	@rm -rf cs296_g24_project

report:
	@cd  $(PROJECT_ROOT)/doc \
	&& pdflatex  g24_project_report.tex \
	&& bibtex  g24_project_report \
	&& pdflatex  g24_project_report.tex \
	&& pdflatex g24_project_report \
	&& rm  -rf  g24_project_report.bbl g24_project_report.blg g24_project_report.aux g24_project_report.log g24_project_report.dvi;\

gencsv:
	@mkdir -p data
	@cd  $(PROJECT_ROOT)/scripts \
	&& python g24_gen_csv.py \

plot:
	@mkdir -p data
	@mkdir -p plots 
	@if ! test -e $(PROJECT_ROOT)/mybins/cs296_24_exe ;\
	then make exe;\
	fi;
	@cd  $(PROJECT_ROOT)/scripts \
	&& python g24_gen_plots.py \
