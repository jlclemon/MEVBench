#Base Makefile

#---------PROJECT FILES---------

CPPFILES= HessKnnMatcher.cpp KnnFlannClassification.cpp MatcherResultsClassifier.cpp MultiThreadedMatResult.cpp SparseFeaturePointObjectClassifier.cpp ThreadManager.cpp
CCFILES= 
CFILES= 
FILES = $(CPPFILES) $(CCFILES) $(CFILES)
EXECUTABLEBASE=FeatureClassification
#-------
MAIN= FeatureClassification.cpp
HEADERS=
#The include director for openCV
INCDIRSBASE=$(OpenCV_DIR)/include
CCFILEXTENSION=.cpp
CCFILEXTENSION2=.cc
CFILEXTENSION=.c
CC=g++
C=gcc
CCDEBUG=-g -p -pg
CDEBUG=-g -p -pg
CCFLAGS=-c -Wall $(CCDEBUG) -O3 $(XTRA_PARAMS)
CFLAGS=-c -Wall $(CDEBUG) -O3 $(XTRA_PARAMS)
LDEBUG=$(CCDEBUG)
CDEFINES=
CCDEFINES=
LLFLAGSBASE= -Wall -o
LIBSBASE=pthread rt opencv_calib3d opencv_core opencv_features2d opencv_features2d opencv_flann opencv_highgui opencv_imgproc opencv_ml opencv_objdetect opencv_video
#opencv_features2d
#The lib path for the opencv libraries assuming you don't have a native install
LIBPATHS=$(OpenCV_DIR)/lib
HEADERPATH=
BINDIR=./bin
OBJDIR=./obj

#LFLAGS=$(LIBS) $(DEBUG) -m32 -L$(LIBPATH) -Wall -o 
#TESTCASES= ./testcase/*.cc

#############################################################################################
#do not edit below this line
#############################################################################################
INCDIRS=$(addprefix -I,$(INCDIRSBASE))
LIBS=$(addprefix -l,$(LIBSBASE))
LIBPATH=$(addprefix -L,$(LIBPATHS))



BASEOBJECTS=$(basename $(FILES))
BASEOBJECTSPATH=$(addprefix $(OBJDIR)/,$(BASEOBJECTS))
OBJECTS=$(addsuffix .o,$(BASEOBJECTSPATH))


BASEMAINOBJECT=$(basename $(MAIN))
MAINOBJECTPATH=$(addprefix $(OBJDIR)/,$(BASEMAINOBJECT))
MAINOBJECT=$(addsuffix .o,$(MAINOBJECTPATH))

LLFLAGS=$(LIBS) $(LDEBUG) $(LIBPATH) $(LLFLAGSBASE)  

EXECUTABLE=$(BINDIR)/$(EXECUTABLEBASE)

#CFLAGS=$(CCFLAGS)
#LFLAGS=$(LLFLAGS)
#LIB=lib$(LIBRARY)

all: $(EXECUTABLE)
	

nolink: $(OBJECTS)

#compiles the executable which is dependent on the objects
$(EXECUTABLE): $(OBJECTS) $(MAINOBJECT)
	@echo $(OBJECTS)
	@echo "linking files"
	$(CC) $(OBJECTS) $(MAINOBJECT) $(LLFLAGS) $@
	@echo "build finished"

#compiles the objects into executables
$(OBJDIR)/%.o: %$(CFILEXTENSION)
	@echo "building changed c files"
	$(C) $(CFLAGS) $(CDEFINES) $(INCDIRS) -o$@ $< 
#compiles the objects into executables
$(OBJDIR)/%.o: %$(CCFILEXTENSION)
	@echo "building changed cpp files"
	$(CC) $(CCFLAGS) $(CCDEFINES) $(INCDIRS) -o$@ $<

#compiles the objects into executables
$(OBJDIR)/%.o: %$(CCFILEXTENSION2)
	@echo "building changed cc files"
	$(CC) $(CFLAGS) $(CCDEFINES) $(INCDIRS) -o$@ $<
	
	
.h.o:
	@echo "building changed header files"
	$(CC) $(CFLAGS) $(FILES)

.hpp.o:
	@echo "building changed header files"
	$(CC) $(CFLAGS) $(FILES)



#---------------3rd way------------------

clean:
	rm -f $(OBJDIR)/*.o 
	rm -f $(EXECUTABLE)





