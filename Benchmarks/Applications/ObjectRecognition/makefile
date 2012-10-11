#Base Makefile

MEVBENCH_FEATURE_EXTRACTION_DIR := ../../FeatureExtraction
MEVBENCH_FEATURE_CLASSIFICATION_DIR := ../../FeatureClassification

#---------PROJECT FILES---------

CPPFILES= $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/HessKnnMatcher.cpp $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/KnnFlannClassification.cpp $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/MatcherResultsClassifier.cpp $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/MultiThreadedMatResult.cpp $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/SparseFeaturePointObjectClassifier.cpp $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/FeatureClassification.cpp  $(MEVBENCH_FEATURE_EXTRACTION_DIR)/ThreadManager.cpp $(MEVBENCH_FEATURE_EXTRACTION_DIR)/FeatureExtraction.cpp
CCFILES= 
CFILES= 
FILES = $(CPPFILES) $(CCFILES) $(CFILES)
EXECUTABLEBASE=FeatureExtractionAndClassification
#-------
MAIN=FeatureExtractionAndClassification.cpp 
HEADERS=
#The include director for openCV
INCDIRSBASE=$(OpenCV_DIR)/include $(MEVBENCH_FEATURE_CLASSIFICATION_DIR) $(MEVBENCH_FEATURE_EXTRACTION_DIR)
CCFILEXTENSION=.cpp
CCFILEXTENSION2=.cc
CFILEXTENSION=.c
CC=g++
C=gcc
CCDEBUG=-g 
CDEBUG=-g 
CCFLAGS=-c -Wall $(CCDEBUG) -O3 $(XTRA_PARAMS)
CFLAGS=-c -Wall $(CDEBUG) -O3 $(XTRA_PARAMS)
LDEBUG=$(CCDEBUG)
CDEFINES=
CCDEFINES=-DFEATURE_EXTRACTION_MODULE -D_DRAW_TEXT_ON_IMAGE_DEFINED -DFEATURE_CLASSIFICATION_MODULE
LLFLAGSBASE= -Wall -o
LIBSBASE=pthread rt $(XTRA_LIBS) opencv_calib3d opencv_core opencv_features2d opencv_features2d opencv_flann opencv_highgui opencv_imgproc opencv_ml opencv_objdetect opencv_video opencv_nonfree
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



BASEOBJECTS=$(notdir $(basename $(FILES)))
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

#compiles the objects into executables
$(OBJDIR)/%.o: $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/%$(CFILEXTENSION)
	@echo "building changed c files"
	$(C) $(CFLAGS) $(CDEFINES) $(INCDIRS) -o$@ $< 
#compiles the objects into executables
$(OBJDIR)/%.o: $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/%$(CCFILEXTENSION)
	@echo "building changed cpp files"
	$(CC) $(CCFLAGS) $(CCDEFINES) $(INCDIRS) -o$@ $<

#compiles the objects into executables
$(OBJDIR)/%.o: $(MEVBENCH_FEATURE_CLASSIFICATION_DIR)/%$(CCFILEXTENSION2)
	@echo "building changed cc files"
	$(CC) $(CFLAGS) $(CCDEFINES) $(INCDIRS) -o$@ $<


#compiles the objects into executables
$(OBJDIR)/%.o: $(MEVBENCH_FEATURE_EXTRACTION_DIR)/%$(CFILEXTENSION)
	@echo "building changed c files"
	$(C) $(CFLAGS) $(CDEFINES) $(INCDIRS) -o$@ $< 
#compiles the objects into executables
$(OBJDIR)/%.o: $(MEVBENCH_FEATURE_EXTRACTION_DIR)/%$(CCFILEXTENSION)
	@echo "building changed cpp files"
	$(CC) $(CCFLAGS) $(CCDEFINES) $(INCDIRS) -o$@ $<

#compiles the objects into executables
$(OBJDIR)/%.o: $(MEVBENCH_FEATURE_EXTRACTION_DIR)/%$(CCFILEXTENSION2)
	@echo "building changed cc files"
	$(CC) $(CFLAGS) $(CCDEFINES) $(INCDIRS) -o$@ $<
	
	
.h.o:
	@echo "building changed header files"
	$(CC) $(CFLAGS) $(FILES)

.hpp.o:
	@echo "building changed header files"
	$(CC) $(CFLAGS) $(FILES)



clean:
	rm -f $(OBJDIR)/*.o 
	rm -f $(EXECUTABLE)





