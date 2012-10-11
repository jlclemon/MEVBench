#!/bin/bash

# Copyright (c) 2011 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#


export OpenCV_DIR="/home/jlclemon/Documents/OpenCV/OpenCV2.4.2NativeInstall/"
#"/home/jlclemon/Documents/OpenCV/OpenCV2.2NativeInstall/"
export OpenCV_STATIC_DIR="/home/jlclemon/Documents/OpenCV/OpenCV2.4.2NativeStaticInstall/"

BASEDIR="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export MEVBENCH_BASEDIR="$BASEDIR"


#export XTRA_PARAMS="-DUSE_MARSS"
#export XTRA_PARAMS="-DTSC_TIMING"
export XTRA_PARAMS="-DCLOCK_GETTIME_TIMING"











function print_usage {

    echo "Usage: $0 [all|<Algorithm> <InputSize> <NumberOfThreads> [static]]"
    echo ""
    echo " options: "
    echo "  all [static]                                               -  Run All Algorithms [static] is optional to run static linked versions"
    echo "  <Algorithm> <InputSize> <NumberOfThreads> [static]         -  Run the Algorithm with input size and number of threads,[static] is optional to run static linked versions"
    echo "                  Feature Extraction Algorithms:      "
    echo "                         SIFT - Scale Invariant Feature Transform  "
    echo "                         HoG  - Histogram Of Oriented Gradients   "
    echo "                         FAST - FAST Corner Detector  "
    echo "                         SURF - Speeded Up Robust Features"
    echo "                         ORB - Oriented Brief Features"
    echo "                  Feature Classification Algorithms:  "
    echo "                         BOOST - Boosted Classifier"
    echo "                         KNN   - K Nearest Neighbor"
    echo "                         SVM   - Support Vector Machine"
    echo "                  Feature Applications:"
    echo "                         FACEDETECT - Face Detection"
    echo "                         OBJRECOG   - Object Recognition and Pose Estimation"
    echo "                         AUGREAL    - Augmented Reality"
    echo "                  Input Sizes:"
    echo "                         l          - Small (cif)"
    echo "                         m          - Medium (vga)"
    echo "                         h          - Large (hd)"
    echo "                  Number Of Threads: "
    echo "                         1          - 1 Thread"
    echo "                         2          - 2 Threads"
    echo "                         4          - 4 Threads"
    echo "                         8          - 8 Threads"

}

if [ $# == 0 ]; then
    print_usage
    exit
fi

if [ "${1}" == "all" ]; then

	if [ $# == 1 ]; then

		echo "Running Script for All Benchmarks"
		$BASEDIR/Common/runAll.sh
	else
		$BASEDIR/Common/runAll.sh static 
	fi


else
	if [ $# -lt 3 -o $# -gt 4 ]; then
	    print_usage
	    exit


	fi

	CONFIG_FILE_BASE=$BASEDIR/Configs
	RESULTS_DIR=$BASEDIR/Results/

	if [ $# == 4 ]; then 

		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/FeatureExtractionStatic
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/FeatureClassificationStatic
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/FeatureExtractionAndClassificationStatic
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/AugmentedRealityStatic
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/FaceDetectionStatic


	else
		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/FeatureExtraction
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/FeatureClassification
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/FeatureExtractionAndClassification
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/AugmentedReality
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/FaceDetection

	fi

	ExecOutputFileName=timing.csv


	APPLICATION="${1}"
	INPUTSIZE="${2}"
	NUMBEROFTHREADS="${3}"


	if [ $APPLICATION == "SIFT" -o $APPLICATION == "FAST" -o $APPLICATION == "HoG" -o $APPLICATION == "SURF" -o $APPLICATION == "ORB" ]; then
		echo "Feature Extraction"

		echo "-configFile $CONFIG_FILE_BASE/FeatureExtraction/$APPLICATION/${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt"
		EXTRACTION_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/FeatureExtraction/$APPLICATION/${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt"
		EXTRACTION_OUTPUT_FILE="${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}.csv"
		$FEATURE_EXTRACTION_EXEC $EXTRACTION_CONFIG_LINE
		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$EXTRACTION_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi







	fi

	if [ $APPLICATION == "BOOST" -o $APPLICATION == "KNN" -o $APPLICATION == "SVM" ]; then
		echo "Feature Classification"

		echo "-configFile $CONFIG_FILE_BASE/FeatureClassification/$APPLICATION/${APPLICATION,,}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt"
		CLASSIFICATION_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/FeatureClassification/$APPLICATION/${APPLICATION,,}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt"
		CLASSIFICATION_OUTPUT_FILE="${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}.csv"
		$FEATURE_CLASSIFICATION_EXEC $CLASSIFICATION_CONFIG_LINE

		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$CLASSIFICATION_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi




	fi

	if [ $APPLICATION == "FACEDETECT" ]; then
		echo "Face Detection"

		echo "-configFile $CONFIG_FILE_BASE/Applications/FaceDetection/facedet_${INPUTSIZE}_1.txt"
		FACE_DETECT_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/Applications/FaceDetection/facedet_${INPUTSIZE}_1.txt"
		FACE_DETECT_OUTPUT_FILE="facedet_${INPUTSIZE}_1.csv"
		$FACE_DETECT_EXEC $FACE_DETECT_CONFIG_LINE
		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$FACE_DETECT_OUTPUT_FILE
		else
		echo "Output file does not exist"
	fi





	fi


	if [ $APPLICATION == "OBJRECOG" ]; then
		echo "Object Recognition"

		echo "-extractionConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/SIFT_${INPUTSIZE}_${NUMBEROFTHREADS}.txt -classificationConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/objrecog_${INPUTSIZE}_${NUMBEROFTHREADS}.txt"
		OBJ_RECOG_CONFIG_LINE="-noShowWindows -noWaitKey -extractionConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/SIFT_${INPUTSIZE}_${NUMBEROFTHREADS}.txt -classificationConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/objrecog_${INPUTSIZE}_${NUMBEROFTHREADS}.txt"
		OBJ_RECOG_OUTPUT_FILE="objrecog_${INPUTSIZE}_${NUMBEROFTHREADS}.csv"

		$OBJ_RECOG_EXEC $OBJ_RECOG_CONFIG_LINE


		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$OBJ_RECOG_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi




	fi
	if [ $APPLICATION == "AUGREAL" ]; then
		echo "Augmented Reality"


		echo "-configFile $CONFIG_FILE_BASE/Applications/AugmentedReality/aug_${INPUTSIZE}_1.txt"
		AUG_REALITY_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/Applications/AugmentedReality/aug_${INPUTSIZE}_1.txt"
		AUG_REALITY_OUTPUT_FILE="aug_${INPUTSIZE}_1.csv"
		$AUG_REALITY_EXEC $AUG_REALITY_CONFIG_LINE
		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$AUG_REALITY_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi


	

	fi



fi

exit



