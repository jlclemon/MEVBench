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



BASEDIR="$( cd -P "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export MEVBENCH_BASEDIR="$BASEDIR"












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
		$BASEDIR/Common/runAll_arm.sh
	else
		$BASEDIR/Common/runAll_arm.sh static 
	fi


else
	if [ $# -ne 3 -a $# -ne 4 -a $# -ne 5 -a  $# -ne 6 -a $# -ne 8 -a $# -ne 9 ]; then
	    let number=$#-1
	    echo "The nth is ${!number} ."

	    print_usage
	    exit


	fi

	CONFIG_FILE_BASE=$BASEDIR/Configs
	RESULTS_DIR=$BASEDIR/Results/
	if [ "${!#}" == "static" ]; then
#	if [ $# == 4 ]; then 

		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/arm/FeatureExtractionStatic
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/arm/FeatureClassificationStatic
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/arm/FeatureExtractionAndClassificationStatic
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/arm/AugmentedRealityStatic
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/arm/FaceDetectionStatic

	elif [ "${!#}" == "staticEffex" ]; then

		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/arm/FeatureExtractionEffexBaseStatic
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/arm/FeatureClassificationEffexBaseStatic
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/arm/FeatureExtractionAndClassificationEffexBaseStatic
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/arm/AugmentedRealityEffexBaseStatic
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/arm/FaceDetectionEffexBaseStatic


	elif [ "${!#}" == "staticEffexAll" ]; then

		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/arm/FeatureExtractionFullEffexStatic
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/arm/FeatureClassificationFullEffexStatic
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/arm/FeatureExtractionAndClassificationFullEffexStatic
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/arm/AugmentedRealityFullEffexStatic
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/arm/FaceDetectionFullEffexStatic



	elif [ "${!#}" == "staticEffexMinusPrefetch" ]; then

		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/arm/FeatureExtractionFullEffexNoPrefetchStatic
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/arm/FeatureClassificationFullEffexNoPrefetchStatic
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/arm/FeatureExtractionAndClassificationFullEffexNoPrefetchStatic
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/arm/AugmentedRealityFullEffexNoPrefetchStatic
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/arm/FaceDetectionFullEffexNoPrefetchStatic


	elif [ "${!#}" == "staticEffexPrefetchOnly" ]; then

		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/arm/FeatureExtractionEffexPrefetchOnlyStatic
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/arm/FeatureClassificationEffexPrefetchOnlyStatic
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/arm/FeatureExtractionAndClassificationEffexPrefetchOnlyStatic
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/arm/AugmentedRealityEffexPrefetchOnlyStatic
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/arm/FaceDetectionEffexPrefetchOnlyStatic



	else
		FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/arm/FeatureExtraction
		FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/arm/FeatureClassification
		OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/arm/FeatureExtractionAndClassification
		AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/arm/AugmentedReality
		FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/arm/FaceDetection

	fi


	if [ $# == 5 -o $# == 6 -o $# == 8 -o $# == 9 ]; then
		NUMBER_OF_CORES_COMMAND=" -nVertCores ${4} -nHoriCores ${5} "
		OBJ_RECOG_EXTRACT_EXTRA_COMMANDS="-extractXtraParams -nVertCores,${4},-nHoriCores,${5}"
		OBJ_RECOG_CLASSIFY_EXTRA_COMMANDS="-classifyXtraParams -nVertCores,${4},-nHoriCores,${5}"
		OUTPUT_FILE_SUFFIX="_Vcores_${4}_Hcores_${5}"

	else
		NUMBER_OF_CORES_COMMAND=""
		OUTPUT_FILE_SUFFIX=""
		OBJ_RECOG_EXTRACT_EXTRA_COMMANDS=""
		OBJ_RECOG_CLASSIFY_EXTRA_COMMANDS=""


	fi


	if [ $# == 8 -o $# == 9 ]; then
		OFFSET_COMMAND=" -imageChunksActive -offsetIntoChunks ${6} -numberOfChunksForSimulation ${7} -totalNumberOfChunks ${8} "
		OBJ_RECOG_EXTRACT_EXTRA_COMMANDS="$OBJ_RECOG_EXTRACT_EXTRA_COMMANDS,-imageChunksActive,-offsetIntoChunks,${6},-numberOfChunksForSimulation,${7},-totalNumberOfChunks,${8}"
		OUTPUT_FILE_SUFFIX="${OUTPUT_FILE_SUFFIX}_Offset_${6}_SimChunks_${7}_TotalChunks_${8}"
	else
		OFFSET_COMMAND=""


	fi


	ExecOutputFileName=timing.csv


	APPLICATION="${1}"
	INPUTSIZE="${2}"
	NUMBEROFTHREADS="${3}"

	if [ $APPLICATION == "SIFT" -o $APPLICATION == "FAST" -o $APPLICATION == "HoG" -o $APPLICATION == "SURF" -o $APPLICATION == "ORB" ]; then
		echo "Feature Extraction"

		echo "-configFile $CONFIG_FILE_BASE/FeatureExtraction/$APPLICATION/${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND "
		EXTRACTION_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/FeatureExtraction/$APPLICATION/${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND"
		EXTRACTION_OUTPUT_FILE="${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}${OUTPUT_FILE_SUFFIX}.csv"
		$FEATURE_EXTRACTION_EXEC $EXTRACTION_CONFIG_LINE
		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$EXTRACTION_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi







	fi

	if [ $APPLICATION == "BOOST" -o $APPLICATION == "KNN" -o $APPLICATION == "SVM" ]; then
		echo "Feature Classification"

		echo "-configFile $CONFIG_FILE_BASE/FeatureClassification/$APPLICATION/${APPLICATION,,}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND "
		CLASSIFICATION_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/FeatureClassification/$APPLICATION/${APPLICATION,,}_${INPUTSIZE}_${NUMBEROFTHREADS}.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND "
		CLASSIFICATION_OUTPUT_FILE="${APPLICATION}_${INPUTSIZE}_${NUMBEROFTHREADS}${OUTPUT_FILE_SUFFIX}.csv"
		$FEATURE_CLASSIFICATION_EXEC $CLASSIFICATION_CONFIG_LINE

		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$CLASSIFICATION_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi




	fi

	if [ $APPLICATION == "FACEDETECT" ]; then
		echo "Face Detection"

		echo "-configFile $CONFIG_FILE_BASE/Applications/FaceDetection/facedet_${INPUTSIZE}_1.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND "
		FACE_DETECT_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/Applications/FaceDetection/facedet_${INPUTSIZE}_1.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND "
		FACE_DETECT_OUTPUT_FILE="facedet_${INPUTSIZE}_1${OUTPUT_FILE_SUFFIX}.csv"
		$FACE_DETECT_EXEC $FACE_DETECT_CONFIG_LINE
		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$FACE_DETECT_OUTPUT_FILE
		else
		echo "Output file does not exist"
	fi





	fi


	if [ $APPLICATION == "OBJRECOG" ]; then
		echo "Object Recognition"

		echo "-extractionConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/SIFT_${INPUTSIZE}_${NUMBEROFTHREADS}.txt -classificationConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/objrecog_${INPUTSIZE}_${NUMBEROFTHREADS}.txt $OBJ_RECOG_EXTRACT_EXTRA_COMMANDS $OBJ_RECOG_CLASSIFY_EXTRA_COMMANDS"
		OBJ_RECOG_CONFIG_LINE="-noShowWindows -noWaitKey -extractionConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/SIFT_${INPUTSIZE}_${NUMBEROFTHREADS}.txt -classificationConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/objrecog_${INPUTSIZE}_${NUMBEROFTHREADS}.txt $OBJ_RECOG_EXTRACT_EXTRA_COMMANDS $OBJ_RECOG_CLASSIFY_EXTRA_COMMANDS"
		OBJ_RECOG_OUTPUT_FILE="objrecog_${INPUTSIZE}_${NUMBEROFTHREADS}${OUTPUT_FILE_SUFFIX}.csv"

		$OBJ_RECOG_EXEC $OBJ_RECOG_CONFIG_LINE


		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$OBJ_RECOG_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi




	fi
	if [ $APPLICATION == "AUGREAL" ]; then
		echo "Augmented Reality"


		echo "-configFile $CONFIG_FILE_BASE/Applications/AugmentedReality/aug_${INPUTSIZE}_1.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND "
		AUG_REALITY_CONFIG_LINE="-configFile $CONFIG_FILE_BASE/Applications/AugmentedReality/aug_${INPUTSIZE}_1.txt $NUMBER_OF_CORES_COMMAND $OFFSET_COMMAND "
		AUG_REALITY_OUTPUT_FILE="aug_${INPUTSIZE}_1${OUTPUT_FILE_SUFFIX}.csv"
		$AUG_REALITY_EXEC $AUG_REALITY_CONFIG_LINE
		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/$AUG_REALITY_OUTPUT_FILE
		else
			echo "Output file does not exist"
		fi


	

	fi



fi

exit


