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


export LD_LIBRARY_PATH=$OpenCV_DIR/lib:$LD_LIBRARY_PATH

#FEATURE_EXTRACTION_EXEC=OpenCv2_2_FeatureExtractionWorking/bin/FeatureExtraction
#FEATURE_CLASSIFICATION_EXEC=OpenCv2_2_FeatureClassification/bin/FeatureClassification
#OBJ_RECOG_EXEC=FeatureExtractionAndClassification/bin/FeatureExtractionAndClassification
#AUG_REALITY_EXEC=OpenCv_AugmentedReality/bin/AugmentedReality


echo $MEVBENCH_BASEDIR
BASEDIR=$MEVBENCH_BASEDIR

CONFIG_FILE_BASE=$BASEDIR/Configs

RESULTS_DIR=$BASEDIR/Results

if [ $# == 0 ]; then
	FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/FeatureExtraction
	FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/FeatureClassification
	OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/FeatureExtractionAndClassification
	AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/AugmentedReality
	FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/FaceDetection
else

	FEATURE_EXTRACTION_EXEC=$BASEDIR/Benchmarks/FeatureExtraction/bin/FeatureExtractionStatic
	FEATURE_CLASSIFICATION_EXEC=$BASEDIR/Benchmarks/FeatureClassification/bin/FeatureClassificationStatic
	OBJ_RECOG_EXEC=$BASEDIR/Benchmarks/Applications/ObjectRecognition/bin/FeatureExtractionAndClassificationStatic
	AUG_REALITY_EXEC=$BASEDIR/Benchmarks/Applications/AugmentedReality/bin/AugmentedRealityStatic
	FACE_DETECT_EXEC=$BASEDIR/Benchmarks/Applications/FaceDetection/bin/FaceDetectionStatic
fi

ExecOutputFileName=timing.csv

AlgoList=('SIFT' 'HoG' 'FAST' 'SURF')
let algoCount=${#AlgoList[@]}
SizeList=('l' 'm' 'h')
let sizeCount=${#SizeList[@]}
ThreadCountList=('1' '2' '4' '8')
let coreCount=${#ThreadCountList[@]}


declare -a EXTRACTION_CONFIG_LINES
declare -a EXTRACTION_OUTPUT_FILES


let extractionConfigLinesCount=0
#cd OpenCv2_2_FeatureExtractionWorking


for (( currentAlgo=0;currentAlgo<$algoCount;currentAlgo++)); do
	for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do
		for (( currentCoreCount=0;currentCoreCount<$coreCount;currentCoreCount++)); do

			echo "-configFile $CONFIG_FILE_BASE/FeatureExtraction/${AlgoList[${currentAlgo}]}/${AlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			EXTRACTION_CONFIG_LINES[$extractionConfigLinesCount]="-configFile $CONFIG_FILE_BASE/FeatureExtraction/${AlgoList[${currentAlgo}]}/${AlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			EXTRACTION_OUTPUT_FILES[$extractionConfigLinesCount]="${AlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.csv"
			$FEATURE_EXTRACTION_EXEC ${EXTRACTION_CONFIG_LINES[${extractionConfigLinesCount}]}
			if [ -e $ExecOutputFileName ]; then
				mv $ExecOutputFileName $RESULTS_DIR/${EXTRACTION_OUTPUT_FILES[${extractionConfigLinesCount}]}
			else
				echo "Output file does not exist"
			fi



			((extractionConfigLinesCount++))
		done
	done
done 

#cd ../OpenCv2_2_FeatureClassification


declare -a CLASSIFICATION_CONFIG_LINES
declare -a CLASSIFICATION_OUTPUT_FILES
ClassificationAlgoList=('svm' 'boost' 'knn')
ClassificationAlgoConfigDirs=('SVM' 'BOOST' 'KNN')

let algoCount=${#ClassificationAlgoList[@]}
let classificationConfigLinesCount=0
for (( currentAlgo=0;currentAlgo<$algoCount;currentAlgo++)); do
	for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do
		for (( currentCoreCount=0;currentCoreCount<$coreCount;currentCoreCount++)); do

			echo "-configFile $CONFIG_FILE_BASE/FeatureClassification/${ClassificationAlgoConfigDirs[${currentAlgo}]}/${ClassificationAlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			CLASSIFICATION_CONFIG_LINES[$classificationConfigLinesCount]="-configFile $CONFIG_FILE_BASE/FeatureClassification/${ClassificationAlgoConfigDirs[${currentAlgo}]}/${ClassificationAlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			CLASSIFICATION_OUTPUT_FILES[$classificationConfigLinesCount]="${ClassificationAlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.csv"
			$FEATURE_CLASSIFICATION_EXEC ${CLASSIFICATION_CONFIG_LINES[${classificationConfigLinesCount}]}

			if [ -e $ExecOutputFileName ]; then
				mv $ExecOutputFileName $RESULTS_DIR/${CLASSIFICATION_OUTPUT_FILES[${classificationConfigLinesCount}]}
			else
				echo "Output file does not exist"
			fi


			((classificationConfigLinesCount++))
		done
	done
done 


#cd ../FeatureExtractionAndClassification

declare -a OBJ_RECOG_CONFIG_LINES
declare -a OBJ_RECOG_OUTPUT_FILES

let objRecogConfigLinesCount=0
for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do
	for (( currentCoreCount=0;currentCoreCount<$coreCount;currentCoreCount++)); do

		echo "-extractionConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/SIFT_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt -classificationConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/objrecog_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
		OBJ_RECOG_CONFIG_LINES[$objRecogConfigLinesCount]="-noShowWindows -noWaitKey -extractionConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/SIFT_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt -classificationConfig $CONFIG_FILE_BASE/Applications/ObjectRecognition/objrecog_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
		OBJ_RECOG_OUTPUT_FILES[$objRecogConfigLinesCount]="objrecog_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.csv"

		$OBJ_RECOG_EXEC ${OBJ_RECOG_CONFIG_LINES[${objRecogConfigLinesCount}]}


		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName $RESULTS_DIR/${OBJ_RECOG_OUTPUT_FILES[${objRecogConfigLinesCount}]}
		else
			echo "Output file does not exist"
		fi


		((objRecogConfigLinesCount++))
	done
done

#cd ../OpenCv_AugmentedReality

declare -a AUG_REALITY_CONFIG_LINES
declare -a AUG_REALITY_OUTPUT_FILES

let augRealityConfigLinesCount=0
for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do


	echo "-configFile $CONFIG_FILE_BASE/Applications/AugmentedReality/aug_${SizeList[${currentSize}]}_1.txt"
	AUG_REALITY_CONFIG_LINES[$augRealityConfigLinesCount]="-configFile $CONFIG_FILE_BASE/Applications/AugmentedReality/aug_${SizeList[${currentSize}]}_1.txt"
	AUG_REALITY_OUTPUT_FILES[$augRealityConfigLinesCount]="aug_${SizeList[${currentSize}]}_1.csv"
	$AUG_REALITY_EXEC ${AUG_REALITY_CONFIG_LINES[${augRealityConfigLinesCount}]}
	if [ -e $ExecOutputFileName ]; then
		mv $ExecOutputFileName $RESULTS_DIR/${AUG_REALITY_OUTPUT_FILES[${augRealityConfigLinesCount}]}
	else
		echo "Output file does not exist"
	fi


	((augRealityConfigLinesCount++))

done


#cd ../FaceDetection

declare -a FACE_DETECT_CONFIG_LINES
declare -a FACE_DETECT_OUTPUT_FILES

let faceDetectionConfigLinesCount=0
for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do


	echo "-configFile $CONFIG_FILE_BASE/Applications/FaceDetection/facedet_${SizeList[${currentSize}]}_1.txt"
	FACE_DETECT_CONFIG_LINES[$faceDetectionConfigLinesCount]="-configFile $CONFIG_FILE_BASE/Applications/FaceDetection/facedet_${SizeList[${currentSize}]}_1.txt"
	FACE_DETECT_OUTPUT_FILES[$faceDetectionConfigLinesCount]="facedet_${SizeList[${currentSize}]}_1.csv"
	$FACE_DETECT_EXEC ${FACE_DETECT_CONFIG_LINES[${faceDetectionConfigLinesCount}]}
	if [ -e $ExecOutputFileName ]; then
		mv $ExecOutputFileName $RESULTS_DIR/${FACE_DETECT_OUTPUT_FILES[${faceDetectionConfigLinesCount}]}
	else
		echo "Output file does not exist"
	fi


	((faceDetectionConfigLinesCount++))

done

