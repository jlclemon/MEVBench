#!/bin/bash


export LD_LIBRARY_PATH=$OpenCV_DIR/lib:$LD_LIBRARY_PATH

#FEATURE_EXTRACTION_EXEC=OpenCv2_2_FeatureExtractionWorking/bin/FeatureExtraction
#FEATURE_CLASSIFICATION_EXEC=OpenCv2_2_FeatureClassification/bin/FeatureClassification
#OBJ_RECOG_EXEC=FeatureExtractionAndClassification/bin/FeatureExtractionAndClassification
#AUG_REALITY_EXEC=OpenCv_AugmentedReality/bin/AugmentedReality

FEATURE_EXTRACTION_EXEC=bin/FeatureExtraction
FEATURE_CLASSIFICATION_EXEC=bin/FeatureClassification
OBJ_RECOG_EXEC=bin/FeatureExtractionAndClassification
AUG_REALITY_EXEC=bin/AugmentedReality
FACE_DETECT_EXEC=bin/FaceDetection

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
cd OpenCv2_2_FeatureExtractionWorking


for (( currentAlgo=0;currentAlgo<$algoCount;currentAlgo++)); do
	for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do
		for (( currentCoreCount=0;currentCoreCount<$coreCount;currentCoreCount++)); do

			echo "-configFile ${AlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			EXTRACTION_CONFIG_LINES[$extractionConfigLinesCount]="-configFile ${AlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			EXTRACTION_OUTPUT_FILES[$extractionConfigLinesCount]="${AlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.csv"
			$FEATURE_EXTRACTION_EXEC ${EXTRACTION_CONFIG_LINES[${extractionConfigLinesCount}]}
			if [ -e $ExecOutputFileName ]; then
				mv $ExecOutputFileName ${EXTRACTION_OUTPUT_FILES[${extractionConfigLinesCount}]}
			else
				echo "Output file does not exist"
			fi



			((extractionConfigLinesCount++))
		done
	done
done 

cd ../OpenCv2_2_FeatureClassification


declare -a CLASSIFICATION_CONFIG_LINES
declare -a CLASSIFICATION_OUTPUT_FILES
ClassificationAlgoList=('svm' 'boost' 'knn')
let algoCount=${#ClassificationAlgoList[@]}
let classificationConfigLinesCount=0
for (( currentAlgo=0;currentAlgo<$algoCount;currentAlgo++)); do
	for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do
		for (( currentCoreCount=0;currentCoreCount<$coreCount;currentCoreCount++)); do

			echo "-configFile ${ClassificationAlgoList[${currentAlgo}]}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			CLASSIFICATION_CONFIG_LINES[$classificationConfigLinesCount]="-configFile ${ClassificationAlgoList[${currentAlgo}],}_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
			CLASSIFICATION_OUTPUT_FILES[$classificationConfigLinesCount]="${ClassificationAlgoList[${currentAlgo}]}_${SizeList[${currentSize}],}_${ThreadCountList[${currentCoreCount}]}.csv"
			$FEATURE_CLASSIFICATION_EXEC ${CLASSIFICATION_CONFIG_LINES[${classificationConfigLinesCount}]}

			if [ -e $ExecOutputFileName ]; then
				mv $ExecOutputFileName ${CLASSIFICATION_OUTPUT_FILES[${classificationConfigLinesCount}]}
			else
				echo "Output file does not exist"
			fi


			((classificationConfigLinesCount++))
		done
	done
done 


cd ../FeatureExtractionAndClassification

declare -a OBJ_RECOG_CONFIG_LINES
declare -a OBJ_RECOG_OUTPUT_FILES

let objRecogConfigLinesCount=0
for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do
	for (( currentCoreCount=0;currentCoreCount<$coreCount;currentCoreCount++)); do

		echo "-extractionConfig SIFT_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt -classificationConfig objrecog_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
		OBJ_RECOG_CONFIG_LINES[$objRecogConfigLinesCount]="-noShowWindows -noWaitKey -extractionConfig SIFT_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt -classificationConfig objrecog_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.txt"
		OBJ_RECOG_OUTPUT_FILES[$objRecogConfigLinesCount]="objrecog_${SizeList[${currentSize}]}_${ThreadCountList[${currentCoreCount}]}.csv"

		$OBJ_RECOG_EXEC ${OBJ_RECOG_CONFIG_LINES[${objRecogConfigLinesCount}]}


		if [ -e $ExecOutputFileName ]; then
			mv $ExecOutputFileName ${OBJ_RECOG_OUTPUT_FILES[${objRecogConfigLinesCount}]}
		else
			echo "Output file does not exist"
		fi


		((objRecogConfigLinesCount++))
	done
done

cd ../OpenCv_AugmentedReality

declare -a AUG_REALITY_CONFIG_LINES
declare -a AUG_REALITY_OUTPUT_FILES

let augRealityConfigLinesCount=0
for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do


	echo "-configFile aug_${SizeList[${currentSize}]}_1.txt"
	AUG_REALITY_CONFIG_LINES[$augRealityConfigLinesCount]="-configFile aug_${SizeList[${currentSize}]}_1.txt"
	AUG_REALITY_OUTPUT_FILES[$augRealityConfigLinesCount]="aug_${SizeList[${currentSize}]}_1.csv"
	$AUG_REALITY_EXEC ${AUG_REALITY_CONFIG_LINES[${augRealityConfigLinesCount}]}
	if [ -e $ExecOutputFileName ]; then
		mv $ExecOutputFileName ${AUG_REALITY_OUTPUT_FILES[${augRealityConfigLinesCount}]}
	else
		echo "Output file does not exist"
	fi


	((augRealityConfigLinesCount++))

done


cd ../FaceDetection

declare -a FACE_DETECT_CONFIG_LINES
declare -a FACE_DETECT_OUTPUT_FILES

let faceDetectionConfigLinesCount=0
for (( currentSize=0;currentSize<$sizeCount;currentSize++)); do


	echo "-configFile facedet_${SizeList[${currentSize}]}_1.txt"
	FACE_DETECT_CONFIG_LINES[$faceDetectionConfigLinesCount]="-configFile facedet_${SizeList[${currentSize}]}_1.txt"
	FACE_DETECT_OUTPUT_FILES[$faceDetectionConfigLinesCount]="facedet_${SizeList[${currentSize}]}_1.csv"
	$FACE_DETECT_EXEC ${FACE_DETECT_CONFIG_LINES[${faceDetectionConfigLinesCount}]}
	if [ -e $ExecOutputFileName ]; then
		mv $ExecOutputFileName ${FACE_DETECT_OUTPUT_FILES[${faceDetectionConfigLinesCount}]}
	else
		echo "Output file does not exist"
	fi


	((faceDetectionConfigLinesCount++))

done

