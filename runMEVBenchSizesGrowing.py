#!/usr/bin/env python

import sys
import os
import traceback
import optparse
import time
import re
import math
import subprocess



def main():
	if (len(sys.argv) <= 1):
		print "no Args"
		numberOfCoresMax = 2
	else:
		numberOfCoresMax = sys.argv[1]
		if(numberOfCoresMax >256):
			print "Max number of Cores too large.  Shrinking to 256"
			numberOfCoresMax = 256

	if (len(sys.argv) <= 2):
		coresStep = 1
	else:
		coresStep = sys.argv[2]
		if(coresStep >numberOfCoresMax):
			print "Core step too large.  Shrinking to 1"
			coresStep = 1




	benchmarksToRun = ('SIFT', 'HoG')#, 'FAST', 'SURF', 'BOOST', 'KNN', 'SVM', 'OBJRECOG')
	imageSizesList = ('l', 'm')


	MEVBenchCommand = "./run.sh"

	logFileName = "./mevbench.log"
	logfile =  open(logFileName, 'w')
	numberOfBenchmarksRun  =0

	for benchmark in benchmarksToRun:

		for imageSize in imageSizesList:

			for numberOfCores in range(1, numberOfCoresMax+1,coresStep):
				run_command_list = [MEVBenchCommand, benchmark, imageSize, '1', 'numberOfCores']
				print "Running benchmark %s %s %s" % (str(numberOfCores), benchmark, imageSize)
				subprocess.call(run_command_list, stdout=logfile, shell=False)
				



	logfile.close()


if __name__ == '__main__':
    try:
        exit_code = main()
        if exit_code is None:
            exit_code = 0
        sys.exit(exit_code)
    except KeyboardInterrupt, e: # Ctrl-C
        raise e
    except SystemExit, e: # sys.exit()
        raise e
    except Exception, e:
        print 'ERROR, UNEXPECTED EXCEPTION'
        print str(e)
        traceback.print_exc()
        os._exit(1)

