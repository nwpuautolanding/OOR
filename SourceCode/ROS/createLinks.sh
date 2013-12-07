#!/bin/bash

if [ $# -ne 1 ]
then
	echo "Usage: $0 <path to ros src workspace>"
	exit
fi

currentDir="$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd )"

for node in $(ls -d */)
do
#	echo "$currentDir/$node to $1/"
	cleanNode="${node%?}"
	sourceDir="$currentDir/$cleanNode"
	destDir="$1/$cleanNode"
	echo "linking: $sourceDir to $destDir"
	ln -s $sourceDir $destDir
done
