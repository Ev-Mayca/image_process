#########################################################################
# File Name: 1.sh
# Author: Ev
# mail: wang2011yiwei@sina.com
# Created Time: 2017年02月28日 星期二 11时03分53秒
#########################################################################
#!/bin/bash
filelist=`ls ../2.15-3/`
for file in $filelist
do
#	echo ../$file >> 2.txt
	 ./gps ../2.15-3/$file/gps.txt >> 2.txt
done
