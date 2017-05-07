#!/usr/bin/env python
import os
from subprocess import check_output, CalledProcessError

def checkPID(process):
	try:
		pidlist = map(int, check_output(["pidof", process]).split())
		return True
	except CalledProcessError:
		return False

if __name__ == '__main__':
	#Kill rovio.
	os.system("rosnode kill rovio")
	#Wait until killed.
	while(checkPID("rovio_node")): print("killing")
	#Restart.
	os.system("rosrun rovio rovio_node")