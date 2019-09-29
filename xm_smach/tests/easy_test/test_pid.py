#!/usr/bin/env python  
# encoding:utf-8
from subprocess import check_output,call
def get_pid(name):
    return map(int,check_output(["pidof",name]).split())

pid = get_pid("people_tracking")
print pid[0]
command = 'kill '+str(pid[0])
print command
call(command,shell=True)