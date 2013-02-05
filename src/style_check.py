#!/usr/bin/python2.6
import os
import re
import subprocess
import os.path

ignore_pattern = 'OLD'
cpplint_args = '--filter=-build/header_guard,-legal/copyright,-whitespace/end_of_line,-runtime/arrays,-readability/streams'
cpplint_args_kinect_interface = '--filter=-build/header_guard,-legal/copyright,-whitespace/end_of_line,-runtime/arrays,-readability/streams,-runtime/references'

def run(cmd, logfile):
    p = subprocess.Popen(cmd, shell=True, universal_newlines=True, stderr=logfile)
    p.wait()
    return p

for dirname, dirnames, filenames in os.walk('.'):
    for subdirname in dirnames:
        cur_dir = os.path.join(dirname, subdirname)
        if not re.search( ignore_pattern, cur_dir ) :  
            print 'CHECKING DIRECTORY: ' + cur_dir + '/'
            dirList=os.listdir(cur_dir)
            for fname in dirList:
                if (re.search( r'.cpp', fname ) or re.search( r'.h', fname )) and not re.search( ignore_pattern, fname ) :
                
                    # First run cpplint and put output in a file
                    if not re.search( 'kinect_interface', fname ) :  
                        COMMAND = './cpplint.py ' + cpplint_args + ' ' + cur_dir + '/' + fname
                    else :
                        COMMAND = './cpplint.py ' + cpplint_args_kinect_interface + ' ' + cur_dir + '/' + fname
                    log = open('log.txt', "w")
                    a_ret = run(COMMAND, log)
                    log.flush()
                    log.close()
                    # Now open cpplint and see if the last line has 0 errors
                    p = open('log.txt', "r")
                    lines = p.readlines();
                    last_line = lines[len(lines)-1]
                    if not re.search( r'Total errors found: 0', last_line ) :  
                        print '*************************************************************'
                        print COMMAND
                        print cur_dir + '/' + fname
                        for line in lines :
                            print line  + "\r",
                        print cur_dir + '/' + fname
                        print '*************************************************************'
                    p.close()
if os.path.isfile('log.txt'):
    os.remove('log.txt')
raw_input("Press ENTER to exit")
