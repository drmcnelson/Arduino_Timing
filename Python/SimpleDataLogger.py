#!/usr/bin/python
"""
SimpleDataLogger.py

Mitchell C. Nelson (c) 2024

"""

__author__    = "Mitchell C. Nelson, PhD"
__copyright__ = "Copyright 2024, Mitchell C, Nelson"
__version__   = "0.1"
__email__     = "drmcnelson@gmail.com"
__status__    = "alpha testing"

import sys
import time
import select

import os
import io
import subprocess

import operator

import signal
import atexit

#from timeit import default_timer as timer

import platform

import serial

from datetime import datetime
from time import sleep, time, process_time, thread_time

# from threading import Lock, Semaphore, Thread
#from queue import SimpleQueue, Empty
from multiprocessing import Process, Queue, Value, Lock
import queue

import struct

import inspect
from itertools import count

import numpy as np

versionstring = os.path.basename(__file__) + ' - version %s %s'%(__version__,__copyright__)

def input_ready():
    return (sys.stdin in select.select([sys.stdin], [], [], 0)[0])

def dumpdictionary( dictionary, exclude=None, file=sys.stdout ):

    for key,val in dictionary.items():

        if exclude is not None:
            if key in exclude:
                continue

        if type(val) in [ float, int ] :
            file.write( '# ' + key + ' = ' + str(val) + '\n' )

        elif type(val) in [ str ] :
            file.write( '# ' + key + ' = "' + str(val) + '"\n' )

        elif type(val) in [ list ] :
            if not len(val):
                file.write( '# ' + key + ' = []\n' )
            else:
                values = val
                if type(values[0]) in [ float, int ] :
                    file.write( '# ' + key + ' = [ ' + ', '.join( [ str(v) for v in values ] ) + ' ]\n'  )
                elif type(values[0]) in [ str ] :
                    file.write( '# ' + key + ' = [ "' + '", "'.join( [ str(v) for v in values ] ) + '" ]\n'  )

def stringSubstitution( s, mydict ):
        
    try:
        exec( "newname__=\""+s+"\"", mydict, globals() )
        print(newname__)
    except Exception as e:
        print( e )
        return None

    return newname__

def findSubstring(s, key):
    if key in s:
        try:
            idx = s.index(key)
            return s[idx:]
        except Exception as e:
            print( s, key, e )
    return None

# ========================================================================                    

class ARDUINODEVICE:

    _ids = count(0)


    def __init__( self, portspec, readtimeout=1., writetimeout=1., logfile=None, debug=False ):

        self.versionstring = os.path.basename(__file__) + ' - version %s %s'%(__version__,__copyright__)
        
        self.ser = serial.Serial( portspec, timeout=readtimeout, write_timeout=writetimeout )

        self.instance = next( self._ids )
        
        self.name= portspec

        self.responsequeue = Queue()

        self.flag = Value( 'i', 1 )
        self.errorflag = Value( 'i', 0 )

        if debug:
            self.debug = Value( 'i', 1 )
        else:
            self.debug = Value( 'i', 0 )

        if logfile is not None:
            self.logfile = logfile
            self.loggerqueue = Queue()
            self.loggerthread = Process( target = self.logger )
            self.loggerthread.start()
        else:
            self.logfile = None
            self.loggerqueue = None
            self.loggerthread = None

        self.readerthread = Process( target = self.reader )
        self.readerthread.start()
        
        sleep(1)
        self.commandwrite("version")
        self.commandwrite("configuration")
        

    def setDebug(self,debug=True):
        if debug:
            self.debug.value = 1
        else:
            self.debug.value = 0

    def clearDebug(self):
        self.debug.value = 0

    def close( self, ignored=None ):

        self.flag.value = 0
        sleep( 0.1 )

        if self.readerthread is not None:
            self.readerthread.terminate()
            self.readerthread.join()

        if self.loggerthread is not None:
            self.readerthread.terminate()
            self.readerthread.join()
            
    def write(self,s):
        self.ser.write( s.encode() )

    # ========================================================
    def commandwrite(self,s,timeout=0.5):

        while not self.responsequeue.empty():
            try:
                response = self.responsequeue.get(True,timeout)
                print("pending response:",response)
            except queue.Empty:
                break
            
        print("Command:", s)
        self.write(s+'\n')

        while True:
            try:
                response = self.responsequeue.get(True,timeout)
                print("response:", response)
            except queue.Empty:
                break
            except Exception as e:
                print(e)

        if self.errorflag.value:
            self.errorflag.value = 0
            return False

        return True

    # -------------------------------------------------------
    def logger(self):

        with open(self.logfile, 'a') as f:
            while self.flag.value:
                try:
                    m = self.loggerqueue.get(True,1)
                    f.write(str(m.strip()) + '\n')
                    f.flush()
                except queue.Empty:
                    pass

                
    # ----------------------------------------------------------            
    def reader( self ):

        response_put = self.responsequeue.put

        if self.loggerqueue is not None:
            logger_put = self.loggerqueue.put
        else:
            logger_put = None

        read_until = self.ser.read_until
        
        while self.flag.value:
            
            try:
                buffer = read_until( )
                buffer = buffer.decode()[:-1]
            except Exception as e:
                print("reader error", e)
                continue
            
            if buffer is None:
                continue

            if not len(buffer):
                continue

            if buffer.startswith("Error"):
                self.errorflag.value = 1
            
            if logger_put:
                logger_put(buffer)
                
            if self.debug.value:
                print("reader:", buffer)
            
            response_put(buffer)
        
    # ================================================================================
    def cli_(self,line):

        if '%' in line and not '=' in line:
            try:
                exec( 'temporary_variable_='+line, self.__dict__, globals())            
                line = temporary_variable_
                print(line)
            except Exception as e:
                print(e)
                return False

        # ----------------------------------------
        # Dump the namespaces
        if line == 'dump all':
            for key, val in globals().items():
                print( "global ", key, val )
            for key, val in locals().items():
                print( "local ",key, val )
            for key, val in self.__dict__.items():
                print( "__dict__ ",key, val )
            return True

        # ----------------------------------------
        # Dump variables from the class dictionary
        if line == '=':
            dumpdictionary(self.__dict__)
            dumpdictionary(globals())
            return True

        # ----------------------------------------
        # Execute in python command processor
        if line.startswith( '=' ):
            try:
                exec( line[1:].strip(), self.__dict__, globals() )
                return True
            except Exception as e:
                print( e )
                return False

        if '=' in line:
            try:
                print( 'executing line:', line )
                exec( line, self.__dict__, globals() )
                return True
            except Exception as e:
                print( e )
                return False

        # --------------------------------------
        # Shell processing
        if line.startswith('!'):
            try:
                p = subprocess.Popen(line[1:], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                #p = subprocess.Popen(line[1:].split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                output,errors = p.communicate()
                if output:
                    print("Output:")
                    print(output.strip())
                if errors:
                    print("Errors:", errors.strip())
                if p.returncode:
                    print( "returncode", p.returncode, "(failed)")
                    return False
            except Exception as e:
                print(e)
                return False
            return True

        # --------------------------------------
        # Batch/indirect command processing
        if line.startswith('@'):
            try:
                linenumber = 0
                with open(line[1:]) as fp:
                    for line in fp:
                        linenumber += 1
                        line = line.strip()
                        if len(line) :
                            print( "*****")
                            print( "line %d:"%(linenumber), line)
                            if not self.cli_(line):
                                return False
            except Exception as e:
                print(e)
                return False
            return True
        
        # --------------------------------------
        # Debug
        if line.startswith("debug"):
            pars = line.split(maxsplit=1)
            if len(pars)<2:
                print( "Debug:", self.debug.value)
                return True

            if pars[1] in ['1','true','on']:
                self.debug.value=1
                print( "debug on")
                return True
            
            if pars[1] in ['0','false','off']:
                self.debug.value=0
                print( "debug off")
                return True

            print( "debug parameter not recognized")
            return False
            
        # --------------------------------------
        # Pass it to the connected device
        return self.commandwrite(line)
        
    # ------------------------------------------------------------------------
    def cli(self):
        
        while self.flag.value:

            try:
                line = input( self.name + ':' )
            except KeyboardInterrupt:
                print('^C')
                line = input( "do you want to exit?" )
                if bool(line):
                    self.flag.value = 0
                    break

            if line.lower() in ['exit', 'quit', 'q' ]:
                self.flag.value = 0
                break

            retv = self.cli_(line)
            print( retv )

        return self.flag.value
            
#=======================================================================================

if __name__ == "__main__":

    import argparse

    try:
        import readline
    except:
        pass

    import atexit
    import signal
    
    if platform.system() == 'Linux':
        ser0_default = '/dev/ttyACM0'
        ser1_default = '/dev/ttyACM1'
    elif platform.system() == 'Windows':
        ser0_default = 'COM1:'
        ser1_default = 'COM2:'

    # ---------------------------------------------------------
    def SignalHandler(signal, frame):

        serialdevice.close()
        sleep(0.1)
            
        print('Exit')
        sys.exit(0)            
    
    # ---------------------------------------------------------
    class ExplicitDefaultsHelpFormatter(argparse.ArgumentDefaultsHelpFormatter):
        def _get_help_string(self, action):
            if action.default in (None, False):
                return action.help
            return super()._get_help_string(action)
    
    parser = argparse.ArgumentParser( description='Simple Arduino Controller Monitor/Cli',
                                      formatter_class=ExplicitDefaultsHelpFormatter )

    parser.add_argument( 'ports', default=[ser0_default], nargs='*',
                         help = 'one or more serial or com ports,' +
                         ' the first is the control port, others are readonly' )

    parser.add_argument( '--logfile', help='file for logging responses' )
    parser.add_argument( '--historyfile', default = 'simpledatalogger.history', help='history file for the command line interface' )
    parser.add_argument( '--nohistoryfile' )
        
    args = parser.parse_args()           

    serialdevice = ARDUINODEVICE( args.ports[0], logfile=args.logfile )


    if not args.nohistoryfile:
        try:
            readline.read_history_file(args.historyfile)
        except Exception as e:
            print('historyfile: ', e)
            print('continuing')
            
        atexit.register(readline.write_history_file, args.historyfile)

    signal.signal(signal.SIGINT, SignalHandler)
    
    # ---------------------------------------------------------

    serialdevice.cli()

    serialdevice.close()
