#!/usr/bin/python3
#
# eeprom_access.py
#
# Updated EEPROM read/write utility
# Based off Gunnar's earlier work
#
# Copyright 2022 Hellbender Inc.
# All Rights Reserved
#
# Changelog:
# Author Email, Date,     , Comment
# gryder      , 2022-09-06  Created original shell script
# niessl      , 2022-11-12, Ported to python
#                           Added multibyte terminal support
#

import argparse
import binascii
import datetime
import os
import smbus
import time

class EEPROM_OOB_Exception:
  pass
  
class EEPROM_membank:
  def __init__(self, devAddr, memSize):
    self.devAddr = devAddr
    self.memSize = memSize
    
class EEPROM_endpoint:
  def __init__(self, busAddr, bankList, debug, test, logFile):
    self.busAddr  = busAddr
    self.bankList = bankList
    self.debug    = debug
    self.test     = test
    self.logFile  = logFile
    self.logEntries = []

  def connect(self):
    self.bus = smbus.SMBus(self.busAddr)
    self.bus.write_quick(self.bankList[0].devAddr)
    if self.debug:
      print("Connecting to I2C dev {}".format(self.busAddr))

  def readByte(self, bank, memAddr):
    devAddr = self.bankList[bank].devAddr
    if memAddr < 0 or memAddr >= self.bankList[bank].memSize:
      raise EEPROM_OOB_Exception
    if self.debug:
      print("reading bank {} addr {}".format(bank, memAddr))
    return self.bus.read_byte_data(devAddr, memAddr)

  def writeByte(self, val, bank, memAddr):
    devAddr = self.bankList[bank].devAddr
    if memAddr < 0 or memAddr >= self.bankList[bank].memSize:
      raise EEPROM_OOB_Exception
    if self.debug:
      print("writing {} to bank {} addr {}".format(val, bank, memAddr))
    self.bus.write_byte_data(devAddr, memAddr, val)  

  def readBytes(self, bank, startAddr, length):
    if startAddr < 0 or length < 1:
      raise EEPROM_OOB_Exception
    if startAddr + length > self.bankList[bank].memSize:
      raise EEPROM_OOB_Exception
    retVal = bytearray(b'')
    for ii in range(0, length):
      addr = startAddr + ii
      val = 0x00
      if not self.test:
        val = self.readByte(bank, addr)
      retVal.append(val)
    self.addLogEntry("READ", self.bankList[bank].devAddr, startAddr,
                     length, None)
    return retVal

  def writeBytes(self, bank, startAddr, writeBuff, length):
    if startAddr < 0 or length < 1:
      raise EEPROM_OOB_Exception
    if startAddr + length - 1 > self.bankList[bank].memSize:
      raise EEPROM_OOB_Exception
    for ii in range(0, length):
      addr = startAddr + ii
      val = writeBuff[ii]
      if not self.test:
        self.writeByte(val, bank, addr)
      time.sleep(0.004)
    self.addLogEntry("WRITE", self.bankList[bank].devAddr, startAddr,
                     length, writeBuff)

  def addLogEntry(self, action, devAddr, offset, length, content):
    entry = [action]
    nowTime = datetime.datetime.now()
    datetimeStr = nowTime.strftime("%Y-%m-%d_%H:%M:%S")
    entry.append(datetimeStr)
    entry.append(str(self.busAddr))
    entry.append(str(hex(devAddr)))
    entry.append(str(hex(offset)))
    entry.append(length)
    if content is not None:
      entry.append(binascii.hexlify(content).decode("utf-8"))
    self.logEntries.append(entry)

  def clearLogEntries(self):
    self.logEntries = []

  def writeLogEntries(self): 
    with open(self.logFile, 'a') as logfile:
      for entry in self.logEntries:
        formatStr = "{} {} - Bus: {} Addr: {} Offset: {}".format(entry[1], entry[0], entry[2], entry[3], entry[4])
        if entry[0] == "READ":
          formatStr = formatStr + " Length: {}\n".format(entry[5])
        elif entry[0] == "WRITE":
          formatStr = formatStr + " Length: {} Content: {}\n".format(entry[5], entry[6])
        logfile.write(formatStr)        

def handleArgs():
  parser = argparse.ArgumentParser()
  #Read or write group
  RWgroup = parser.add_mutually_exclusive_group()
  RWgroup.add_argument("-r", "--read", action="store_true", help="Read data from EEPROM")
  RWgroup.add_argument("-w", "--write", action="store_true", help="Write data to EEPROM")
  #File or byte group
  FTgroup = parser.add_mutually_exclusive_group()
  FTgroup.add_argument("-f", "--file", type=str, help="Read/Write to/from file source. (Data read as is/UTF-8)")
  FTgroup.add_argument("-by", "--byte", type=str, nargs='?', default="", help="Read/Write to/from standard out/in. (Data should be hex formatted)")
  #Content options
  parser.add_argument("-o", "--offset", type=int, nargs='?', default=0, help="Address offset of bank (default 0)")
  parser.add_argument("-ba", "--bank", type=int, help="EEPROM bank (0 or 1). Bank 0 is for reserved data.")
  parser.add_argument("-l", "--length", type=int, nargs='?', default=256, help="Length of data to read or write (default to whole bank/source)")
  #Instrumentation options
  parser.add_argument("-t", "--test", action="store_true", default=False, help="Perform test action (Only write to log and not device")
  parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
  #Deprecated options
  parser.add_argument("-s", "--serial", help="Deprecated option. Does nothing now (originally serial information write)")
  parser.add_argument("-n", "--null", help="Deprecated option. Does nothing now (originally optional null terminator write)")
  return parser.parse_args()
  
if __name__ == "__main__":
  args = handleArgs()
  #EEPROM definition
  bank0 = EEPROM_membank(0x50, 256)
  bank1 = EEPROM_membank(0x51, 256)
  endpt = EEPROM_endpoint(0x1, [bank0, bank1], args.verbose, args.test, "/mnt/data/eeprom.log")
  endpt.connect()
  #Read from EEPROM
  if args.read:
    content = endpt.readBytes(args.bank, args.offset, args.length, )
    if args.file:
      if args.verbose:
        print("Writing EEPROM contents to {}".format(args.file))
      with open(args.file, 'wb') as target:
        target.write(content)
    else:
      if args.verbose:
        print("Reading EEPROM contents to terminal")
      outStr = binascii.hexlify(content)
      print(outStr.decode("utf-8"))      
  #Write to EEPROM
  if args.write:
    content = []
    conLen  = 0
    if args.file:
      if args.verbose:
        print("Writing {} contents to EEPROM".format(args.file))   
      with open(args.file, 'rb') as source:
        source.seek(0, os.SEEK_END)
        sourceLen = source.tell()
        source.seek(0, 0)
        conLen = min(sourceLen, args.length)  
        if args.verbose:
          print("Writing {} bytes from file".format(conLen))
        content = source.read(conLen)  
    else:
      if args.verbose:
        print("Writing terminal contents ({}) to EEPROM".format(args.byte))   
      content = binascii.unhexlify(args.byte)
    conLen = len(content)
    endpt.writeBytes(args.bank, args.offset, content, conLen)
  endpt.writeLogEntries()
