#!/usr/bin/env python

import telnetlib

HOST = "192.168.100.100"

PORT =  5432
tn = telnetlib.Telnet(HOST,PORT)

tn.write("nav blind\n")

tn.close()
