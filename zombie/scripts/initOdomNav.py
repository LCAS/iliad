#!/usr/bin/env python

import telnetlib

HOST = "192.168.100.100"

PORT =  5432
tn = telnetlib.Telnet(HOST,PORT)

tn.read_until('\n',500)

tn.write("nav blind \r\n")

tn.close()
