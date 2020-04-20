#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
file = open("test.txt","w")
for i in range(77):
    file.write(str(i+1))
    file.write("\n")
file.close()


