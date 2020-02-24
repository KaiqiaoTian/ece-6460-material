#! /usr/bin/env python

import rostest

PKG = 'homework3'
NAME = 'homework3_test'

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, 'homework3_test.Homework3Test')
