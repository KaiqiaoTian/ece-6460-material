#! /usr/bin/env python

import rostest

PKG = 'homework1'
NAME = 'homework1_test'

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, 'homework1_test.Homework1Test')