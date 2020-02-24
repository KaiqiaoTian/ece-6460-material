#! /usr/bin/env python

import rostest

PKG = 'homework4'
NAME = 'homework4_test'

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, 'homework4_test.Homework4Test')
