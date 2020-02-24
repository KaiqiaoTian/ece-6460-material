#! /usr/bin/env python

import rostest

PKG = 'homework2'
NAME = 'homework2_test'

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, 'homework2_test.Homework2Test')