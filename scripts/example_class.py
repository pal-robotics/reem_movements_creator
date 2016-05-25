#!/usr/bin/env python

class UselessClass(object):
    def __init__(self):
        self.x = 1
        self.y = "jkdfhkfdjksa"

    def mymethod(self):
        print self.x
        print self.y

if __name__ == '__main__':
    u = UselessClass()
    u.mymethod()
