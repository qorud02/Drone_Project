#!/usr/bin/env python

import os, time, sys, termios, atexit, tty
from select import select

# class for checking Keyboard input
class GetChar:
  def __init__(self):
    # save the terminal setting
    self.fd = sys.stdin.fileno()
    self.new_term = termios.tcgetattr(self.fd)
    self.old_term = termios.tcgetattr(self.fd)
    
    # New terminal setting unbuffered
    self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
    termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
    
    # Support normal-terminal reset at exit
    atexit.register(self.set_normal_term)
    
    
  def set_normal_term(self):
    termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)
    
  def getchar(self):  # get 1 byte from stdin
    """ returns a keyboard character after getch() has been called """
    return sys.stdin.read(1)
  
  def chk_stdin(self):  # check keyboard input
    """ returns True if keyboard character was hit, False otherwise """
    dr, dw, de = select([sys.stdin], [], [], 0)
    return dr
