#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 13/03/2018

termcolor_avail = False
try:
    import termcolor
    termcolor_avail = True
except ImportError:
    pass


def init_logger(want_colors=True):
    """
    Initialize the logging system.

    :param want_colors: True to allow colored output, False to disable it
    """
    global termcolor_avail
    if termcolor_avail and not want_colors:
        termcolor_avail = False


def critical(args, exit_code=1):
    """
    Print a critical error message. After such errors, the script exits with the given code.

    :param args: data to be printed
    :param exit_code: code to be used when exiting the program
    """
    t = 'error: ' + args
    print(termcolor.colored(t, 'red') if termcolor_avail else t)
    import sys
    sys.exit(exit_code)


def error(args):
    """
    Print an error, added for completeness.

    :param args: data to be printed
    """
    t = 'error: ' + args
    print(termcolor.colored(t, 'red') if termcolor_avail else t)


def warning(args):
    """
    Print a warning, added for completeness. Please prefer warnings.warn().

    :param args: data to be printed
    """
    t = 'warning: ' + args
    print(termcolor.colored(t, 'yellow') if termcolor_avail else t)


def log(args):
    """
    Print a message w/o any prefix.

    :param args: data to be printed
    """
    print(termcolor.colored(args, 'cyan') if termcolor_avail else args)
