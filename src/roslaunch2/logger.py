#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 04/02/2017

# System imports:
import sys
termcolor = None
try:
    import termcolor
except ImportError:
    pass


def init_logger(want_colors=True):
    """
    Initialize the logging system.

    :param want_colors: True to allow colored output, False to disable it
    :return: None
    """
    global termcolor
    if termcolor and not want_colors:
        termcolor = None


def critical(args, exit_code=1):
    """
    Print a critical error message. After such errors, the script exits with the given code.

    :param args: data to be printed
    :param exit_code: code to be used when exiting the program
    :return: None
    """
    t = 'error: ' + args
    print(termcolor.colored(t, 'red') if termcolor else t)
    sys.exit(exit_code)


def error(args):
    """
    Print an error, added for completeness.

    :param args: data to be printed
    :return: None
    """
    t = 'error: ' + args
    print(termcolor.colored(t, 'red') if termcolor else t)


def warning(args):
    """
    Print a warning, added for completeness. Please prefer warnings.warn().

    :param args: data to be printed
    :return: None
    """
    t = 'warning: ' + args
    print(termcolor.colored(t, 'yellow') if termcolor else t)


def log(args):
    """
    Print a message w/o any prefix.

    :param args: data to be printed
    :return: None
    """
    print(termcolor.colored(args, 'cyan') if termcolor else args)
