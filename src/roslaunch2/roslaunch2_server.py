#!/bin/env python
# coding=utf-8

import Pyro4
import socket
import roslaunch2
import Pyro4.naming
import getpass
import termcolor
import sys

__version__ = '0.1'


class Server:
    def __init__(self):
        self.machine_address = Server.get_ip_address()
        self.reg_db = {}
        print(termcolor.colored('This is the roslaunch2 PyRO server v{:s}, '
              '(C) Copyright by Adrian Böckenkamp 2017.\n'.format(__version__), 'blue'))
        Pyro4.config.REQUIRE_EXPOSE = True

        if not Server.is_name_server_running():
            print(termcolor.colored('- Cannot find a running PyRO name server, please start it with:\n', 'red') +
                  termcolor.colored('  $ python -m Pyro4.naming -n $(hostname -I)\n', 'yellow'))
            sys.exit(1)
        else:
            print('- Name server found.')

        self.daemon = Pyro4.Daemon(self.machine_address)  # make a Pyro daemon (NOT on the localhost)
        print('- Started daemon on {:s}.'.format(self.machine_address))
        self.ns = Pyro4.locateNS()  # find the name server

        self.register_remote_class(roslaunch2.package.Package)
        self.register_remote_class(roslaunch2.remote.Internals)
        self.register_remote_class(roslaunch2.remote.API)
        print('- Registered the following types on this machine:')
        for k in self.reg_db:
            print('  * {:s} -> {:s}'.format(k, self.reg_db[k]))

    @staticmethod
    def is_name_server_running():
        with Pyro4.Proxy("PYRONAME:Pyro.NameServer") as p:
            try:
                p._pyroBind()
                return True
            except Pyro4.errors.PyroError:
                return False

    @staticmethod
    def get_ip_address():
        # http://stackoverflow.com/questions/24196932/how-can-i-get-the-ip-address-of-eth0-in-python
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        return str(s.getsockname()[0])

    def register_remote_class(self, cls):
        name = self.machine_address + '.' + getpass.getuser() + '.' + str(cls)
        uri = self.daemon.register(cls)
        self.ns.register(name, uri)
        self.reg_db[name] = uri

    def main(self):
        print(termcolor.colored('\nReady, entering request processing loop.', 'green'))
        self.daemon.requestLoop()  # start the event loop of the server to wait for calls
        pass


if __name__ == "__main__":
    Server().main()
