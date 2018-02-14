- Q: When launching, you get:</br>
"Pyro4.errors.CommunicationError: cannot connect to (u'192.168.178.12', 41544): [Errno 111] Connection refused"

  => You need to start the roslaunch2_server instance on the remote host first.

- Q: After launching `roslaunch2_server` on a host, you get:</br>
error: cannot find a running PyRO name server, please start it with:</br>
  `$ python -m Pyro4.naming -n $(hostname -I | grep -o '^\S*')`

  => As the output implies, PyRO requires a name server to be started (only) ONCE on any host inside your subnet. You can simply copy-paste-run the above command to do so. Alternatively, you can setup an upstart/systemd script, see docs in `roslaunch2/config/{systemd,upstart}/`
