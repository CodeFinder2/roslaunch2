This guide explains how to setup a PyRO name server as well as the roslaunch2 server (takes ~5 mins) using [*Upstart*](https://wiki.ubuntuusers.de/Upstart/) (replaced in Ubuntu since 14.10; thus, for Ubuntu 16.04 / Kinetic, you'll want to use [*systemd*](https://freedesktop.org/wiki/Software/systemd/)).
See [*Upstart HowTo*](https://help.ubuntu.com/community/UbuntuBootupHowto) or `man 5 init` for further information on how to write use/write an Upstart service.

# Setting up a PyRO name server:
- Note that **a PyRO name server should only be started *once* per setup** (for all robots / simulations); otherwise you end up with a partitioned name space, see [here](https://pythonhosted.org/Pyro4/nameserver.html).
- First, edit the file `$(rospack find roslaunch2)/config/upstart/roslaunch2-pyro-name-server.conf` and fix the path in the last line after `exec`. The rest should be fine.
- Once done, copy the file to the default location upstart scripts:
```
sudo cp $(rospack find roslaunch2)/config/upstart/roslaunch2-pyro-name-server.conf /etc/init/
```
- Update list of available upstart jobs: `sudo initctl reload-configuration`
- Test if you can run it with: `sudo initctl start roslaunch2-pyro-name-server`. This should show something like (different PID):
> roslaunch2-pyro-name-server start/running, process 5114
-  Then either `sudo initctl status roslaunch2-pyro-name-server` or `sudo initctl list` should show the same entry. Note that those commands without `sudo` only show a subset of jobs, apparently without our `roslaunch2-pyro-name-server`.
- Reboot and check if the server is running:
```
sudo reboot
# After reboot:
ps aux | grep Pyro4\.naming
```

# Setting up a roslaunch2 server:
- Note that **a roslaunch2 server is required to be running on all machines that should support roslaunch2's remote API**. (If such features are not required, the server does not need to run and all other features of roslaunch2 still work.)
- The process is similar to setting up the name server, see above for more details.
- Like for the PyRO name server (see above), fix the file `$(rospack find roslaunch2)/config/upstart/roslaunch2-server.conf` according to your system configuration (path).
- Depending on whether you are on a system with a PyRO name server (and its associated upstart job file) or not, you need to choose one of the following start options in `roslaunch2-server.conf`.

  On systems with a PyRO name server use:
```
# Start after RyRO name server is available
start on started roslaunch2-pyro-name-server
```
  Otherwise use:
```
# Start after networking is available
start on started network-services
```
  The respectively other line can be commented out or deleted.
- Then, type:
```
sudo cp $(rospack find roslaunch2)/config/upstart/roslaunch2-server.conf /etc/init/
sudo initctl reload-configuration
sudo initctl start roslaunch2-server
sudo reboot
# After reboot:
ps aux | grep roslaunch2_server
```
