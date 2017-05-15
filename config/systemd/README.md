This guide explains how to setup a PyRO name server as well as the roslaunch2 server (takes ~5 mins) using [*systemd*](https://freedesktop.org/wiki/Software/systemd/) (fully available in Ubuntu since 15.04; thus, for Ubuntu 14.04 / Indigo, you'll need the old [*Upstart*](https://wiki.ubuntuusers.de/Upstart/)).

# Setting up a PyRO name server:
- Note that **a PyRO name server should only be started *once* per setup** (for all robots / simulations); otherwise you end up with a partitioned name space, see [here](https://pythonhosted.org/Pyro4/nameserver.html).
- First, edit the file `$(rospack find roslaunch2)/config/systemd/pyro_name_server.service` and fix the path in `ExecStart` as well as the user in `User`. The rest should be fine.
- Once done, copy the file to the default location systemd scripts:
```
sudo cp $(rospack find roslaunch2)/config/systemd/pyro_name_server.service /etc/systemd/system/
```
- Add/enable the service unit to systemd: `sudo systemctl enable pyro_name_server.service`
- Test if you can run it with: `sudo systemctl start pyro_name_server.service` (shouldn't produce any output). Then `sudo systemctl status pyro_name_server.service` should show something like:
> ● pyro_name_server.service - PyRO Name Server
>   Loaded: loaded (/etc/systemd/system/pyro_name_server.service; enabled; vendor preset: enabled)
>   Active: active (running) since Mo 2017-05-15 15:07:00 CEST; 6s ago
> Main PID: 14033 (pyro_name_serve)
>   CGroup: /system.slice/pyro_name_server.service
>           ├─14033 /bin/bash -l /home/abo/Development/SmartMAPS/catkin_ws/src/roslaunch2/config/systemd/pyro_name_server.bash
>           └─14163 python -m Pyro4.naming -n 129.217.52.228
>
> Mai 15 15:07:00 boeckenkamp systemd[1]: Started PyRO Name Server.
- Reboot and check if the server is running:
```
sudo reboot
# After reboot:
ps aux | grep Pyro4\.naming
```
- Logging output can be viewed with:
```
sudo journalctl -ru pyro_name_server
```

# Setting up a roslaunch2 server:
- Note that **a roslaunch2 server is required to be running on all machines that should support roslaunch2's remote API**. (If such features are not required, the server does not need to run and all other features of roslaunch2 still work.)
- The process is similar to setting up the name server, see above for more details.
- Like for the PyRO name server (see above), fix the file `$(rospack find roslaunch2)/config/systemd/roslaunch2_server.service` according to your system configuration (path and user name).
- Clearly, on a system without a PyRO name server (and its associated service unit file), you also need to change:
```
Wants=pyro_name_server.service
After=pyro_name_server.service
```
to
```
Wants=network-online.target
After=network.target network-online.target
```
in `roslaunch2_server.service`.
- Then, type:
```
sudo cp $(rospack find roslaunch2)/config/systemd/roslaunch2_server.service /etc/systemd/system/
sudo systemctl enable roslaunch2_server.service
sudo systemctl start roslaunch2_server.service
sudo reboot
# After reboot:
ps aux | grep roslaunch2_server
```

# Remarks:
- To remove a systemd unit, type `sudo systemctl disable $SERVICE_UNIT_FILE`. This only works if you haven't made changes that prevents systemctl to find the correct file. In such cases, remove the appropriate file in `/etc/systemd/system/$TYPE_GIVEN_IN_INSTALL_SECTION_WANDED_BY.wants/` manually.
- Unfortunately, **`sudo` is required** here. Although there is a `systemctl --user` option, it is not possible to execute a script on reboot. (You can try to use this but you will need to login.) [This](https://unix.stackexchange.com/a/192714) may help partially.
- We can probably generate a `.service` file from a given `.service.in` template file using CMake/Python (TODO). The above procedures then can refer to the generated and installed files which are specifically created for the system of interest.
