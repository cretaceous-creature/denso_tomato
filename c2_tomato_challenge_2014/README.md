* How to use

** install

C2 depends on Currentor/Aria

```
$ wstool set currentor/aria https://github.com/Currentor/Aria.git --git
$ wstool update currentor/aria
$ cd currentor/aria
$ git remote add hyaguchijsk https://github.com/hyaguchijsk/Aria.git
$ git checkout -b tomato_challenge_2014 hyaguchijsk/tomato_challenge_2014
$ catkin build aria_utils
$ catkin build ros2http
$ catkin build aria_model
$ roscd aria_model
$ ./gen-aria.sh
```


** login

Default: Robot PC (Thinkpad) is tomatillo.

(If the PC is updated or network changed, please change the following IPs.)

```
tomatillo 192.168.101.146
```

First of all, please login leus@tomatillo

```
ssh -X
```


** byobu

```
$ byobu
```

- [F2] to create new tab
- [F3][F4] to cheange tab




** roscore

```
$ roscore
```


** launch currentor socket

```
$ roslaunch hrp2_tomato_challenge_2014 currentor_type2.launch
```

** launch ros bridge

```
$ roslaunch hrp2_tomato_challenge_2014 c2.launch
```


** send servo off command (At first time after power on)

```
$ rosrun aria_utils aria servo_off
```

It returns immediately.



** rviz (on client PC)

At first, please set envs.

```
export ROS_MASTER_URI=http://192.168.101.146:11311
export ROS_IP=192.168.101.146
export ROS_HOSTNAME=192.168.101.146
```

then, run rviz.

```
rosrun rviz rviz
```

add RobotModel and check robot pose.



** servo on and go initial

```
$ rosrun aria_utils ariacore
```

It returns immediately.


** run euslisp interface

```
$ roscd hrp2_tomato_challenge_2014/euslisp
$ emacs -nw
M-x shell
$ rosrun roseus roseus
$ (load "setup-c2-tomato.l")
$ (real2model)
```

*** reset pose

```
$ (c2-reset-manip-pose)
```

If robot doesn't move, please type `(real2model)` and retry it.


*** scissors control

```
$ (c2-stop-cut)
$ (c2-start-cut)
```

*** base control

NOTE that the forward direction of base is the backward of the body.

```
$ (c2-move-forward)
$ (c2-move-forward 3000)
$ (c2-move-back)
$ (c2-move-left)
$ (c2-move-right)
```

*** tomato manipulation

```
$ (c2-reset-manip-larm)
$ (c2-move-lhand-vi)
```

move-lhand-vi is vi-like interface,

- f: forward
- b: backward
- h: right
- j: down
- k: up
- l: left


*** put tomato pose

```
$ (c2-reset-manip-pose)
$ (c2-put-tomato)
$ (c2-stop-cut)
```
