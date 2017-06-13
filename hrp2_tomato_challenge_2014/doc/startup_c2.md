# C2 startup

## Before startup

Add udev rules for dxhub

```
$ sudo emacs -nw /etc/udev/rules.s/99-dxhub.rules
```

then

```
SUBSYSTEMS=="usb",ATTRS{idProduct}=="6015",ATTRS{idVendor}=="0403",MODE="666",GROUP="dialout"
```

## Start Up

- turn up switch, turn up relay
- login leus@tomatillo

- startup launchers
-- `$ roslaunch hrp2_tomato_challenge_2014 currentor_type2.launch`
-- `$ roslaunch hrp2_tomato_challenge_2014 c2.launch`

and check pose using rviz, to avoid error of potentio meter.
If potentio meter includes large error, then reset power using main switch.

## Servo On

- `$ rosrun aria_utils ariacore`

## ServoOff

- `$ rosrun aria_utils aria servo_off`

## Operate by keyboard

- `roscd hrp2_tomato_challenge_2014/euslisp`
- `rosrun roseus roseus`
-- emacs shell is better
- `(load "setup-c2-tomato.l")`
- `(c2-reset-manip-pose)`
- `(c2-swing-waist 45)`
- `(c2-move-lhand-vi)`