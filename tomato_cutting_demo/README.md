# tomato_cutting_demo

## How to use

### start tomato detect

```
  $ roslaunch tomato_cutting_demo start_tomato_detect.launch
```

### start euslisp commander

```
  $ roscd tomato_cutting_demo/euslisp
  $ roseus cut-tomato.l
```

### start cutting

On the roseus console:

```
  (move-to-tomato)
  (send-angle-vector)  ;; robot move!!
  (move-to-pedicel)
  (send-angle-vector)  ;; robot move!!
  (start-cut)  ;; scissors move!!
  (send-reset-manip-pose)  ;; robot move!!
  (reset-place-pose)  ;; scissors and robot move!!
  (send-reset-manip-pose)  ;; robot move!!
```

