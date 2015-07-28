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
  $ (pr)  ;; reset man-ip-pose
```

### start cutting

On the roseus console:

```
  $ (move-to-tomato)
  x
  g
  c
  r
  p
  s
  r
  q
```

| command | description |
|:--------|:------------|
| q | Quit function |
| x | Approach to the tomato |
| g | Go to pedicel |
| c | Cut |
| r | Reset pose |
| p | Put to tray |
| s | Stop cut |
| r | Reset pose |
| m | Alter motion (teleop) |

