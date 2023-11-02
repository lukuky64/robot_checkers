# robot_checkers

## To Run main code:
1. Ensure you have all required files locally available

2. Open an ubuntu terminal and execute:
```Bash
roscore
```

3. Open matlab and in the console, execute:
```Bash
gb = gameBoard()
```

4. Open another ubuntu terminal, cd to `/roboCheckers/src/robot_checkers/Checkers_AI_modded` and execute:
```Bash
python3 main.py
```

5. In the matlab console, execute:
```Bash
gb.run()
```

6. To end matlab program, execute:
```Bash
gb.stop()
```


# Using physical doBot Magician:
1. In ubuntu terminal:
```Bash
roslaunch dobot_magician_driver dobot_magician.launch
```

2. In matlab:
```Bash
rosinit
```

