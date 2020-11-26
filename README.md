# Goal

Input a serial of GPS points and choose some points from them and save them.
Shortcut keyword:
1. Esc: quit
2. w: save chosen points
3. r: remove the last points
4. m: change mode between *free* and *constraint*

# Note

1. `waitKey(int)`: make sure int value is not ignored
2. `namedWindow()` should be put front of `setMouseCallback`
