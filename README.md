# What is this?

Input a serial of GPS points and choose some points from them and save them.
Shortcut keyword:
1. Esc: quit
2. w: save chosen points
3. r: remove the last points
4. m: change mode between *free* and *constraint*

# Note

1. `waitKey(int)`: make sure int value is not ignored
2. `namedWindow()` should be put front of `setMouseCallback`

# usage

*extract_from_rosbag.py* is used for extracting GPS points from rosbag. The topic is called 'ros_dwdx' and its type is user-defined. You can change it to your own type and topic. Run the python script, you can get a result file call res.txt. I have put it in **data** dirtory.  
You can run c++ executable file with the following command to extract key points from data/res.txt.  
```sh
mkdir build
cd build
cmake ..
make -j
./extract_data_rosbag
```
