## Litter collection 

High level control nodes for planning paths based on the perception module for the litter collection. 

Collect litter with the Bezier curve (quadratic one, for this case). 

### Run node 

Run node with: 
```
roslaunch uam_ros_litter litter_ctl.launch 
```

### yaml-cpp 

In order to use `yaml-cpp` run following commands before building package: 
```
git clone https://github.com/jbeder/yaml-cpp.git --branch yaml-cpp-0.6.3
cd yaml-cpp
export CXXFLAGS="$CXXFLAGS -fPIC"
mkdir build && cd build
make -j12
sudo make install
```


### TODO: 
- [x] Add config for the topic names 
- [ ] Add states || conditions for the pickup 
- [ ] Test with the movable object + instance segmentation 

