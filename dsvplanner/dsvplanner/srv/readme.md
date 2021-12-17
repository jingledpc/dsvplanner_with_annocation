# dsvplanner_srv.srv
获得目标点和对应模式
``` c++
geometry_msgs/Point[] goal
std_msgs/Int32 mode
```

# clean_frontier_srv.srv
清除waypoint（frontier）点服务
```
Header header
-------
bool success 
```

