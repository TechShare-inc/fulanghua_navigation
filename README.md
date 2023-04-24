# fulanghua最小構成

scan, odom, amcl, move_baseができている状態で、waypoint_navを使うための最小構成 

```
$ roslaunch fulanghua_waypoints_nav nav.launch
$ roslaunch orne_waypoints_editor edit_waypoints_viz.launch
```


以下のような amcl + move_base が出来ている状態で
（  https://github.com/TechShare-inc/go1_nav_rplidar_s1  ）

https://user-images.githubusercontent.com/106230925/226292479-b589329a-55cc-4851-b8dd-2a9f7761e11d.mp4


本プログラムを使うと・・・


https://user-images.githubusercontent.com/106230925/227402044-a1424b58-998d-43dd-8819-c1a4149135b1.mp4



他動画  
https://techshareinc.sharepoint.com/:f:/s/UnitreeMaterialStorage/EjfQbhpWtyZKpjzwgcvtWrcByLui2wD4N5U-ZshtBjwdNw?e=T2IgWZ

# build

なぜか一発でいけない
```
$ catkin_make --only-pkg-with-deps fulanghua_action
$ catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
