# Stereo-Plane-SLAM
**Authors:** 
Zhang, Xiaoyu; Wang, Wei; Qi, Xianyu; Liao, Ziwei. 

Plane  feature  is  a  kind  of  stable  landmark  to reduce drift error in SLAM system. It is easy and fast to extract planes  from  dense  point  cloud,  which  is  commonly  acquired from RGB-D camera or lidar. But for stereo camera, it is hard to compute dense point cloud accurately and efficiently. In this project, we propose a novel method to compute plane parameters using  intersecting  lines  which  are  extracted  from  the  stereo image.  The  plane  features  commonly  exist  on  the  surface  of man-made  objects  and  structure,  which  have  regular  shape and  straight  edge  lines.  In  3D  space,  two  intersecting  lines can  determine  such  a  plane.  Thus  we  extract  line  segments from both stereo left and right image. By stereo matching, we compute  the  endpoints  and  line  directions  in  3D  space,  and then  the  planes  from  two  intersecting  lines.  We  discard  those inaccurate  plane  features  in  the  frame  tracking.  Adding  such plane  features  in  stereo  SLAM  system  reduces  the  drift  error and  refines  the  performance.  We  test  our  proposed  system  on public  datasets  and  demonstrate  its  robust  and  accurate  estimation results, compared with state-of-the-art SLAM systems.

### Related Publications:
For more details, please read our paper [Stereo  Plane  SLAM  Based  on  Intersecting  Lines](https://arxiv.org/abs/2008.08218), which has been accepted by IROS 2021.

If you have some questions to discuss, you could contact me by e-mail (zhang_xy@buaa.edu.cn) for fast reply.

If you use our system in an academic work, please cite the paper.
```
@INPROCEEDINGS{9635961,  
  author={Zhang, Xiaoyu and Wang, Wei and Qi, Xianyu and Liao, Ziwei},  
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},   
  title={Stereo Plane SLAM Based on Intersecting Lines},   
  year={2021}, 
  pages={6566-6572},  
  doi={10.1109/IROS51168.2021.9635961}
}
```
### Video

https://www.youtube.com/watch?v=3VWF-JJU9T8

# Stereo-Plane-SLAM System
We build our SLAM system based on ORB-SLAM2 Stereo version. For some prerequisites, you could read their page, [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)

We only add PCL library to deal with point cloud, we tested on PCL-1.80. The system only supports Stereo sensor. We have not removed those unrelevent components coming from ORB-SLAM2.
