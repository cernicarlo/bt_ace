# PCL geometric primitives detector
ros package to run PCL

## Installation
* before starting install [PCL 1.14](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.14.1) manually. [Here](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html) more info.

```bash
 wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.1.tar.gz
 tar -xzf pcl-1.14.1.tar.gz
 cd pcl-pcl-1.14.1
 mkdir build
 cd build
 cmake ..

 # the number depends on how many jobs you want to use
 make -j10
 sudo make -j10 install
```
* install groq (`pip install groq`)
* install graphviz (`pip install graphviz`)

