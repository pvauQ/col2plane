### work in progress !!

This is utility software to align and  to establish absolute scale for colmap model. This is achieved by using visual markers (cctag) and their known positions.

There are two distinct operation modes:
4 tags visible from least 3 cameras; simple p3p  -> outputs camerea_locations.txt in format expected by colmap model_aligner

Least 3 tags in the scene;  each tag has to bee seen least from 3 cameras, but no camera needs to see multiple markers. This uses Levenberg–Marquardt algorithm, to solve transformation that takes any point in frame defined in colmap frame to  world frame defined via tag locations. This outputs 2 txt files scale_trans.txt and transform.txt use those with  colmap model_transformer (run_model_transformer.sh script can be used)

## Usage
1. place cctags in scene
2. specify locations of tags to file tag_world_pos.txt
3. capture photographs of your scene.
4. place in photodir/malli
5. create colmap model of scene , export is as txt (in malli folder)
6. run col2plane, specify model to use, output of cctag step is saved to tags.txt and this can be used to skip calculating cctags on following runs.
7. software generates output files that can be used to transform colmap scene.

## files generated or used
tag_world_pos.txt -> specify world locations of tags you are using line starting # is a comment
example:
# one per line: "id x y z"
0 -0.429 0 0
2  0      0 0
3  0.520  0 0

tags.txt
Results from cctag marker detection are stored here.
example:
# one image per line: "filename x y id x y id"....
_MG_0965.JPG 1279.26000977 2357.82519531 2 3398.65429688 2137.80200195 3 

transform.txt
file that can be used with colmap model_transformer this is scale quat trans, this does not have scale baked in
example:
1 0.915818315675864 0.397647905556966 -0.0162397421485741 0.0537515269897509 0.263553364166345 2.45591097150966 -2.7730467380097
scale_trans.txt
exactly same but just with scale component:
0.378878802061081 1 0 0 0 0 0 0

camera_locations.txt
This is produced when using p3p solve mode.
this can be used with colmap model_aligner
example
filename  x y z



## important
cctag has some problems on finding tags on some modern systems, check that it works on your system, by running it against their test set. this can be done bu building cctag and runing their  detection binary.
This problem is somehow related to using auto type with some Eigen structures(some UB changed on some lib that is shipped with least ubuntu 24.04)
"In short: do not use the auto keywords with Eigen's expressions, unless you are 100% sure about what you are doing" 
https://libeigen.gitlab.io/eigen/docs-nightly/TopicPitfalls.html
https://github.com/alicevision/CCTag/issues/219
https://github.com/alicevision/CCTag/issues/226



## installation
To build this you need 
cctag, opencv , eigen. and all their depencies
https://github.com/alicevision/CCTag

mkdir build
cd build
cmake
make
