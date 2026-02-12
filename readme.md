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
7. software genereates output files that can be used to transform colmap scene.

## important
cctag has some problems on finding tags on some systems, check that it works on your system, by running it against their test set.

## installation
To build this you need 
cctag, opencv , eigen. and all their depencies
https://github.com/alicevision/CCTag

