

start wih colmap model
1 - [] sort colmap images by lovest error
2 - [] find ccTags from best images
3 - [] calculate p3p for n best image tag pairs (location of camera)
4 - [] calculate difrence between colmap image location and one we calulate 
5 - [] store this is in transforms.txt as S quat trans
6 - [] aja colmap model_transform tällä.
7 - [] model_aligner vs model_transformer


with ccTag we want to do different modes, use opencv for now
[] 4 tags pnp no ambiquity
[] 3 tags use pnp multiple images to  take care of ambiquity
[x] 3 tags  with ambiquity
[] 2 tags use homography and circle ellipse property to calculate plane. and locations for orientation
[] 2 tags use pnp from multiple images
[] single tag, use homography to estabilish a a plane.


[] figure out how to make cctag have actual outputfile, so i dont need to parse, or use as lib
[] figure out if it's usefull to have cctag as lib in the proces
[] make a script that automates all this!


for all math convert to eigen as soon as possible. (opencv things look cursed)

