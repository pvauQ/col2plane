#!/bin/bash

cp transform.txt ./photodir/malli/transform.txt
cp scale_trans.txt ./photodir/malli/scale_trans.txt

# Navigate to transform directory
cd ./photodir/malli


colmap model_transformer --input_path . --output_path ./transformed --transform_path transform.txt --is_inverse 0
colmap model_transformer --input_path ./transformed --output_path ./transformed --transform_path scale_trans.txt --is_inverse 0

#colmap model_transformer --input_path . --output_path ./transformed --transform_path scale_trans.txt --is_inverse 0
#colmap model_transformer --input_path ./transformed --output_path ./transformed --transform_path transform.txt --is_inverse 0



echo "Transformation complete. model in ./transformed"