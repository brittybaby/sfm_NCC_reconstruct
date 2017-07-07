# sfm_NCC_reconstruct


NCC based correspondence

This repository includes the structure from motion pipeline of opencv and the input points are obtained using the NCC based correspondences from images. The NCC based correspondence is found in using windows initialised at the first frame and finding the corresponding match window in the next frame. The features are tracked for a given number of count and the points thus tracked are provided to the structure from motion pipeline of opencv (sfm::reconstruct)

The reconstruction happens for the given number of count and then the features are re-initialized.
