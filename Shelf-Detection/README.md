# Automatic shelf detection using point clouds of indoor warehouse environments
## Introduction
This work is part of my master thesis at Örebro University (January - June, 2022). 

Semantic labeling of shelves was, in this thesis, done by finding the supporting poles of shelves in the environment and classifying them as either shelf or non-shelf depending on specified dimensions of shelves searched for.

## How it works
It starts by creating a neighborhood $n$ for each point $p$ in the point cloud $P$ resulting in a set of neighborhoods $N$ = \{n<sub>1</sub>, n<sub>2</sub>, ..., n<sub>n</sub>\}. The generated $n \in N$ are then classified as valid or invalid depending on the alignment within three-dimensional space by principal component analysis. Valid neighborhoods, in this case, are those that are shaped as a line and vertical *enough* and are kept represented as their mean $\mu_i$ where $\mu_i$ is the mean of the $ith$ neighborhood.

Valid neighborhoods are then classified as either shelf or non-shelf by searching for all PCA valid neighborhoods within a radius $r_s$. Found neighborhoods are then examined if they construct either parallel or perpendicular vectors using examined neighborhood as origin. Furthermore the score of the examined neighborhood gets increased by $1$ for each pair of neighbors that are deemed perpendicular or parallel based on an angular threshold $\phi$. Neighborhoods with a score $S$ where $S$ < $k$ gets removed; they are considered to be misclassifications or invalid.

Remaining neighborhoods are then segmented using region growing to gain a set of racks. A bounding box is then constructed for each of the racks within the environment using a rotating caliper based approach. For each of the segmented bounding boxes pick slots are placed along one of the longest sides (or both in cases where the shelves are standing back-to-back).

## Dependencies
- Point Cloud Library (PCL) --- `sudo apt-get install libpcl-dev`
- OpenMP --- `sudo apt-get install libomp-dev`
- JSON (included on git)
- Eigen (should get installed by issuing `sudo apt-get install libpcl-dev`)

## Settings
All settings are found in `settings/settings.json` and following in this section are a brief explanation of each one

### OpenMP
- `nrThreads` --- how many threads to use when multi-threading (PCA classification is multi-threaded)

### Poles
- `radius` --- radius used to generate the initial set of neighborhoods
- `linearThreshold` --- how narrow a neighborhood should be to be considered a pole (smaller value $\implies$ narrower neighborhood)
- `minPolePoints` --- how many points a neighborhood should be to even be considered
- `xRatio` --- how dominant $z$ should be compared to $x$
- `yRatio` --- how dominant $z$ should be compared to $y$

### Visualization
- `visualizeSegmentedPoles` --- visualize PCA valid neighborhoods
- `visualizeIndividualBoxes` --- visualize individual boxes in the order they are created
- `visualizeScoredSegments` --- visualize segments kept post scoring
- `visualizeSegmentedRacks` --- visualize segmented racks in different colors
- `visulizePickSlots` --- visualize distributed pick slots and their corresponding bounding box

### Downsampling
- `enabled` --- enables / disables downsampling
- `leafSize.x,  y, z` --- specifies dimensions of voxel

### Filtering
- `enabled` --- enables / disables filtering of points
- `fieldName` --- axis to filter
- `minHeight` --- minimum value
- `maxHeight` --- maximum value

### Radius Segmenter
- `radius` --- radius used when segmenting with region growing to generate racks of valid poles
- `minPoints` --- minimum number of neighborhoods to be considered a rack

### Shelf dimensions
- `dims` --- dimensions of shelf types searched for
  - `typeA` --- first type
    - `widths` --- expected distance between two poles in terms of width
    - `depths` --- expected distance between two poles in terms of depth
- `distanceThreshold` --- specifies the accepted difference between specified dimensions for a pole to be considered a depth or width
- `angleDiffDeg` --- angular threshold used to classify a pole as perpendicular and/or parallel
- `shelfScore` --- minimum score for poles to be kept for segmentation
- `searchRadius` --- the radius used to search for neighboring poles

### Merging
- `enabled` --- enables/disables the merging of neighboring poles (this **should** be enabled)
- `radius` --- radius which is used to construct merged poles

## Running
The developed software was tested on Ubuntu 20.04 with PCL 1.10.<br>Prior to running edit the settings in `settings/settings.json` to your liking.
```cmake
mkdir build && cd build && cmake .. && make -j8
```
Then execute it as follows `./shelf-detection <path to map> <path to settings>`


## Adding more than one shelf type
```json
"dims": {
    "first_type": {
        "widths": [widths of first_type],
        "depths": [depths of first_type]
    },
    "second_type": {
        "widths": [widths of second_type],
        "depths": [depths of second_type]
    }
}
```