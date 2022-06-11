# Robust frequency-based Structure Extraction (ROSE) in 3D
## Introduction
This work is part of my master thesis at Örebro University (January - June, 2022). 

[ROSE](https://github.com/tkucner/rose), by [Kucner et al.](https://arxiv.org/pdf/2004.08794.pdf), is a method used to remove clutter from two-dimensional grid maps. This method aims to naively solve that. Naively in the sense that ROSE was not implemented to natively deal with environments in the three-dimensional frequency spectrum of a point cloud but instead three different methods of creating the needed grid maps were implemented.

## Dependencies
Apart from the ones used by the original implementation of ROSE the following dependencies are required

- NumPy --- `python -m pip install numpy`
- Open3D --- `python -m pip install open3d`
- PyYAML --- `python -m pip install PyYAML`
- PIL --- `python -m pip install Pillow`
- matplotlib --- `python -m pip install matplotlib`

## Settings
All settings are found in `settings/run_settings.json`

### Preparations
- `height_filtered` --- specifies if points where `point.z <= min_height or points.z >= max_height`<br>
should be removed from grid map creation
- `downsample` --- enables downsampling using voxels of `voxel_size`

### Naive projection
- `naive_projection` --- enables/disables naive projection where the remaining points, after wanted preparations,<br>are projected onto the xy plane
- `resolution` --- sets the numbers of metres per pixel (0.1 = 10 pixels per metre)
- `pointcloud` --- three-dimensional point cloud to declutter

### Partitioning
- `partitioned` --- enables/disables partitioning where the remaining, after wanted preparations,<br>
are partitioned based on their value of the z component. Creates slices of specified `partition_height`
- `partition_height` --- how high each partition should be (in metres)
- `resolution` --- sets the numbers of metres per pixel (0.1 = 10 pixels per metre)
- `pointcloud` --- three-dimensional point cloud to declutter

### Grid map (PGM)
- `grid_map_pgm` --- enables/disables the PGM based grid map creation
- `pgm` --- pgm grid map used to filter point cloud points based on
- `pointcloud` --- three-dimensional point cloud to declutter
- `corner` --- sets the corner that (0, 0) is located at `[L/R, U/B]` for left/right, upper/bottom
- `yaml` --- YAML file detailing offsets and resolution of pgm

### Grid map (PNG)
- `grid_map_png` --- enables/disables the PNG based grid map creation
- `png`--- as with pgm but in png format
- `pointcloud` --- input point cloud
- `resolution` --- resolution **used** when creating the specified png grid map

### ROSE
- `peak_height` --- sets the parameter `peak_height` used by ROSE
- `filter_level` --- sets the parameter `filter_level` used by ROSE
- `sigma` --- sets the parameter `sigma` used by ROSE

### Reconstruction
- `insert_floor` --- if enabled inserts filtered floor points back into the decluttered point cloud
- `insert_ceiling` --- if enabled inserts filtered ceiling points back into the decluttered point cloud

### Visualization
- `visualize_input_cloud` --- Visualizes the loaded point cloud **after** preparations
- `visualize_decluttered_cloud` --- Visualizes the decluttered cloud after reconstruction
- `visualize_filtered_png` --- Visualizes point cloud **after** filtering on occupied pixels in png grid map
- `visualize_filtered_pgm` --- Visualizes point cloud **after** filtering on occupied pixels in pgm grid map

## Requirements
The method requires a specific folder structure, largely due to quickly becoming too cluttered without it.
### Folder structure
- ROSE3D
  - input
  - maps
  - pgm
  - result
  - settings
  - yaml

All of the items in the list above are folders where indented items are subdirectories of `ROSE3D`.

## Running
To run the adaptation of ROSE into 3D space run 
```python
python main.py
or
python3 main.py
```
**after** adjusting `settings/run_settings.json` to your liking. **Only one** of the methods will get executed and the priority is the order they appear in `settings/run_settings.json` which, at the time of writing, is
- `Naive projection`
- `Partitioned`
- `Grid map (PGM)`
- `Grid map (PNG)`


