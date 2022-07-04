<p align="right">
    <b> <img src="https://img.shields.io/badge/Supported%20Platforms-Windows%20%7C%20macOS%20%7C%20Linux-green" title="Supported Platforms"/> </b> <br>
    <b> <img src="https://img.shields.io/badge/license-GPL-blue" title="license-GPL"/> </b> <br> <br>
</p>


Tree Growth Modelling (GTree) implements the tree reconstruction method described in the following [thesis](https://repository.tudelft.nl/islandora/object/uuid:d284c33a-7297-4509-81e1-e183ed6cca3c/datastream/OBJ/download):
```
Noortje van der Horst.
Procedural Modelling of Tree Growth Using Multi-temporal Point Clouds.
Delft University of Technology. 2022.
URL: http://resolver.tudelft.nl/uuid:d284c33a-7297-4509-81e1-e183ed6cca3c.
```


GTree depends on some third-party libraries and most dependencies are included in the distribution except
[Boost](https://www.boost.org/). So you will need to have Boost install first. 

You need [CMake](https://cmake.org/download/) and of course a compiler to build GTree:

- CMake `>= 3.1`
- a compiler that supports `>= C++17`


### Usage
In order to reconstruct tree growth, please input multiple cleaned .xyz point clouds of the same tree at different dates.
Use the TreeGrowthInterpolation algorithm to reconstruct the skeleton structures for the input timestamp data.
Reconstructions are:
- A merged main skeleton corresponding between all timestamps, which describes the main branching strucutre of the tree over time.
- Smaller branches in the tree crown, represented by lobe hulls and generated using region growing.
- A timestamp-specific skeletonization of the main structure of all individual input timestamps.
- Branch geometry of both the merged main + lobes and timestamp-specific skeletons, made using cylinder fitting.

In order to interpolate and animate the changes between individual timestamp main reconstructions, use the Interpolator algorithm.
Import both the needed correspondence path files and the accompanying timestamp-specific skeleton graph reconstructions.
The required files can be outputted with the TreeGrowthReconstruction algorithm.
Intermediary steps between the known timestamps can then be interpolated and animated. Use the arrow keys to cycle through the steps.
Branch geometry of all steps can also be generated via the "processing" menu, press "shift + 'b'" to turn the visualisation of them on or off.


### Data
Some test tree point clouds are provided in the 'resources/data/GTree_examples/' folder.

**Note:** When testing on your point clouds, please make sure that:
- your point cloud represents a single tree (i.e., the tree is segmented out from the background; no ground, no fence...);
- the tree has an upright orientation (i.e., with Z-axis pointing up);
- the point cloud does not contain excessive noise.

---
### Citation
If you use the code/program (or part) of GTree in a scientific work, please cite the source thesis:

```bibtex
@article{van2022procedural,
  title={Procedural Modelling of Tree Growth Using Multi-temporal Point Clouds},
  author={van der Horst, Noortje},
  year={2022}
}
```

---

### License
This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License or (at your option) any later version. The full text of the license can be found in the accompanying LICENSE file.

---

Should you have any questions, comments, or suggestions, please contact us at liangliang.nan@tudelft.nl

3D Geoinformation Research Group, TU Delft,

https://3d.bk.tudelft.nl,

July 2, 2022