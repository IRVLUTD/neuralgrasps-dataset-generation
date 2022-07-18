# Prepare YCB objects for PyBullet loading 

## Generating the URDFs

Simply go through the script `ycb_prepare_urdf.py` with appropriate command line flags:

You will only need to pass the `-f` (`--ycb_dir`) flag to point to the location
of ycb models. For example it can then be:
`python ycb_prepare_urdf.py -f /mnt/Data/ycb-objects/ycb-tools/models/ycb/`

The `/mnt/Data/ycb-objects/ycb-tools/models/ycb/` contains the ycb object folders like
`001_chips_can/`, `003_cracker_box/` etc..

### Default Params
- `-m` (`--models_file`) is already the one that is default specified
- `-o` can be anywhere on your system so long as you remember it for the point cloud
  generation process. Or to make matters easy, copy the generate urdfs to 
  `../rendering-test/ycb-urdfs/`
- `-t` is the mesh type. No need to change it.

## Script Explanation

Pybullet requires URDF files for loading. So we need to write a script that'll
take the `.obj` files for a model (e.g. cheezit box id=003) and have a simple
`.urdf` generated for it.

A sample urdf file is provided: `sample_003_cracker_box.urdf`

Nothing needs to change in the urdf apart from the following. All three derived
from the exact model we want to create urdf of.

1. urdf file name!
2. robot name xml tag
3. baseLink mesh filename
