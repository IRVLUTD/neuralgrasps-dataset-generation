# Prepare YCB objects for Graspit

The processing code is borrowed from https://github.com/lwohlhart/mano_grasp.

Subtle changes made to the script to fix the bug of not working with current
MeshLab version along with steps to re-produce the bug-fix for future
MeshLab versions (in case the filter script becomes out-dated).

## Dex YCB list of objects
See: https://github.com/NVlabs/dex-ycb-toolkit/blob/master/dex_ycb_toolkit/dex_ycb.py (line-num: 35)
It is declared as `_YCB_CLASSES` variable.

Saved the list of objects in `object_list.txt`

textured google_512k .obj

- 002 master chef can
- 003 cracker box
- 004 sugar box
- 005 tomato soup can
- 006 mustard bottle
- 007 tuna fish can
- 008 pudding box
- 009 gelatin box
- 010 potted meat can
- 011 banana
- 019 pitcher base
- 021 bleach cleanser
- 024 bowl
- 025 mug
- 035 power drill
- 036 wood block
- 037 scissors
- 040 large marker
- 051 large clamp
- 061 foam brick

## Meshlab .mlx script

Make sure you have meshlab installed: `sudo apt install meshlab`. The given
script is for Ubuntu 20.04 version of meshlab 
(apt installed version: MeshLab_64bit_fp v2020.03+dfsg1). Depending on your
Meshlab version, the script encoded in the python file may need to be changed.

To obtain the meshlab script for your version is quite easy. Load any model and
- GUI -> Filters -> Normals.Curvature,Orientation -> Transform: Scale, Normalize
- Appply the basic filter and then GUI -> Filters -> View filter script -- save it!

This gives us the correct syntax for the script! Copy the script to the
`SCALE_FILTER_TEMPLATE` variable in `ycb_prepare_objects.py` file. 

## Python Script Args

Use this script **OUTSIDE** the docker container! (pybullet env)

Use `python ycb_prepare_objects.py --help` to see all available options.

Make sure to read the command line args:

- `--models_file` : See `./ycb_object_list_google_512k.txt` for an example. 
  To change use a simple find-and-replace for the appropriate model file to be 
  used. For example `google_64k -> google_512k ; textured.obj -> non-tex`

- `--models_folder` : Its in `/mnt/Data/ycb-objects/ycb-tools/models/ycb/`
  essentially, the folder containing the 001, 002, 003 ... object folders

- `--graspit_dir` : Give a dummy location like `./graspit`. Make sure to run
  `mkdir -p ./graspit_google_16k/models/objects/` before so that its valid. 
  CAN then COPY the files to the true graspit location (from inside the docker
  container):

  Check the dir first if the files are there not!
  
  `cp graspit_google_16k/models/objects/* /opt/graspit/models/objects/`


- `-sc` or `--scales` : Set it to 1000 (Usually you want to apply some scaling to the objects to fit the hand)
- `-n` : Set to 16 (number of jobs) 
- `-o` : output log file -- which objects were converted?

### Sample calls:

Replace `--models_folder` by path to where you downloaded YCB object data set.

**For Google 16k**
`python ycb_prepare_objects.py --graspit_dir ./graspit_google_16k/ --models_file ycb_object_list_google_16k.txt --models_folder /mnt/Data/ycb-objects/ycb-tools/models/ycb/ -n 16 -o output_objects_log.txt -sc 1000 `

**For Google 64k**
`python ycb_prepare_objects.py --graspit_dir ./graspit_google_64k/ --models_file ycb_object_list_google_64k.txt --models_folder /mnt/Data/ycb-objects/ycb-tools/models/ycb/ -n 16 -o output_objects_log.txt -sc 1000 `

**For Google 512k**
`python ycb_prepare_objects.py --graspit_dir ./graspit_google_512k/ --models_file ycb_object_list_google_512k.txt --models_folder /mnt/Data/ycb-objects/ycb-tools/models/ycb/ -n 16 -o output_objects_log.txt -sc 1000 `





