# Prepare Google 3D Scanned objects for Graspit

The processing code is borrowed from `prepare-ycb-graspit` portion of dataset generation
code for NeuralGrasps paper.

## Google objects: initial debug list

Saved the list of objects in `object_list.txt` in `/mnt/Data/googledata-initial/`
- BIA_Porcelain_Ramekin_With_Glazed_Rim_35_45_oz_cup.zip
- Chef_Style_Round_Cake_Pan_9_inch_pan.zip
- Cole_Hardware_Hammer_Black.zip
- Crosley_Alarm_Clock_Vintage_Metal.zip
- Ecoforms_Cup_B4_SAN.zip
- Marc_Anthony_Strictly_Curls_Curl_Envy_Perfect_Curl_Cream_6_fl_oz_bottle.zip
- Pony_C_Clamp_1440.zip
- Shurtape_Gaffers_Tape_Silver_2_x_60_yd.zip


## Meshlab .mlx script

Make sure you have meshlab installed: `sudo apt install meshlab`. The given
script is for Ubuntu 20.04 version of meshlab 
(apt installed version: MeshLab_64bit_fp v2020.03+dfsg1). Depending on your
Meshlab version, the script encoded in the python file may need to be changed.

To obtain the meshlab script for your version is quite easy. Load any model and
- GUI -> Filters -> Normals.Curvature,Orientation -> Transform: Scale, Normalize
- Appply the basic filter and then GUI -> Filters -> View filter script -- save it!

This gives us the correct syntax for the script. Copy the script to the
`SCALE_FILTER_TEMPLATE` variable in `google_prepare_objects.py` file. 

## Python Script Args

Use this script **OUTSIDE** the docker container! (pybullet env)

Use `python google_prepare_objects.py --help` to see all available options.

Make sure to read the command line args:

- `--models_file` : See `./object_ids.txt` for an example. 

- `--models_folder` : Its in `/mnt/Data/ycb-objects/ycb-tools/models/ycb/`
  essentially, the folder containing the 001, 002, 003 ... object folders

- `--output_dir` : Give a dummy location like `/mnt/Data/google_graspit/`. 

  Check the dir first if the files are there not!
  
  `cp output_folder/* /opt/graspit/models/objects/`


- `-sc` or `--scales` : Set it to 1000 (Usually you want to apply some scaling to the objects to fit the hand)
- `-n` : Set to 16 (number of jobs) 
- `-o` : output log file -- which objects were converted?

### Sample calls:

Replace `--models_folder` by path to where you downloaded and extracted the Google 
Scanned Objects Dataset.

`python google_prepare_objects.py --output_dir /mnt/Data/google_graspit_initial --models_file object_ids.txt --models_folder /mnt/Data/googledata-initial/ -n 16 -o output_objects.logfile -sc 1000`

#### Doing the conversion on Entire Google dataset
`python google_prepare_objects.py --output_dir /mnt/Data/google_graspit_all_scale1000 --models_file object_ids.txt --models_folder /mnt/Data/google-scanned-objects-all/ -n 16 -o output_objects.logfile -sc 1000`

