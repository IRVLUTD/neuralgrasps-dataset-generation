# NeuralGrasps Dataset

This repository hosts the code and utilities associated with pipeline for generating
the training dataset of grasps, point clouds, contact maps and SDF values used in the paper
[NeuralGrasps: Learning Implicit Representations for Grasps of Multiple Robotic Hands](https://irvlutd.github.io/NeuralGrasps/).

It includes things like:

- graspit interface python code
- processing of the grasp information (xml files) to 3d models
- rendering of the 3d models and obtaining the point clouds from a given camera viewpoint.
- saving all results to an output directory.

Graspit grasp generation is done on a python-2 environment on a Ubunut 18.04 docker image.
All the other steps make use of a python-3 env (see `rendering-test/environment.yml`)

# Grasp Generation

## Initial Setup (Docker container) for GraspIt Grasp Generation
I am sharing the container image (`[CONTAINER-NAME].tar.gz` -- need to extract it first) 
on ut-dallas box. Example container name = `img-grasp-pyb.tar.gz`

Extract the `img-grasp-pyb.tar.gz` file -- this will give you a .tar archive.
You can load this as docker image using: `sudo docker load --input img-grasp-pyb.tar`.
The docker image will now be loaded as `img-grasp-pyb`

To generate grasps using Graspit, we require support for GUI mode:

Run `xhost +` (required)

Start the docker container as follows (supports GUI mode for graspit):

```
sudo nvidia-docker run -it --name CONTAINER_NAME --privileged --gpus all \
-e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
--device=/dev/dri:/dev/dri -v PATH_TO_DOCKER_DATA_FOLDER:/root/data/ IMAGE_NAME`
```

- `PATH_TO_DOCKER_DATA_FOLDER` is where you have extracted the `docker_data` folder or it
  can simply be a symlink to an appropriate folder which will be mounted on the docker
  container at the `/root/data/` location. 

- `IMAGE_NAME`: docker image name (for us it's `img-grasp-pyb`)

- `CONTAINER_NAME` : a descriptive name for the container

- This will open an interactive bash session. tmux is already installed, so you can open 
  multiple terminals. Note that the tmux "prefix" is `Ctrl-b` but I have changed it to 
  `Ctrl-a`. So for example, if you need to create new window in a session, it is: 
  `Ctrl-a c`. Using tmux is highly recommended as we need to have multiple terminals!

- Additional notes (for future reference):
  - Graspit Directory: `/opt/graspit/`
  - The ycb model xml files are already added the graspit directory.
  - For ycb models to graspit, see `prepare-ycb-graspit/README.md`


## Grasp Generation Script
Python 2 env -- default one on the container (using this one). Make sure that you have 
mounted the `docker_data` dir on host (or any other host dir of  your choice)  as 
`/root/data/` on the container. You can check this from inside the docker containter: 
`ls ~/data/` -- should list the files from the host folder.

- Run `roscore` in terminal
- In a separate terminal, run:
  - `source ~/graspit_ros_ws/devel/setup.bash`
  - Inside the *same* terminal where you sourced `graspit_ros_ws`, 
    run `generate_grasps.py` with appropriate flags. 
    
This will output the initial set of grasps to `~/data/output_dataset/initial_grasps/` 
folder in the docker container. Note that since `~/data/` is actually a host mounted dir,
you can also see the changes via file explorer on the host. Once the grasps are generated,
we do not need the docker container. Also note that the `output_dataset`
is referred to as the dataset directory in this readme.

Example:

```
python generate_grasps.py -e \
--path_out /root/data/output_dataset/initial_grasps/ \
--n_jobs 16 --max_steps 100000 \
-l ycb_models_file.txt --num_grasps 300 \
--gripper fetch_gripper`
```

- `-e` makes the process headless (non-gui) making it much faster to run in practice.
- `-l` option specifies the file which shows ycb models will be used.
- Options for `--gripper` (case sensitive):
   - `fetch_gripper`
   - `Barrett`
   - `HumanHand`
   - `panda_grasp`
   - `Allegro`
- We might need to tune the `--num_grasps` and `--gripper` flags depending on how the 
  process goes. Can remove `-e` flag to debug the vizualization.

## Grasp Refinement
Run refinement code `refine_grasps.py` which will do farthest point sampling and output 
a fixed number of grasps (here 50) to `~/data/output_dataset/refined_grasps` as 
`refined-*.json` files. Note: we do not modify the grasps (or *refine* them)!

The command for `refine_grasps.py` will be:

```
python refine_grasps.py \
--path_out ~/data/output_dataset/ \
--num_samples 50 \
--gripper fetch_gripper
```

### Visualizing the Refined Grasps

- If you have followed the docker commands, then GUI mode of applications is supported.
- From a tmux terminal:
  - Activate the `pybullet` env: `conda activate pybullet`
  - Launch jupyter notebook: `jupyter notebook --allow-root`
  - It should
- Check out the `rendering_test/test_grasp_viz-gripper.ipynb` notebook:
  - The pybullet connnect GUI command should launch the pybullet gui window. IF it does
    not launch, you can also do this viz *outside* the docker env by simply making a new
    `pybullet` on your host env via the `rendering_test/environment.yml` file.
    - Make sure to change the `dataset_dir` (since now you are accessing from the host).
  - It assumes that the refined grasps are stored in `output_dataset/refined_grasps/`
  - Change the `object_model_name` as needed
  - Change the `gripper_name` as needed


# Dataset Generation

## Preparing YCB models for Pybullet

Use the download script in [ycb-tools](https://github.com/sea-bass/ycb-tools) repo to
download the dataset. We are using the google-16k scans. Download (clone) the `ycb-tools`
in some appropriate location like `~/external_datasets/ycb_dataset/`

Objects will be downloaded to `ycb-tools/models/ycb/`

Then check out the `prepare-ycb-pybullet` folder. Steps copied below:

**Note:** Use these generated urdf files to do the refined grasp vizualization

### Generating the URDFs

Need to do this from outside the docker environment since the ycb data is not present
inside the docker-data folder.

Simply go through the script `ycb_prepare_urdf.py` with appropriate command line flags:

You will only need to pass the `-f` (`--ycb_dir`) flag to point to the location
of ycb models. For example it can then be:
`python ycb_prepare_urdf.py -f /mnt/Data/ycb-objects/ycb-tools/models/ycb/`

The `/mnt/Data/ycb-objects/ycb-tools/models/ycb/` contains the ycb object folders like
`001_chips_can/`, `003_cracker_box/` etc..

#### Default Params
- `-m` (`--models_file`) is already the one that is default specified
- `-o` can be anywhere on your system so long as you remember it for the point cloud
  generation process. Or to make matters easy, copy the generate urdfs to 
  `../rendering-test/ycb-urdfs/`
- `-t` is the mesh type. No need to change it.


## Point Cloud Rendering, SDF and Contact Maps
For each of the steps below, you can do a `--help` with the python script to know more
about the command line arguments required. Most of the defaults are chosen carefully so
they should work as is and they are straight forward to understand.

Point Cloud Generation via pybullet:
See `rendering-test/grasp_pc.py` for generating point clouds of each grasp.

Ground Truth SDF Sampling from point clouds:
See `rendering-test/gen_sdf.py` for generating the sdf samples (training data)

Contact Maps:
See `rendering-test/gen_obj_contact_map.py` for generating the contact map of 
gripper-object grasping scene (across all the grasps for a gripper-object pair).

Visualizing the Grasps as RGB Images: There are two files to generate rgb images of each 
grasping scene. 
- `rendering-test/gen_images.py`
- `rendering-test/gen_high_res_images.py`

You can tweak the params of either to get high or low-res images in order to quickly 
visualize how the grasps look like. Example run:

```
python gen_high_res_images.py \
-l ../grasp-generation/ycb_models_file_bkp.txt \
-o ../../../dataset_train \
--gripper Barrett
```

Assumes that the dataset dir (e.g. `dataset_train`) is present one level above the 
repository root (OR at least its symlink is present).

## Dataset Folder Structure

Here `object_003` is an example name. Each object will have similar sub-folders as shown.
Each of the above scripts will populate the sub-folders on execution. The 
`object_point_cloud.ply` can be copied from the corresponding ycb directory.

```
|---initial_grasps/*.json
|---refined_grasps/*.json
|---object_003/
    |---point_cloud/
        |---Allegro/
        |---Barrett/
        |---HumanHand/
        |---fetch_gripper/
        |---panda_grasp/
    |---sdf/
    |---contactmap/
    |---images/
    |---object_point_cloud.ply
    |---norm_params.npz
```


# Citation

Please consider citing the paper if it helps in your work.

```bibtex
@misc{https://doi.org/10.48550/arxiv.2207.02959,
      doi = {10.48550/ARXIV.2207.02959},
      url = {https://arxiv.org/abs/2207.02959},
      author = {Khargonkar, Ninad and Song, Neil and Xu, Zesheng and Prabhakaran, Balakrishnan and Xiang, Yu},
      title = {NeuralGrasps: Learning Implicit Representations for Grasps of Multiple Robotic Hands},       
      publisher = {arXiv},
      year = {2022},          
      copyright = {Creative Commons Attribution 4.0 International}
      }
```

# Acknowledgements
Thanks to the authors of the following repositories for open-sourcing their code and from 
which major parts of our codebase were inspired from:

- [ObMan Paper](https://www.di.ens.fr/willow/research/obman/data/)
- [Github: GraspIt2URDF](https://github.com/samarth-robo/GraspIt2URDF)
- [Github: mesh_to_sdf](https://github.com/marian42/mesh_to_sdf)

