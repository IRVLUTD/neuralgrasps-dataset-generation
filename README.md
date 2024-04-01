# MultiGripperGrasp Graspit Dataset

This repository hosts the code and utilities associated with pipeline for generating
the graspit-based dataset of grasps the paper
[MultiGripperGrasp: A Dataset for Robotic Grasping from Parallel Jaw Grippers to Dexterous Hands](https://irvlutd.github.io/MultiGripperGrasp/). The graspit-based grasp generation code is based upon the code from
the
[NeuralGrasps: Learning Implicit Representations for Grasps of Multiple Robotic Hands](https://irvlutd.github.io/NeuralGrasps/).
paper.


# Grasp Generation
Graspit grasp generation is done on a python-2 environment on a Ubunut 18.04 docker image.

## Initial Setup (Docker container) for GraspIt Grasp Generation
I am sharing the [container image](https://utdallas.box.com/s/z515hoc7qe6am4jml05wk9zw6fowdb3u) as a `[CONTAINER-NAME].tar.gz` file -- (need to extract it first) via ut-dallas box. Example container name = `img-grasp-pyb.tar.gz`.

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

- You can mount an external folder where the code is synced to the `~/data/` inside the
  docker env for convinience!

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
a fixed number of grasps to `~/data/output_dataset/refined_grasps` as 
`refined-*.json` files. Note: we do not modify the grasps (or *refine* them)!

The command for `refine_grasps.py` will be:

```
python refine_grasps.py \
--path_out ~/data/output_dataset/ \
--num_samples 50 \
--gripper fetch_gripper
```

# Dataset Generation

## Preparing models for Pybullet

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
  generation process.
- `-t` is the mesh type. No need to change it.

## GoogleScanned3D Objects

- Follow similar steps for the google models. See `prepare-googleScan3d-*` folders!

# Citation

Please consider citing the paper if it helps in your work.

MultiGripperGrasps:
```bibtex
@misc{murrilo2024multigrippergrasp,
      title={MultiGripperGrasp: A Dataset for Robotic Grasping from Parallel Jaw Grippers to Dexterous Hands}, 
      author={Luis Felipe Casas Murrilo and Ninad Khargonkar and Balakrishnan Prabhakaran and Yu Xiang},
      year={2024},
      eprint={2403.09841},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
}
```

NeuralGrasps:
```bibtex
@inproceedings{khargonkar2023neuralgrasps,
  title={Neuralgrasps: Learning implicit representations for grasps of multiple robotic hands},
  author={Khargonkar, Ninad and Song, Neil and Xu, Zesheng and Prabhakaran, Balakrishnan and Xiang, Yu},
  booktitle={Conference on Robot Learning},
  pages={516--526},
  year={2023},
  organization={PMLR}
}
```

# Acknowledgements
Thanks to the authors of the following repositories for open-sourcing their code and from 
which major parts of our codebase were inspired from:

- [ObMan Paper](https://www.di.ens.fr/willow/research/obman/data/)
- [Github: GraspIt2URDF](https://github.com/samarth-robo/GraspIt2URDF)
- [Github: mesh_to_sdf](https://github.com/marian42/mesh_to_sdf)

