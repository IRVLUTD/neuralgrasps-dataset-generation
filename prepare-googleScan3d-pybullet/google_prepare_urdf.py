import argparse
import os

parser = argparse.ArgumentParser(description='Prepare Google Scan-3D objects URDFs for Pybullet',
    usage="\
    Run this from OUTSIDE the docker environment! Since this requires the path to YCB\
    python google_prepare_urdf.py -f /PATH/TO/GOOGLE_DATA/ -m model_list.txt' -o ../rendering-test/google-urdfs/")

parser.add_argument('-m', '--models_file', type=str, default='model_list.txt', help="List of objects to convert")
parser.add_argument('-f', '--google_dir', type=str, default='/mnt/Data/googledata-initial/', help='Path to GoogleScan3d Models Directory. Example: "~/datasets/ycb-tools/models/ycb/"')
parser.add_argument('-o', '--output_dir', type=str, default='../rendering-test/google-urdfs/', help="Output directory for the urdf files")
# parser.add_argument('-s', '--scale', type=int, default='1000', help='Scale in mm. default is 1000 (i.e 1.0 scale in urdf mesh attribute)')
# parser.add_argument('-t', '--mesh_type', type=str, default='google_16k', help='Which mesh types desired (google__16k, 64k etc...')


def get_urdf_file(ycb_model: str, mesh_path: str) -> str:
    URDF_TEMPLATE = '''<?xml version='1.0' encoding='ASCII'?>
<robot name="{ycb_model_name}">
    <link name="baseLink">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{ycb_model_mesh_path}" scale="1.0 1.0 1.0"/>
            </geometry>
            <material name="texture">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{ycb_model_mesh_path}" scale="1.0 1.0 1.0"/>
            </geometry>
        </collision>   
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
        </inertial>
    </link>
</robot>'''
    return URDF_TEMPLATE.format(ycb_model_name=ycb_model, ycb_model_mesh_path=mesh_path)


def main(args):

    if args.models_file and not os.path.isfile(args.models_file):
        print(f"File does not exist: {args.models_file}")
        exit(0)

    if not args.google_dir:
        print(f"models list not specified")
        exit(0)

    if not os.path.isdir(args.google_dir):
        print(f"models directory (containing meshes): {args.ycb_dir} is incorrect")
        exit(0)

    if not args.output_dir or not os.path.isdir(args.output_dir):
        print(f"Output directory: {args.output_dir} for urdf files is incorrect")


    with open(args.models_file) as f:
        model_names = f.read().splitlines()

    SCALE = 1000 # HARD CODED -- in mm
    for model in model_names:
        mesh = os.path.join(args.google_dir, model, "meshes", 'model.obj')
        urdf_content = get_urdf_file(model, mesh)
        # sample fname: 021_bleach_cleanser.urdf
        urdf_fname = f"{model}.urdf"
        output_file = os.path.join(args.output_dir, urdf_fname)
        with open(output_file, "w") as outf:
            outf.write(urdf_content)


if __name__ == '__main__':
    main(parser.parse_args())
