#!/usr/bin/env python2

import argparse
import json
import os
import time
import subprocess
import re
import sys

parser = argparse.ArgumentParser(description='Prepare objects for GraspIt',
    usage="python -m mano_grasp.prepare_objects -f /PATH/TO/YCB/ -o ycb_objects.txt -sc 1000 -n 8 --strip_regex '/google_512k/(non)?textured' ")
parser.add_argument('-l', '--models_file', type=str, default='', help="list of objects to convert")
parser.add_argument('-f', '--models_folder', type=str, default='', help="base folder containing models")
parser.add_argument('-n', '--n_jobs', type=int, default=1, help="number of parallel jobs to run")
parser.add_argument('-o', '--file_out', type=str, default='', help="list of prepared object versions")
parser.add_argument('-v', '--verbose', action='store_true')
parser.add_argument('-d', '--debug', action='store_true')
parser.add_argument('--graspit_dir',
                    type=str,
                    default=os.environ['GRASPIT'],
                    help="Path to GraspIt root directory")
parser.add_argument('-sc', '--scales', nargs='*', default=[], help="scales to apply to each model")
parser.add_argument('-re', '--strip_regex', type=str, default='', help="regex to strip away unwanted parts from final object name")


class ObjectConverter:

    SCALE_FILTER_TEMPLATE = '''<!DOCTYPE FilterScript>
<FilterScript>
 <filter name="Transform: Scale">
  <Param type="RichDynamicFloat" value="{scale}" min="0.1" name="axisX" max="{scale}"/>
  <Param type="RichDynamicFloat" value="{scale}" min="0.1" name="axisY" max="{scale}"/>
  <Param type="RichDynamicFloat" value="{scale}" min="0.1" name="axisZ" max="{scale}"/>
  <Param type="RichBool" value="true" name="uniformFlag"/>
  <Param enum_val0="origin" enum_val1="barycenter" enum_cardinality="3" enum_val2="custom point" type="RichEnum" value="0" name="scaleCenter"/>
  <Param x="0" y="0" z="0" type="RichPoint3f" name="customCenter"/>
  <Param type="RichBool" value="false" name="unitFlag"/>
  <Param type="RichBool" value="true" name="Freeze"/>
  <Param type="RichBool" value="false" name="ToAll"/>
 </filter>
</FilterScript>'''
    GRASPIT_OBJECT_XML_TEMPLATE = '<root><geometryFile type="off">{}.off</geometryFile></root>'
    VALID_INPUT_FORMATS = ['.obj', '.stl', '.off', '.ply']

    """ GraspIt object converter """

    def __init__(self, output_dir, basepath='', scripts={'default':''}, strip_regex=''):
        """Constructor

        Arguments:
            output_dir {str} -- graspit objects directory

        Keyword Arguments:
            basepath {str} -- path to strip from object filename at name conversion (default: {''})
            scripts {dict} -- named meshlab scripts to apply on each object
            strip_regex {str} -- regex to strip for each object to arrive at true name
        """
        self._output_dir = output_dir
        self._basepath = basepath
        self._scripts = scripts
        if not self._scripts.keys():
            self._scripts['default'] = ''
        self._strip_regex = strip_regex

    def __call__(self, object_file):
        """Prepare object for Graspit

        Arguments:
            object_file {str} -- object

        Returns:
            tuple -- object_file, object_names
        """
        devnull = open(os.devnull, 'wb')
        object_names = []

        object_basename, object_ext = os.path.splitext(object_file)
        object_basename = object_basename.replace(self._basepath, '')
        if self._strip_regex:
            object_basename = re.sub(self._strip_regex, '', object_basename)
        object_basename = object_basename.replace('/', '_')

        if object_ext in ObjectConverter.VALID_INPUT_FORMATS:
            for script_name, script_file in self._scripts.items():
                obj_name = object_basename + '_' + script_name
                print('converting ', obj_name)
                off_path = os.path.join(self._output_dir, obj_name+'.off')
                xml_path = os.path.join(self._output_dir, obj_name+'.xml')
                meshlab_call = ['meshlabserver', '-i', object_file, '-o', off_path]
                if script_file != '':
                    meshlab_call.extend(['-s', script_file])
                try:
                    proc = subprocess.check_output(meshlab_call, stderr=subprocess.STDOUT)
                    with open(xml_path, 'w') as xml_file:
                        xml_file.write(ObjectConverter.GRASPIT_OBJECT_XML_TEMPLATE.format(obj_name))
                    object_names.append(obj_name)
                except Exception as ex:
                    print(ex)
        else:
            raise NotImplementedError('')
        return (object_file, object_names)


def main(args):
    if not os.path.isdir(args.graspit_dir):
        print('Wrong GraspIt path: "{}"'.format(args.graspit_dir))
        exit(0)

    if args.models_file and not os.path.isfile(args.models_file) and not args.models_folder:
        print('File not exists: "{}"'.format(args.models_file))
        exit(0)

    if not args.file_out:
        print('Output file not specified')
        exit(0)

    temp_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.temp')
    if not os.path.isdir(temp_dir):
        os.makedirs(temp_dir)

    models = []
    args.models_folder = os.path.abspath(args.models_folder) + os.path.sep
    if not args.models_file and args.models_folder:
        for root, dirnames, filenames in os.walk(args.models_folder):
            for filename in [f for f in filenames if f[-4:] in ObjectConverter.VALID_INPUT_FORMATS]:
                models.append(os.path.join(root, filename))
        if len(models) > 0:
            temp_models_file = os.path.join(temp_dir, 'objects_{}.txt'.format(time.strftime("%b_%d_%Y_%H_%M_%S", time.localtime())))
            with open(temp_models_file, 'w') as f:
                f.write('\n'.join(models))
                
            print('No models_file provided but found {} models in {}'.format(len(models), args.models_folder))
            print('Have a look at {} to see the list of objects; you can also edit it'.format(temp_models_file))
            use_response = ''
            while use_response not in ['y', 'n']:
                if sys.version_info < (3, 0, 0):
                    use_response = raw_input('Do you want to use this object list for conversion (y/n): ')
                else:
                    use_response = input('Do you want to use this object list for conversion (y/n): ')
                if use_response == 'n':
                    os.remove(temp_models_file)
                    exit(0)
                elif use_response == 'y':
                    args.models_file = temp_models_file

    if args.models_file:
        with open(args.models_file) as f:
            models = f.read().splitlines()

    scripts = {}
    for s in args.scales:
        script = ObjectConverter.SCALE_FILTER_TEMPLATE.format(scale=s)
        script_name = 'scale_{}'.format(s)
        script_file = os.path.join(temp_dir, 'meshlab_{}_filter.mlx'.format(script_name))
        with open(script_file, 'w') as f:
            f.write(script)
        scripts[script_name] = script_file

    converter = ObjectConverter(output_dir=os.path.join(args.graspit_dir, 'models/objects'),
                                basepath=args.models_folder,
                                scripts=scripts,
                                strip_regex=args.strip_regex)

    if args.n_jobs > 1:
        from joblib import Parallel, delayed
        objects = Parallel(n_jobs=args.n_jobs, verbose=50)(delayed(converter)(m) for m in models)
    else:
        objects = [converter(body) for body in models]

    prepared_objects = []
    for object_file, object_names in objects:
        prepared_objects.extend(object_names)
        with open(args.file_out, 'w') as f:
            f.write('\n'.join(prepared_objects))


if __name__ == '__main__':
    main(parser.parse_args())
