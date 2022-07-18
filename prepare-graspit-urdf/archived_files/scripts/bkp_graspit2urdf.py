"""
Test Converts a graspit model to URDF
"""
from lxml import etree as ET
import os.path as osp
import numpy as np
from tf import transformations as trans
import sympy
import argparse
import math

def eval_exp(s):
  expr = sympy.sympify(s)
  var = list(expr.free_symbols)[0]
  coeff = float(expr.coeff(var))
  const = float(expr.subs(var, 0))
  return const, coeff

def T2xyzrpy(T):
  tx, ty, tz = T[:3, 3]
  R = np.eye(4)
  R[:3, :3] = T[:3, :3]
  rx, ry, rz = trans.euler_from_matrix(R)
  return tx, ty, tz, rx, ry, rz


def convert(input_filename, data_dir, output_dir):
  """
  :param input_filename: input GraspIt! XML file
  :param data_dir: Directory containing the STL meshes and data files for links (see README.md)
  :param output_dir: Directory where output URDF file is saved
  :return:
  """
  graspit_data_dir, robot_name = osp.split(input_filename)
  robot_name = robot_name.split('.')[0]
  graspit_data_dir = osp.join(graspit_data_dir, 'iv')
  output_filename = osp.join(output_dir, '{:s}.urdf'.format(robot_name))
  data_dir = osp.join(data_dir, robot_name)

  hand = ET.Element('robot', attrib={'name': robot_name})
  g_hand = ET.parse(input_filename).getroot()
  scale = 1e3
  link = ET.Element('link', attrib={'name': 'base_link'})
  joint = ET.Element('joint', attrib={'name': 'base_link_palm', 'type': 'fixed'})
  ET.SubElement(joint, 'origin', attrib={
    'rpy': '{:.8f} {:.8f} {:.8f}'.format(np.pi/2, 0, 0)})
  ET.SubElement(joint, 'parent', attrib={'link': 'base_link'})
  ET.SubElement(joint, 'child',  attrib={'link': 'palm'})
  hand.append(link)
  hand.append(joint)

  def convert_link(g_link):
    info_filename = g_link.text
    link_name = info_filename.split('.')[0]
    link_names = [link_name]
    link = ET.Element('link', attrib={'name': link_name})

    # visual
    v = ET.SubElement(link, 'visual')
    offset_filename = osp.join(data_dir,
        '{:s}_visual_offset.txt'.format(link_name))
    if osp.isfile(offset_filename):
      ox, oy, oz = np.loadtxt(offset_filename)
      ET.SubElement(v, 'origin',
          attrib={'xyz': '{:.8f} {:.8f} {:.8f}'.format(ox, oy, oz)})
    g = ET.SubElement(v, 'geometry')
    mesh_filename = osp.join(data_dir, '{:s}.stl'.format(link_name))
    mesh_filename = 'file://{:s}'.format(osp.abspath(mesh_filename))
    scale_str = (' '.join(['{:.8f}']*3)).format(1/scale, 1/scale, 1/scale)
    ET.SubElement(g, 'mesh',
        attrib={'filename': mesh_filename, 'scale': scale_str})
    
    # inertial
    gg_link = ET.parse(osp.join(graspit_data_dir, info_filename)).getroot()
    i = ET.SubElement(link, 'inertial')
    
    mass = float(gg_link.find('mass').text)
    mass = '{:.8f}'.format(mass / 1000.0)  # convert from grams to kg
    ET.SubElement(i, 'mass', attrib={'value': mass})
    
    inertial_filename = osp.join(data_dir, '{:s}.txt'.format(link_name))
    inertia_tensor = np.genfromtxt(inertial_filename, skip_footer=1)
    # http://gazebosim.org/tutorials?tut=inertia
    inertia_tensor /= (scale**5)
    if inertia_tensor[0] < 0:
      inertia_tensor *= -1
    inertia_comps = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
    ET.SubElement(i, 'inertia',
        attrib={k: '{:.4e}'.format(v) for k,v in zip(inertia_comps, inertia_tensor)})
    
    com = np.genfromtxt(inertial_filename, skip_header=1)
    com /= scale
    com = (' '.join(['{:.8f}']*3)).format(*com)
    ET.SubElement(i, 'origin', attrib={'xyz': com})

    hand.append(link)

    # add extra link if joint is Universal
    if g_link.get('dynamicJointType') == 'Universal':
      link_name = '{:s}_0'.format(link_name)
      ET.SubElement(hand, 'link', attrib={'name': link_name})
      link_names.insert(0, link_name)
    
    return link_names
    
  def add_dh_links_joints(dh_a, dh_d, dh_alpha, dh_theta, start_link_name,
        end_link_name):
    tx = dh_a / scale
    tz = dh_d / scale
    rx = dh_alpha
    rz = dh_theta

    if rz != 0:
      link_name = '{:s}_dh'.format(end_link_name)
      link = ET.Element('link', attrib={'name': link_name})
      hand.append(link)
      joint_name = '{:s}_{:s}'.format(start_link_name, link_name)
      joint = ET.Element('joint', attrib={'name': joint_name, 'type': 'fixed'})
      ET.SubElement(joint, 'origin', attrib={
        'rpy': '{:.8f} {:.8f} {:.8f}'.format(0, 0, rz)})
      ET.SubElement(joint, 'parent', attrib={'link': start_link_name})
      ET.SubElement(joint, 'child',  attrib={'link': link_name})
      hand.append(joint)
      start_link_name = link_name
    joint_name = '{:s}_{:s}'.format(start_link_name, end_link_name)
    joint = ET.Element('joint', attrib={'name': joint_name, 'type': 'fixed'})
    ET.SubElement(joint, 'origin', attrib={
      'rpy': '{:.8f} {:.8f} {:.8f}'.format(rx, 0, 0),
      'xyz': '{:.8f} {:.8f} {:.8f}'.format(tx, 0, tz)})
    ET.SubElement(joint, 'parent', attrib={'link': start_link_name})
    ET.SubElement(joint, 'child',  attrib={'link': end_link_name})
    hand.append(joint)

  # read DoF effort limits
  effort_limits = []
  for dof in g_hand.findall('dof'):
    effort_limit = float(dof.find('maxEffort').text) / 1e6
    effort_limits.append(effort_limit)

  # palm
  root_link_name = convert_link(g_hand.find('palm'))[0]

  chain_idx = 0 # not being used.
  dof_idx = 0
  tmp1 = len(list(g_hand.findall('chain')))
  for chain in g_hand.findall('chain'):

    link_names = [root_link_name]
    for link in chain.findall('link'):
      link_names.extend(convert_link(link))
    
    tmp = list(chain.findall('joint'))
    joint_idx = 1
    
    for joint in chain.findall('joint'):
      min_offset = 0
      max_offset = 0
      joint_min = str(np.deg2rad(float(joint.find('minValue').text)+min_offset))
      joint_max = str(np.deg2rad(float(joint.find('maxValue').text)+max_offset))
      tx = ty = tz = rx = ry = rz = ax = ay = az = 0
      if joint_idx == 1:
        init_T = chain.find('transform')
        tx, ty, tz = np.fromstring(init_T.find('translation').text,
          sep=' ') / scale
        rx = ry = rz = 0
        try:
          angle, axis = init_T.find('rotation').text.split(' ')
          angle = np.deg2rad(float(angle))
          if axis.upper() == 'X':
            rx += angle
          elif axis.upper() == 'Y':
            ry += angle
          elif axis.upper() == 'Z':
            rz += angle
          else:
            raise IOError('Wrong axis of rotation {:s}'.format(axis))
        except AttributeError:
          try:
            rotmat = np.fromstring(init_T.find('rotationMatrix').text, sep=' ')
            R = np.eye(4)
            R[:3, :3] = np.reshape(rotmat, (3, 3)).T
            _, _, _, rx, ry, rz = T2xyzrpy(R)
          except AttributeError:
            rx = ry = rz = 0

      dh_theta = joint.find('theta').text
      try:
        dh_theta = float(dh_theta)
      except ValueError:
        joint_type = 'revolute'
        dh_theta, az = eval_exp(dh_theta)
      dh_theta = np.deg2rad(dh_theta)

      dh_alpha = joint.find('alpha').text
      try:
        dh_alpha = float(dh_alpha)
      except ValueError:
        joint_type = 'revolute'
        dh_alpha, ax = eval_exp(dh_alpha)
      dh_alpha = np.deg2rad(dh_alpha)

      dh_d = joint.find('d').text
      try:
        dh_d = float(dh_d)
      except ValueError:
        joint_type = 'prismatic'
        dh_d, az = eval_exp(dh_d)

      dh_a = joint.find('a').text
      try:
        dh_a = float(dh_a)
      except ValueError:
        joint_type = 'prismatic'
        dh_a, ax = eval_exp(dh_a)
      # if ax+ay+az != 1:
      if ax+ay+az > 1:
        raise ValueError('1 and only 1 variable allowed per frame')

      link_name = '{:s}_joint'.format(link_names[joint_idx])
      link = ET.Element('link', attrib={'name': link_name})
      hand.append(link)
      joint_name = '{:s}_{:s}'.format(link_names[joint_idx-1], link_name)
      joint = ET.Element('joint', attrib={'name': joint_name, 'type': joint_type})
      ET.SubElement(joint, 'origin', attrib={
        'rpy': '{:.8f} {:.8f} {:.8f}'.format(rx, ry, rz),
        'xyz': '{:.8f} {:.8f} {:.8f}'.format(tx, ty, tz)})
      ET.SubElement(joint, 'parent', attrib={'link': link_names[joint_idx-1]})
      ET.SubElement(joint, 'child',  attrib={'link': link_name})
      ET.SubElement(joint, 'axis',
          attrib={'xyz' : '{:.8f} {:.8f} {:.8f}'.format(ax, ay, az)})
      max_effort = effort_limits[dof_idx]
      if joint_type == 'revolute':
        max_effort /= 1e3  # convert from N-mm to N-m
      max_velocity = 30 * 3.14 / 180
      ET.SubElement(joint, 'limit', attrib={
        'lower': joint_min, 'upper': joint_max,
        'effort': '{:.8f}'.format(max_effort),
        'velocity': '{:.8f}'.format(max_velocity)})
      hand.append(joint)
      
      # add dummy links and fixed joints for DH parameters
      add_dh_links_joints(dh_a, dh_d, dh_alpha, dh_theta, link_name,
          link_names[joint_idx])

      joint_idx += 1
      dof_idx += 1

  tree = ET.ElementTree(hand)
  tree.write(output_filename, pretty_print=True, xml_declaration=True)
  print('Written to {:s}'.format(output_filename))


if __name__ == '__main__':
  graspit_dir = osp.expanduser(osp.join('/', 'opt', 'graspit', 'models', 'robots',
    'Barrett'))
    # 'Barrett'))
    # 'fetch_gripper'))
    # 'HumanHand'))
  parser = argparse.ArgumentParser()
  parser.add_argument('--input_filename', default=osp.join(graspit_dir,
    'Barrett.xml'))
    # 'Barrett.xml'))
    # 'fetch_gripper.xml'))
    # 'HumanHand20DOF.xml'))
  parser.add_argument('--data_dir', default=osp.join('./', 'data'))
  parser.add_argument('--output_dir', default=osp.join('./', 'urdf'))
  args = parser.parse_args()
  print(args.input_filename)
  convert(args.input_filename, args.data_dir, args.output_dir)
