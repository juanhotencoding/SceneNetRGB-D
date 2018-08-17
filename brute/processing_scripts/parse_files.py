from collections import namedtuple
import numpy as np
import pathlib
import re
import sys

# functions for parsing render_info.log
def get_info_log_lines(info_log):
    with open(str(info_log),'r') as f:
        readlines = f.readlines()
    return readlines

def get_instances(info_lines):
    instances = []
    for line in info_lines:
        if line.startswith('time:'):
            break
        if line.startswith('instance:'):
            instance_dict = {}
            try:
                instance_num,wnid,english,shapenethash = line.strip().split(';')
            except:
                try:
                    instance_num,wnid,english,position,radius,power = line.strip().split(';')
                except:
                    instance_num,wnid,english,position,v1,v2,power = line.strip().split(';')
                shapenethash = None
            instance_num = instance_num.split(':')[1]
            wnid = wnid.split(',')[0]
            english = english.split(',')[0]
            if shapenethash == '':
                shapenethash = None
            instance_dict['instance_num'] = int(instance_num)
            instance_dict['wnid'] = wnid
            instance_dict['english'] = english
            instance_dict['hash'] = shapenethash
            instances.append(instance_dict)
    return instances


# functions for parsing scene_and_trajectory.txt
# functions for parsing scene_and_description.txt
def get_text_layout_lines(text_layout_file):
    with open(text_layout_file,'r') as f:
        readlines = f.readlines()
    return readlines

def process_objects_into_instances(layout_lines):
    objects = []
    current_object = None
    next_line_type = None
    for line in layout_lines:
        if line.startswith('#first'):
            break
        if line.strip() == 'object':
            next_line_type = 'object'
            continue
        if line.strip() == 'wnid':
            next_line_type = 'wnid'
            continue
        if line.strip() == 'scale':
            next_line_type = 'scale'
            continue
        if line.strip() == 'transformation':
            next_line_type = 'trans'
            continue
        if next_line_type == 'object':
            if current_object is not None:
                objects.append(current_object)
            current_object = {}
            num_transformations = 0
            current_object['hash'] = line.rstrip()
            continue
        if next_line_type == 'wnid':
            current_object['wnid'] = line.rstrip()
            continue
        if next_line_type == 'scale':
            current_object['scale'] = float(line.rstrip())
            continue
        if next_line_type == 'trans':
            if num_transformations == 0:
                current_object['transformation'] = []
            if num_transformations < 3:
                current_object['transformation'].append([float(x) for x in line.rstrip().split()])
            num_transformations += 1
            continue
    if current_object is not None:
        objects.append(current_object)
    return objects



# make an object information lookup json file using the information found in the textfiles directory
def make_object_lookup(text_file):
    file = get_info_log_lines(text_file)

    lookup = {}
    
    # start after header
    for info in file[1:]:
        items = info.split(' ')
        shapenet_dir = items[0]
        obj_ids = items[1].split(',')
        names = items[2].split(',')
        
        lookup[shapenet_dir] = {}
        
        for obj_id in obj_ids:
            lookup[shapenet_dir].update({obj_id:names})
            