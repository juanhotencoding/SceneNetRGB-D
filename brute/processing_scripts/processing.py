# Utilities
import os
import json
import pprint as pprint
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
from skimage import io

# cocoapi for encode instance image into rle byte string
import cocoapi.PythonAPI.pycocotools.mask as mask

# parsing functions
import parse_files

# location of dataset
scene_dir = "../scenes/"


# store the rgb, instance images and files
rgb_images = []
instance_images = []
logs = []


# recurse through the scenes directory
for subdir, dirs, files in os.walk(scene_dir):
    for file in files:
        
        # store the file path
        file_path = os.path.join(subdir, file)
        
        # store the rgb image
        if 'photo' in file_path:
            rgb_images.append(file_path)
        
        # store the instance image
        if 'instance' in file_path:
            instance_images.append(file_path)
            
        # store log files
        if 'render' in file_path or 'scene_and_trajectory' in file_path:
            logs.append(file_path)
            
            
# pair up the files to their respective scene
paired_logs = list(zip(logs[0::2], logs[1::2]))


# create a list to hold the paired files to the respective scene number
scenes = [[log] for log in paired_logs]



# utility functions
get_frame_num = lambda x: x.split('/')[-1].split('_')[0]
get_scene_num = lambda x: x.split('/')[-3]


# match up the rgb and instance frame numbers with respect to the scene number
paired_images = [(rgb, instance) for rgb in rgb_images for instance in instance_images
                if get_frame_num(rgb) == get_frame_num(instance) and get_scene_num(rgb) == get_scene_num(instance)]


# add the paired iamges to the correct scene number in the scenes list
for scene in scenes:
    imgs = []
    for img in paired_images:
        if scene[0][0].split('/')[-2] == get_scene_num(img[0]):
            imgs.append(img)
        scene.append(imgs)


# make the name lookup json if it doesn't already exists
# name lookup is for looking up the name of the object instance and getting its wnid

# text_file = 'src/scene_generator/textfiles/train_split_filtered_model_info_and_texture.txt'

# if os.path.exists("shapenet_dir_and_object_ids.json"):
#     print("name_lookup already exists")
# else:
#     lookup = parse_files.make_object_lookup(text_file)
#     with open("shapenet_dir_and_object_ids.json", "w") as write_file:
#         json.dump(lookup, write_file)
with open("shapenet_dir_and_object_ids.json", "r") as read_file:
    name_lookup = json.load(read_file)


# template data object to be used in making json file

data = {
    'info': {
        'description': "Autonomous Robotics and Perception Group Synthetic Images Dataset(ARPG-SID). ",
        'url': None,
        'version': 1,
        'year': 2018,
        'contributor': "Juan Vargas-Murillo",
        'date_created': None,
        
    },
    'images':[ ],
    'licenses':[
        {
            'url': "https://creativecommons.org/licenses/",
            'id': 0,
            'name': "creative commons",
        }
    ],
    'annotations':[ ],
    'categories':[ ],
}


# These will contain the frame numbers and images
# that we can actually use
scene_number_frames = []
scene_number_images = []



for ii, scene in enumerate(scenes):
#     print(f"{ii}")

    # get the render_info.log and scene_and_trajectory_description.txt file for the current scene
    rl = scene[0][1] if "render" in scene[0][1] else scene[0][0]
    df = scene[0][0] if "description" in scene[0][0] else scene[0][1]
#     print(rl, df)
    
    # get the object instance information from the description file
    ll = parse_files.get_text_layout_lines(df)
    po = parse_files.process_objects_into_instances(ll)
#     print(po)

    # get the object instance information from the render log
    gl = parse_files.get_info_log_lines(rl)
    oi = parse_files.get_instances(gl)
#     print(oi)

    # loop through the object information from the description file using the 'hash' value to match
    # with the object information from the render log so that we can update the 'wnid' and 'english'
    # fields for the object instance.
    # we are updating the missing information in the render log so that we can use it later when
    # we make the json file
    for p in po:
#         print(p)
        for o in oi:
#             print(o)
            # objects with a 'hash' of None are default objects that are not sampled from the 
            # the ShapeNet database
            if o['hash'] is not None and o['hash'] == p['hash']:
#                 print(o)
                # update the empty 'wnid' field in the render log with the 'wnid' found in the description file
                o['wnid'] = p['wnid']
        
                # update the empty 'name' field in the render log with the name found in the name_lookup file
                o['english'] = name_lookup[o['hash']][o['wnid']][0]
                
#     print(oi)
    # store the scene number to match images with
    sn = rl.split('/')[-2]
#     print(sn)

    # store the frames and images that pass the filtering process
    vf = []
    vi = []
    
    # make a list that contains the frame numbers and images for the respective scene
    scene_number_frames.append([sn])
    scene_number_images.append([sn])
    
#     print(scene_number_images)

    # store the number of objects in a given image/frame so that we can use it during filtering
    # against the average amount of objects for a given scene.
    num_objs = []
    
    
    # loop through the images getting the number of objects in each image and adding it to the overall
    # number of objects in the scene.
    for img in scene[1]:
#         print(img)
        ini = img[1]
#         print(ii)
        iimg = io.imread(ini)
#         print(iimg)
        obj_ids = np.unique(iimg)
#         print(len(obj_ids), obj_ids)
        num_objs.append(len(obj_ids)-1)
#         print(num_objs)
#         plt.imshow(iimg==71)
#         break

    # get the min, max, and average number of objects for the given scene
    min_objs = min(num_objs)
    max_objs = max(num_objs)
    
    avg_objs = np.ceil((min_objs + max_objs) / 2)
#     pprint.pprint(f"Using object info: \n{oi}")

    # loop through the images now that we have the threshold for the number of objects
    # we want to be visible for a given scene we can find the frames that we can actually use
    for img in scene[1]:
        ini = img[1]
#         print(ii)
        fn = get_frame_num(ini)
#         print(fn)
        iimg = io.imread(ini)
#         print(f"Objects in image: {ini}")
        objs_ids = np.unique(iimg)
#         print(f"{objs_ids}")
        
        # for the updated information of each object instance in the render log
        # we check if the object instance in the render log is actually a part of this image/frame
        # we also check if it is not a default scene object and finally we check if the image/frame
        # has more objects than the average amount of objects for the given scene, if so we append the
        # frame number to the usable frames list
        for o in oi:
#             print(o)
            if o['instance_num'] in objs_ids and o['hash'] is not None and len(objs_ids) > avg_objs:
#                 print(f"Using {o}")
                vf.append(fn)
        
    vf = np.unique(vf)
#     print(len(vf))

    scene_number_frames[ii].append(vf)
#     print(scene_number_frames)
    
    
    # now that we have the frame numbers for the images we can use we can start to make the data object
    # that will be made into a json object
    for img in scene[1]:
        # get the rgb and instane image file path
        rgb, inimg = img[0], img[1]
#         print(rgb, inimg)
        
        # get the scene number and frame number of the image we are working with 
        sn, fn = get_scene_num(rgb).split('_')[1], get_frame_num(rgb)
        
#         print(sn, fn)
#         print(len(scene_number_frames[0][1]))

        # if the image frame number is in the usable frame numbers than we add an entry into the
        # data object
        if fn in scene_number_frames[ii][1]:
#             print(f"Using scene image frame {fn}")

            # the scene number and frame number (scene 6 frame number 375 => 060375)
            img_id = "0"+sn+"0"+fn

            # get the object instances for the given image
            instance_img = io.imread(inimg)    
            objs_in_img = np.unique(instance_img)

#             print(objs_in_img)
            
    
            # add the current usable image file path to the data object
            data['images'].append({
                'license': data['licenses'][0]['id'],
                'file_name': rgb,
                'coco_url': None,
                'height': 240,
                'width': 320,
                'date_captured': None,
                'flickr_url': None,
                'id': img_id,
            })
            
            
            # for the current scene and image use the updated render log information for each object
            # instance and loop through the information for each object instance checking if the object 
            # is in the current image and if it is not a default object
            for o in oi:
                if o['instance_num'] in objs_in_img and o['hash'] is not None:
#                     print(f"{o['instance_num']} in {objs_in_img} for frame {fn}")
                    
    
                    # add the object instance id to the data object field 
                    in_id = o['instance_num'] 
#                     print(in_id)
                    
                    # add the wnid to the data object field 
                    wnid = o['wnid']
#                     print(wnid)
                    
                    # add the name of the object to the data object
                    name = o['english']
                    
                
                    # get the object instance mask and its associated pixel area, bounding box and
                    # rle mask
                    pxl = instance_img==in_id
                    xmin = np.where(pxl)[1].min()
                    xmax = np.where(pxl)[1].max()
                    ymin = np.where(pxl)[0].min()
                    ymax = np.where(pxl)[0].max()
                    
                    
                    area = len(np.where(pxl)[0]) + len(np.where(pxl)[1])
                                     
                    
                    width = xmax-xmin
                    
                    height = ymax-ymin
                    
                    # format the bbox array to be json serializable
                    bbox = [int(xmin), int(ymin), int(width), int(height)]
                    
                    rle = mask.encode(np.asfortranarray(pxl.astype(np.uint8)))
                    
                    # format the rle object to be json serializable
                    json_rle = {
                        'counts': rle['counts'].decode('utf-8'),
                        'size': rle['size'],
                    }
                    
                    # add the information about the current object instance to the data object
                    data['annotations'].append({
                        'id': in_id,
                        'category_id': wnid,
                        'bbox': bbox,
                        'image_id': img_id,
                        'iscrowd': None,
                        'area': area,
                        'segmentation': json_rle,
                    })

                    data['categories'].append({
                        'supercategory': "",
                        'id': wnid,
                        'name': name,
                    })


# remove old json file
try:
    os.remove("instance.json")
except OSError:
    pass


# write data object to json file
with open("instance.json", "w") as write_file:
    json.dump(data, write_file)
    