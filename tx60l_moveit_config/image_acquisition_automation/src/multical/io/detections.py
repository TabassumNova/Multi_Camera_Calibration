
import pickle
import os
import sys
from src.multical.io.logging import info
from pathlib import Path

from structs.struct import struct
# from py_structs import struct

def try_load_detections(filename, cache_key={}):
  try:
    
    with open(filename, "rb") as file:
      loaded = pickle.load(file)
      
      # Check that the detections match the metadata
      if (loaded.get('cache_key', {}) == cache_key or check_similarity(loaded, cache_key)):
        info(f"Loaded detections from {filename}")
        return loaded.detected_points
      else:
        check_similarity(loaded, cache_key)
        info(f"Config changed, not using loaded detections in {filename}")
  except (OSError, IOError, EOFError, AttributeError) as e:
    return None

def check_similarity(loaded, cache_key):
  filenames = loaded.cache_key['filenames']
  caches = cache_key['filenames']
  new_cam = {'08320217':'cam1', '08320218':'cam2', '08320220':'cam3', '08320221':'cam4', '08320222':'cam5', '36220113':'cam6'}

  assert len(filenames) == len(caches)
  for i in range(len(filenames)):
    assert len(filenames[i]) == len(caches[i])
    for j in range(len(filenames[i])):
      file_dirs = find_char(filenames[i][j])
      cache_dirs = find_char(caches[i][j])
      if (file_dirs[-3:] == cache_dirs[-3:]):
        # new_path = os.path.join('icosahedron', new_cam[file_dirs[-2]], file_dirs[-1])
        # filenames[i][j] = new_path
        continue
      else:
        return False
  # with open('/home/nova/Desktop/Nova/Calibration_paper/datasets/V35/icosahedron.detections.pkl', 'rb') as file:
  #   myvar = pickle.load(file)
  #   with open('icosahedron.detections.pkl', 'wb') as file:
  #     pickle.dump(loaded, file)
  return True

def find_char(str):
  x = str.split("\\")
  y = str.split("/")
  return x if len(x) > len(y) else y

def write_detections(filename, detected_points, cache_key={}):
  print("write_filename: ",filename)
  data = struct(
    cache_key = cache_key,
    detected_points = detected_points
  )
  with open(filename, "wb") as file:
    pickle.dump(data, file)