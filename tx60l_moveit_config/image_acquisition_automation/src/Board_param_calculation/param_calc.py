from src.multical.tables import *
from structs.struct import choose, struct

def board_param(ws):
    name = 'board'
    table = ws.pose_table
    n = table._shape[2]
    overlaps = pattern_overlaps(table, axis=2)

    info(f"Overlaps by {name}:")
    info(overlaps)

    master, pairs = graph.select_pairs(overlaps, hop_penalty=0.9)
    info(f"Selected master {master} and pairs {pairs}")

    pose_dict = {master: np.eye(4)}

    for parent, child in pairs:
        t = estimate_transform(table, parent, child, axis=2)
        pose_dict[child] = t @ pose_dict[parent]

    new_dict = {}
    for keys in pose_dict:
         new_dict[keys] = matrix.relative_to(pose_dict[keys], pose_dict[0])
    pass
    return new_dict

def estimate_transform(table, i, j, axis=0):
  table_i = table._index_select(i, axis=axis)
  table_j = table._index_select(j, axis=axis)

  assert table_i._shape == table_j._shape
  valid = (table_i.valid & table_j.valid).ravel()

  table_i = np.array([table_i._index_select(idx, axis=1).poses for idx, i in enumerate(valid) if i == True]).reshape(-1,4,4)
  table_j = np.array([table_j._index_select(idx, axis=1).poses for idx, j in enumerate(valid) if j == True]).reshape(-1,4,4)

  T1 = matrix.align_transforms_mean(table_i, table_j)

  transformation = 0
  for pose in range(0, table_i.shape[0]):
      T = matrix.relative_to(np.linalg.inv(table_j[pose]), np.linalg.inv(table_i[pose]))
      transformation += T
  final_T = transformation/len(table_i)


  return final_T