# import sys
# import os
# # sys.path.append(os.path.dirname(os.path.abspath(__file__)))
# sys.path.insert(0, os.path.abspath('./'))

# sys.path.insert(1, 'D:/MY_DRIVE_N/Masters_thesis/multical_test_new/multical-master/multical-master/multical/config')


from dataclasses import dataclass
from multical.config.arguments import run_with
from multical.io.logging import MemoryHandler, info


# from multiprocessing import cpu_count
from typing import Union


from multical.app.boards import Boards
from multical.app.calibrate import Calibrate
from multical.app.intrinsic import Intrinsic
from multical.app.vis import Vis


@dataclass
class Multical:
  """multical - multi camera calibration 
  - calibrate: multi-camera calibration
  - intrinsic: calibrate separate intrinsic parameters
  - boards: generate/visualize board images, test detections
  - vis: visualize results of a calibration 
  """ 
  info("step: multical.app.Multical")
  print("step: multical.app.Multical")
  command : Union[Calibrate, Intrinsic, Boards, Vis]
  # print(command)
   
  def execute(self):
    info("step: self.command.execute()")
    print("step: self.command.execute()")
    # print((self.command))
    return self.command.execute()
    


def cli():
  
  print("cli() ",Multical)
  run_with(Multical)
  info("step: run_with(Multical)")
  
  
  

if __name__ == '__main__':
  cli()
