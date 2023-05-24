from copy import copy

from multical.io.logging import error
from multical.board.board import Board
from structs.numpy import Table
from multical.board.common import *
from pprint import pformat
from cached_property import cached_property
import cv2
import numpy as np

from structs.struct import struct, choose, subset
from multical.optimization.parameters import Parameters


def import_Checkerboard():
  try:
    import aprilgrid
    return aprilgrid
  except ImportError as err:
    error(err)
    error("aprilgrid support depends on apriltags2-ethz, a pip package (linux only)")

class Checkerboard(Parameters, Board):
    def __init__(self, size, square_length):

        self.size = tuple(size)
        self.square_length = square_length