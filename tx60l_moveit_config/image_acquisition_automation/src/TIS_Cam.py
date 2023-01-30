import sys
import numpy
import gi
from collections import namedtuple

gi.require_version("Gst", "1.0")
gi.require_version("GLib", "2.0")
gi.require_version('Tcam', '1.0')

from gi.repository import Tcam, Gst, GLib, GObject

DeviceInfo = namedtuple("DeviceInfo", "status name identifier connection_type")
CameraProperty = namedtuple("CameraProperty", "status value min max default step type flags category group")



# This sample shows, how to get an image and exchange a pixel color
# needed packages:
# pyhton-gst-1.0
# tiscamera

# Open camera, set video format, framerate and determine, whether the sink is color or bw
# Parameters: Serialnumber, width, height, framerate , color
# If color is False, then monochrome / bw format is in memory. If color is True, then RGB32
# colorformat is in memory

def on_new_buffer(appsink):
    print("Hallo")
    return False

def print_device(device):
    """

    """

    # struc is a Gst.Structure
    struc = device.get_properties()

    print("\tmodel:\t{}\tserial:\t{}\ttype:\t{}".format(struc.get_string("model"),
                                                        struc.get_string("serial"),
                                                        struc.get_string("type")))

def TIS_list_devices():
    Gst.init(sys.argv)  # init gstreamer

    # this line sets the gstreamer default logging level
    # it can be removed in normal applications
    # gstreamer logging can contain verry useful information
    # when debugging your application
    # see https://gstreamer.freedesktop.org/documentation/tutorials/basic/debugging-tools.html
    # for further details
    Gst.debug_set_default_threshold(Gst.DebugLevel.WARNING)

    monitor = Gst.DeviceMonitor.new()

    monitor.add_filter("Video/Source/tcam")

    #
    # static query
    # list all devices that are available right now
    #
    device_list = monitor.get_devices()

    return device_list


class TIS:
    'The Imaging Source Camera'

    def __init__(self, serial, width, height, numerator, denumerator, color):
        Gst.init([])
        self.height = height
        self.width = width
        self.sample = None
        self.samplelocked = False
        self.newsample = False
        format = "BGRx"
        if (color == False):
            format = "GRAY8"

        p = 'tcambin serial="%s" name=source ! video/x-raw,format=%s,width=%d,height=%d,framerate=%d/%d' % (
        serial, format, width, height, numerator, denumerator,)
        p += ' ! videoconvert ! appsink name=sink'

        try:
            self.pipeline = Gst.parse_launch(p)
        except GLib.Error as error:
            print("Error creating pipeline: {0}".format(err))
            raise

        self.pipeline.set_state(Gst.State.READY)
        self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
        # Query a pointer to our source, so we can set properties.
        self.source = self.pipeline.get_by_name("source")

        # Query a pointer to the appsink, so we can assign the callback function.
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.set_property("max-buffers", 5)
        self.appsink.set_property("drop", 1)
        self.appsink.set_property("emit-signals", 1)
        self.appsink.connect('new-sample', self.on_new_buffer)

    def on_new_buffer(self, appsink):
        self.newsample = True
        if (self.samplelocked == False):
            try:
                self.sample = appsink.get_property('last-sample')
            except GLib.Error as error:
                print("Error on_new_buffer pipeline: {0}".format(err))
                raise
        return False

    def Start_pipeline(self):
        try:
            self.pipeline.set_state(Gst.State.PLAYING)
            self.pipeline.get_state(Gst.CLOCK_TIME_NONE)


        except GLib.Error as error:
            print("Error starting pipeline: {0}".format(err))
            raise

    def Get_image(self):
        # Sample code from https://gist.github.com/cbenhagen/76b24573fa63e7492fb6#file-gst-appsink-opencv-py-L34
        if (self.sample != None and self.newsample == True):
            self.samplelocked = True
            buf = self.sample.get_buffer()
            caps = self.sample.get_caps()
            bpp = 4
            if (caps.get_structure(0).get_value('format') == "BGRx"):
                bpp = 4

            if (caps.get_structure(0).get_value('format') == "GRAY8"):
                bpp = 1

            self.img_mat = numpy.ndarray(
                (caps.get_structure(0).get_value('height'),
                 caps.get_structure(0).get_value('width'),
                 bpp),
                buffer=buf.extract_dup(0, buf.get_size()),
                dtype=numpy.uint8)
            self.newsample = False
            self.samplelocked = False
            return self.img_mat
        return None

    def Stop_pipeline(self):
        self.pipeline.set_state(Gst.State.PAUSED)
        self.pipeline.set_state(Gst.State.READY)
        self.pipeline.set_state(Gst.State.NULL)

    def List_Properties(self):
        for name in self.source.get_tcam_property_names():
            print(name)

    def Get_Property(self, PropertyName):
        try:
            return CameraProperty(*self.source.get_tcam_property(PropertyName))
        except GLib.Error as error:
            print("Error get Property {0}: {1}", PropertyName, format(error))
            raise

    def Set_Property(self, PropertyName, value):
        try:
            self.source.set_tcam_property(PropertyName, GObject.Value(type(value), value))
        except GLib.Error as error:
            print("Error set Property {0}: {1}", PropertyName, format(error))
            raise



