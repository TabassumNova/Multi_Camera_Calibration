from .aravis import Camera

def main():
    #cam = Camera("Prosilica-02-2110A-06145")
    #cam = Camera("AT-Automation Technology GmbH-20805103")
    cam = Camera('The Imaging Source Europe GmbH-42120643')
    try:
        #Aravis.enable_interface ("Fake")
        #x, y, width, height = cam.get_region()
        print("Camera model: ", cam.get_model_name())
        print("Vendor Name: ", cam.get_vendor_name())
        print("Device id: ", cam.get_device_id())
        #print("Image size: ", width, ",", height)
        print("Sensor size: ", cam.get_sensor_size()) 
        print("Exposure: ", cam.get_exposure_time())
        print("Frame rate: ", cam.get_frame_rate())
        print("Payload: ", cam.get_payload())
        print("AcquisitionMode: ", cam.get_feature("AcquisitionMode"))
        print("Acquisition vals: ", cam.get_feature_vals("AcquisitionMode"))
        #print("TriggerMode: ", cam.get_feature("TriggerMode"))
        #print("Bandwidth: ", cam.get_feature("StreamBytesPerSecond"))
        print("PixelFormat: ", cam.get_feature("PixelFormat"))
        #print("ExposureAuto: ", cam.get_feature("ExposureAuto"))
        print("PacketSize: ", cam.get_feature("GevSCPSPacketSize"))


        from IPython import embed
        embed()
    finally:
        cam.shutdown()

