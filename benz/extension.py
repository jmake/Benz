import omni.ext
import omni.ui as ui


import os 
import sys
import pathlib
path_root = pathlib.Path(__file__)
sys.path.append(str(path_root.parent))

#import Vehicles 
from Vehicles import g63 as Vehicle 


Vehicle.Create() 


class TutorialBenzExtension(omni.ext.IExt):

    def on_startup(self, ext_id) :
        print("[tutorial.benz] tutorial benz startup")

    def on_shutdown(self):
        print("[tutorial.benz] tutorial benz shutdown")
