import time

from robots_orchestra.controller.controller import Controller
from robots_orchestra.viz.viser import ViserUI

def main():
    viser_ui = ViserUI()
    controller = Controller(viser_ui)
    controller.run()
    

if __name__ == "__main__":
    main()