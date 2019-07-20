import argparse
import droneHelper
import treeRecognition

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-y", "--yolo", required=True,
        help="base path to YOLO directory")
ap.add_argument("-c", "--confidence", type=float, default=0.1,
        help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0.3,
        help="threshold when applyong non-maxima suppression")
ap.add_argument('--connect', default='127.0.0.1:14550',
        help="address of to connect")
args = vars(ap.parse_args())


def main():
    treeRecognition.setupArgument(args["yolo"], args["confidence"], args["threshold"])
    vehicle = droneHelper.connection(args["connect"], 921900)
    while(True):
        if vehicle.mode == "OFFBOARD":
            droneHelper.derive(vehicle)





if __name__ == "__main__":
    main()