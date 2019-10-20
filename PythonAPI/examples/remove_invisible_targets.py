import matplotlib.pyplot as plt
import numpy as np
from scipy import misc

scenario = "out8"
target_type = "vehicle"

semseg_folder = "/home/tamert/carla/PythonAPI/examples/_{0}/semseg".format(scenario)
bbox_filepath = "/home/tamert/carla/PythonAPI/examples/{0}_bboxes_{1}.txt".format(target_type, scenario)
bbox_outfilepath = "/home/tamert/carla/PythonAPI/examples/{0}_bboxes_{1}_vis.txt".format(target_type, scenario)

redValForTargetType = {"pedestrian":  4,
                       "vehicle":    10}

def doesPixelMatchTarget(color, targetType):
    redVal = round(color[0] * 255)
    return redVal == redValForTargetType[targetType]

def doesRectContainTarget(image, rectangle, targetType):
    for x in range(rectangle[0], rectangle[1] + 1):
        if x < 0 or x >= image.shape[1]:
            continue
        for y in range(rectangle[2], rectangle[3] + 1):
            if y < 0 or y >= image.shape[0]:
                continue

            color = image[y, x, :]
            if doesPixelMatchTarget(color, targetType):
                return True

    return False

def shouldKeepLine(line, semsegFolder, targetType):
    s = line.strip().split("|")
    if len(s) == 0:
        return False

    # Always keep the camera information
    if s[1] == "-1":
        return True

    # Look up the corresponding image
    imagePath = "{0}/{1:08d}.png".format(semsegFolder, int(s[0]))
    image = plt.imread(imagePath)

    xmin, xmax, ymin, ymax = s[2:6]
    rect = (int(xmin), int(xmax), int(ymin), int(ymax))
    return doesRectContainTarget(image, rect, targetType)

def parseFile(inFilepath, semsegFolder, targetType, outFilepath):
    fin = open(inFilepath, 'r')
    fout = open(outFilepath, 'w')

    for line in fin:
        if shouldKeepLine(line, semsegFolder, targetType):
            fout.write(line)

    fin.close()
    fout.close()

if __name__ == "__main__":
    parseFile(bbox_filepath, semseg_folder, target_type, bbox_outfilepath)