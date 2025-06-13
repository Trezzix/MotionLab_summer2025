import PIL
import os
import shutil

import PIL.Image
import PIL.ImageEnhance
import random

import PIL.ImageFilter

labelsPath = "./dataset/labels"
imgPath = "./dataset/images"

minbrigthness = 0.3
maxBrightness = 1.7 # apply to dark as well
maxColorIntensity = 2.0
maxBlur = 1.5 # radius
runs = 2

newImgdir = "./dataset/imgAugmented"
newLabeldir = "./dataset/labelsAugmented"
os.makedirs(newImgdir, exist_ok=True)
os.makedirs(newLabeldir, exist_ok=True)
runsComplete = 0
while runsComplete < runs:
    for root, dirs, files in os.walk(imgPath):
        for file in files:
            img = PIL.Image.open(imgPath + "/" + file)
            if maxBrightness is not None: # apply random brightening
                filter = PIL.ImageEnhance.Brightness(img)
                randFac = max(random.random()*maxBrightness,minbrigthness)
                img = filter.enhance(randFac)
            if maxBlur is not None: # apply random blurring to same image
                randFac = random.random()*maxBlur
                filter = PIL.ImageFilter.GaussianBlur(randFac)
                img = img.filter(filter)
            #if maxColorIntensity is not None: # TODO
            # save image in new folder
            filename = '.'.join(os.path.basename(file).split('.')[:-1])
            img.save(newImgdir + "/" + filename + "_" + str(runsComplete) + "aug.jpg")
            # copy label and rename to same as img
            shutil.copy(labelsPath + "/" + filename + ".txt" ,newLabeldir + "/" + filename + "_" + str(runsComplete) + "aug.txt")
    runsComplete += 1
