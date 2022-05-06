import json
import argparse
import os
import shutil
from PIL import Image, ImageDraw, ImageFont

# simple python script to convert a series of maze text files into a .gif image

parser = argparse.ArgumentParser() # parsing argument for target simulation directory
parser.add_argument("dir")
args = parser.parse_args()

directory = args.dir # getting directory from arguments
img_directory = directory + "frames/" # directory for .png exporting
simulation_json_name = "Simulation.json" # name of simulation info file

file = open(directory + simulation_json_name) # opening json file
simulation_json = json.load(file) # putting simulation.json into a dictionary

num_of_printouts = simulation_json["Info"]["Number_of_Printouts"] # getting number of printouts

try: # try to make directory for images
    os.mkdir(img_directory) # making directory to export images to
except: # if directory already exists
    shutil.rmtree(img_directory) # delete image directory and its contents
    os.mkdir(img_directory) # making the directory again

x_size = 0 # variable to determine x limit of maze image
y_size = 0 # variable to determine y limit of maze image

print("Generating images for gif creation...")

for i in range(1,num_of_printouts + 1): # iterate through all text files to make png versions
    
    if i == 1: # if this is the first maze, need to determine image size
        test_txt = open(directory + "printouts/printout_" + str(i) + ".txt") # opening first text file
        test_image = Image.new(mode="RGB", size=(10000, 10000), color=(255,255,255)) # creating an image which is large enough to fit maze fully
        test_draw = ImageDraw.Draw(test_image) # create draw object to allow for text printing on image
        test_font = ImageFont.truetype("/usr/share/fonts/noto/NotoMono-Regular.ttf/NotoMono-Regular.ttf",size=15) # selecting font
        test_draw.text((0,0), test_txt.read(), (0,0,0), font=test_font) # drawing text file into large image

        for j in range (0, 10000): # iterating through pixels to determine x and y maximum of image
            pix1 = test_image.getpixel((j, 25)) # getting pixe;s
            pix2 = test_image.getpixel((5, j)) 

            if pix1 == (0,0,0): # if this pixel is black
                x_size = j # mark it as new x maximum
                
            if pix2 == (0,0,0): # if this pixel is black
                y_size = j # mark it as new y maximum
    
    txt_file = open(directory + "printouts/printout_" + str(i) + ".txt")  # opening text file for gif creation
    image = Image.new(mode="RGB", size=(x_size + 5, y_size + 10), color=(255,255,255)) # creating a new image using the x_size and y_size maximums
    draw = ImageDraw.Draw(image) # creating draw object
    font = ImageFont.truetype("/usr/share/fonts/noto/NotoMono-Regular.ttf/NotoMono-Regular.ttf",size=15) # getting font
    draw.text((0,0), txt_file.read(), (0,0,0), font=font) # drawing maze from txt file into image
    image.save(img_directory + "frame_" + str(i) + ".png") # adding generated images to list

    print("Number of images generated: " + str(i) + "/" + str(num_of_printouts + 1), end='\r') # printing progress

print("All images successfully generated!") # image creation completion printout

os.system("ffmpeg -f image2 -framerate 15 -i " + img_directory + "frame_%d.png -loop 0 " + directory + "maze_exploration.gif") # using ffmpeg to make a gif using generated images