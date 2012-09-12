#!/usr/bin/env python

import sys
from xml.etree import ElementTree
import matplotlib.pyplot as plt
import matplotlib.figure as fig

if len(sys.argv) < 6:
   print "Usage: " + sys.argv[0] + " mason_sliders_xml generated_sliders_csv hysteresis_csv_out hysteresis_threshold window_average_radius"
   sys.exit(0)

slider_names = [ "size1", "x1", "y1", "size2", "x2", "y2", "activity", "density", "excitement" ]
sliders_to_hysteresis = [ "size1", "size2" ]
sliders_to_window_average = [ "density", "activity" ]

mason_sliders_xml = ElementTree.parse(open(sys.argv[1], "r"))
gen_sliders_file = open(sys.argv[2], "r")
hysteresis_csv_out_path = sys.argv[3]
hysteresis_threshold = float(sys.argv[4])
window_average_radius = float(sys.argv[5])


mason_frames = []
gen_frames = []

mason_sliders = {}
gen_sliders = {}
hysteresis_sliders = {}
selective_hysteresis_sliders = {}
selective_hyst_and_avg_sliders = {}
union_mason_sliders = {}
union_gen_sliders = {}
union_hysteresis_sliders = {}

for slider_name in slider_names:
   mason_sliders[slider_name] = []
   gen_sliders[slider_name] = []
   hysteresis_sliders[slider_name] = []
   union_mason_sliders[slider_name] = []
   union_gen_sliders[slider_name] = []
   union_hysteresis_sliders[slider_name] = []

slider_count = None

def hysteresis(values_in):
   values_out = []
   last = values_in[0]
   for value_in in values_in:
      if abs(value_in - last) > hysteresis_threshold:
         last = value_in
      values_out.append(last)
   return values_out

def get_window_average(values, index, radius):
   first_index = int(max(index - radius, 0))
   last_index  = int(min(index + radius, len(values)-1))
   sample_count = last_index - first_index + 1
   weighted_sum = 0.0
   for i in range(first_index, last_index+1):
      weighted_sum = weighted_sum + float(values[i]) / float(sample_count)
   return weighted_sum

def window_average(values_in):
   values_out = []
   index = 0
   for value_in in values_in:
      window_average = get_window_average(values_in, index, window_average_radius)
      values_out.append(window_average)
      index = index + 1
   return values_out

# parse mason's sliders from XML
tags = mason_sliders_xml.iter()
try:
   while True:
      tag = tags.next()
      if tag.tag == "FRAME":
         mason_frames.append(int(tag.attrib['id']))
         mason_sliders['size1'].append(int(tag.find("FISH1").attrib['size']))
         mason_sliders['x1'].append(int(tag.find("FISH1").attrib['x']))
         mason_sliders['y1'].append(int(tag.find("FISH1").attrib['y']))
         mason_sliders['size2'].append(int(tag.find("FISH2").attrib['size']))
         mason_sliders['x2'].append(int(tag.find("FISH2").attrib['x']))
         mason_sliders['y2'].append(int(tag.find("FISH2").attrib['y']))
         mason_sliders['activity'].append(int(tag.find("OVERALL").attrib['activity']))
         mason_sliders['density'].append(int(tag.find("OVERALL").attrib['density']))
         mason_sliders['excitement'].append(int(tag.find("OVERALL").attrib['excitement']))
except StopIteration:
   pass


# parse generated sliders from CSV (actually "Space Separated Values")
for line in gen_sliders_file:
   elements = line.split("\n")[0].split(" ")
   gen_frames.append(int(elements[0]))
   frame_sliders = [float(e) for e in elements[1:]]
   slider_count = len(frame_sliders)
   for slider_name in slider_names:
      gen_sliders[slider_name].append(frame_sliders[slider_names.index(slider_name)])

# do hysteresis on generated sliders
for slider_name in slider_names:
   hysteresis_sliders[slider_name] = hysteresis(gen_sliders[slider_name])
   selective_hysteresis_sliders[slider_name] = gen_sliders[slider_name]
   selective_hyst_and_avg_sliders[slider_name] = gen_sliders[slider_name]
for slider_name in sliders_to_hysteresis:
   selective_hysteresis_sliders[slider_name] = hysteresis_sliders[slider_name]
   selective_hyst_and_avg_sliders[slider_name] = hysteresis_sliders[slider_name]
for slider_name in sliders_to_window_average:
   selective_hyst_and_avg_sliders[slider_name] = window_average(gen_sliders[slider_name])

# do window average


mason_slider_frame_count = len( mason_frames )
gen_slider_frame_count = len( gen_frames )

#print sliders
print str(mason_slider_frame_count) + " mason slider frames"
print str(gen_slider_frame_count) + " generated & hysteresis slider frames"

all_hysteresis_out       = hysteresis_csv_out_path + ".all_hyst.csv"
selective_hysteresis_out = hysteresis_csv_out_path + ".selective_hyst.csv"
all_hysteresis_csv_out       = open(all_hysteresis_out, "w")
selective_hysteresis_csv_out = open(selective_hysteresis_out, "w")
present_frames_index = 0

desired_frame = gen_frames[0]
last_explicit_desired_frame = None
last_explicit_frame_index = None

for gen_frame in gen_frames:

   step = 1

   while desired_frame < gen_frame:
      # output interpolations
      selective_hysteresis_csv_out.write(str(desired_frame) + " ")

      steps_between_explicit_frames = gen_frame - last_explicit_desired_frame

      for slider_name in slider_names:
         previous_explicit_value = selective_hysteresis_sliders[slider_name][last_explicit_frame_index]
         next_explicit_value = selective_hysteresis_sliders[slider_name][last_explicit_frame_index+1]
         value_delta = next_explicit_value - previous_explicit_value

         interp_value = float(previous_explicit_value) + float(value_delta) * float(step) / float(steps_between_explicit_frames)

         selective_hysteresis_csv_out.write(str(interp_value) + " ")

      step = step + 1
      
      selective_hysteresis_csv_out.write("\n")
      desired_frame = desired_frame + 1
   
   # then output gen frame
   last_explicit_desired_frame = gen_frame
   frame = gen_frame

   all_hysteresis_csv_out.write(str(frame) + " ")
   selective_hysteresis_csv_out.write(str(frame) + " ")
   for slider_name in slider_names:
      all_hysteresis_csv_out.write(str(hysteresis_sliders[slider_name][present_frames_index]) + " ")
      selective_hysteresis_csv_out.write(str(selective_hysteresis_sliders[slider_name][present_frames_index]) + " ")
   all_hysteresis_csv_out.write("\n")
   selective_hysteresis_csv_out.write("\n")

   desired_frame = gen_frame + 1
   last_explicit_frame_index = present_frames_index

   present_frames_index = present_frames_index + 1

print "saved " + hysteresis_csv_out_path

# do a union of the frames indices
"""
all_frames = mason_frames
all_frames.extend(gen_frames)
all_frames = list(set(all_frames))
for frame in all_frames:
   for slider_name in slider_names:
      if frame in mason_frames:
         union_mason_sliders[slider_name].append(mason_sliders[slider_name][frame])
      else:
         union_mason_sliders[slider_name].append(None)
      if frame in gen_frames:
         union_gen_sliders[slider_name].append(gen_sliders[slider_name][frame])
         union_hysteresis_sliders[slider_name].append(hysteresis_sliders[slider_name][frame])
      else:
         union_gen_sliders[slider_name].append(None)
         union_hysteresis_sliders[slider_name].append(None)
"""


sys.exit(0)

html = '<html><head><title>Selective Hysteresis and Window Average</title></head><body><table cellpadding="0" cellspacing="0" border="0">'

for slider_name in slider_names:

   # plot signal

   plt.clf()

   plt.subplot(3, 1, 1)
   plt.title(slider_name)
   plt.ylabel("mason")
   #plt.title("mason slider " + slider_name)
   #plt.plot(all_frames, union_mason_sliders[slider_name], linewidth=0.5)
   plt.plot(mason_frames, mason_sliders[slider_name], linewidth=0.5)

   plt.subplot(3, 1, 2)
   plt.ylabel("generated")
   #plt.plot(all_frames, union_gen_sliders[slider_name], linewidth=0.5)
   plt.plot(gen_frames, gen_sliders[slider_name], linewidth=0.5)

   plt.subplot(3, 1, 3)
   plt.ylabel("generated filtered")
   #plt.plot(all_frames, union_hysteresis_sliders[slider_name], linewidth=0.5)
   plt.plot(gen_frames, selective_hyst_and_avg_sliders[slider_name], linewidth=0.5)

   image_name = "slider_" + slider_name + ".png"
   image_path = "html/" + image_name
   plt.savefig(image_path)
   print "saved " + image_path
   html = html + '<tr><td><img src="'+image_name+'"></td>'

   # plot histogram

   plt.clf()

   plt.subplot(3, 1, 1)
   plt.title(slider_name)
   plt.ylabel("mason")
   #plt.title("mason slider " + slider_name)
   #plt.plot(all_frames, union_mason_sliders[slider_name], linewidth=0.5)
   plt.hist(mason_sliders[slider_name], 50, linewidth=0.5)

   plt.subplot(3, 1, 2)
   plt.ylabel("generated")
   #plt.plot(all_frames, union_gen_sliders[slider_name], linewidth=0.5)
   plt.hist(gen_sliders[slider_name], 50, linewidth=0.5)

   plt.subplot(3, 1, 3)
   plt.ylabel("hysteresis( generated )")
   #plt.plot(all_frames, union_hysteresis_sliders[slider_name], linewidth=0.5)
   plt.hist(hysteresis_sliders[slider_name], 50, linewidth=0.5)

   image_name = "histogram_" + slider_name + ".png"
   image_path = "html/" + image_name
   plt.savefig(image_path)
   print "saved " + image_path
   html = html + '<td><img src="'+image_name+'"></td></tr>'


   continue

   plt.subplot(3, 1, 2)
   plt.title("mason slider " + slider_name + " histogram")
   plt.hist(sliders[slider_name], 50)

   plt.savefig("mason_slider_"+slider_name+".png")


   plt.clf()

   plt.subplot(2, 2, 1)
   plt.title("slider " + str(slider_index) + " computer vision")
   plt.plot(frame_numbers, sliders[slider_index], linewidth=0.2)

   plt.subplot(2, 2, 3)
   plt.title("slider " + str(slider_index) + " hysteresis")
   plt.ylabel("hysteresis threshold = " + str(hysteresis_threshold))
   plt.plot(frame_numbers, hysteresis_sliders[slider_index], linewidth=0.2)

   plt.subplot(2, 2, 2)
   plt.title("slider " + str(slider_index) + " computer vision histogram")
   plt.hist(sliders[slider_index], 25, align="left", rwidth=0.5)
   plt.hist(hysteresis_sliders[slider_index], 25, align="mid", rwidth=0.5)

   plt.subplot(2, 2, 4)
   plt.title("slider " + str(slider_index) + " hysteresis histogram")
   plt.hist(hysteresis_sliders[slider_index], 50)


html = html + '</table></body></html>'

html_path = "html/index.html"
html_file = file("html/index.html", "w")
html_file.write(html)
print "saved " + html_path
