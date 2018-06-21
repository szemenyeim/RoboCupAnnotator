# RoboCupAnnotator
Simple app for labelling images for RoboCup 

This application&#39;s purpose is to aid the user in creating pixel-level semantic annotations for robot soccer images. The program allows you to annotate 5 semantic classes (field/background/unlabeled – black; ball – red; robot – green; goalpost – blue; field line – purple).

## 1.Requirements and build

In order to build the application, you will need two third party libraries:

- OpenCV (version 3.1 or later, WITH CONTRIB)
- Boost

Please note, that the program requires the OpenCV contrib library, so if you are building the library from source, make sure to build contrib as well. On Mac and Ubuntu the versions installed using package managers (such as Homebrew or Apt) will do fine.

Once these are installed, you can use the makefile to build the application, or on MAC you can use the Xcode project as well. If you want to use your own build system, look for the necessary include and link options in the makefile.

Once built, run according to the following usage:

    robocupannotator &lt;path-to-dataset&gt;

The dataset path should have a folder named &quot;images&quot; that contains your image files. The labels will be saves in the &quot;labels&quot; folder in the same level.

## 2.Features

The annotator program provides five tools to annotate your images:

- Circular brush
- Square brush
- Draw and fill polygons
- Draw lines
- Fill superpixels

You can adjust the size of the brushes and superpixels, or the thickness of the line. You can also use the overwrite protection feature, in which case your tools will only overwrite pixels labeled as &quot;background&quot;.

In case of image sequences the program is able to estimate the labels of the next image using dense optical flow. Enable the optical flow feature in order to get this functionality.

## 3.Using the Horizon Clipping

Run the annotator with an extra option:

    robocupannotator &lt;path-to-dataset&gt; --horizon

The program will allow you to erase the parts of the image and labels outside the soccer field. We recommend doing this after the labelling is complete and all the original labels are saved. The program will save the modified image set separately into the &quot;horImages&quot; and &quot;horLabels&quot; folders in the main dataset folder.

In order to set the horizon, select the line tool and put down two or three points to define the field edges. Then press &#39;h&#39; to save the modified image. If the image does not contain the field edge, press &#39;g&#39; instead. **IMPORTANT**: The line you define persists through the images (this makes cropping image sequences much easier), so if you need to redefine the field edge line, press &#39;c&#39; to clear the previous one.
