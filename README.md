# GRM_Project

To use it, you need to have opencv (to open the images) installed and set up (for visual studio, https://www.tutorialspoint.com/how-to-install-opencv-for-cplusplus-in-windows)

Also, I used Visual Studio 2022 if you want to use the solution.

I added an executable in case you want to have a run without building it, to use it (windows x64) follow those instructions:

(note that the weights and thresholds are hard coded in both case, but the results should be pleasing nonetheless)

## For denoising 
it noise the image first, you can give it a (not too big) file in color if you want:

GRM_Project.exe path/to/file

## For Stereo matching

GRM_Project.exe path/to/file/left path/to/file/right
