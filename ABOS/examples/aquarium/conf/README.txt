These are the different parameters being used:-

iterations, levels, pyrScale, winSize - parameters for optical flow algorithm
velThresh - minimum flow of a pixel after computing optical flow
areaThresh - minimum area of a blob after velocity thresholding
velMax - maximum predicted average velocity of the frame. used to normalize the velocity to 127 (Mason slider's maximum)
areaMax - maximum predicted area of a blob. used to normalize.
pixelsMax - maximum no. of predicted moving pixels in a frame. again, used to normalize
