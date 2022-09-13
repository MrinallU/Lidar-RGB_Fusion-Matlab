% PROJECT PTCLOUD TO IMAGE FOR IMAGE DEPTH MAP

pcapFileName = "dataLID.pcap";
calibFileName = "dataLID.json";
im = imread("environment_img.jpg"); 

ousterReader = ousterFileReader(pcapFileName,calibFileName);
frameTime = ousterReader.StartTime + seconds(3);
[ptCloud,pcatt] = readFrame(ousterReader,frameTime);
p1 = pcdownsample(ptCloud,'gridAverage',0.5);

% CAMERA PARAMS 
theta = 0;  
focalLengthX = 1.98359898e+03; 
focalLengthY = 1.98783529e+03; 
imageCenterX = 9.34531049e+02; 
imageCenterY =  5.63150590e+02; 
imageResX = 4000; 
imageResY = 3000; 

focalLength    = [focalLengthX, focalLengthY]; 
principalPoint = [imageCenterX, imageCenterY]; % cx, cy
imageSize      = [imageResX, imageResY]; % row, col


% ASSEMBLING MATRICIES
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize); 
rot = [ cosd(theta) sind(theta) 0; ...
       -sind(theta) cosd(theta) 0; ...
      0 0 1]; 
trans = [-0.2 11.2 -1];
camToLidar = rigid3d(rot, trans); 
lidarToCam = invert(camToLidar);


imPts = projectLidarPointsOnImage(p1,intrinsics,lidarToCam);
figure
imshow(im)
hold on
plot(imPts(:,1),imPts(:,2),'.','Color','r')
hold off
