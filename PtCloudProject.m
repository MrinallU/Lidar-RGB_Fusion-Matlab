% PROJECT PTCLOUD TO IMAGE FOR IMAGE DEPTH MAP

pcapFileName = "dataLID.pcap";
calibFileName = "dataLID.json";
im = imread("frame.png"); 

ousterReader = ousterFileReader(pcapFileName,calibFileName);
frameTime = ousterReader.StartTime + seconds(23);
[ptCloud,pcatt] = readFrame(ousterReader,frameTime);
p1 = pcdownsample(ptCloud,'gridAverage',0.5);

% CAMERA PARAMS 
theta = 0;  
focalLengthX = 1983.59898; 
focalLengthY = 1987.83529;
imageCenterX = 563; 
imageCenterY =  934; 
imageResX = 1080; 
imageResY = 1920; 

focalLength    = [focalLengthX, focalLengthY]; 
principalPoint = [imageCenterX, imageCenterY]; % cx, cy
imageSize      = [imageResX, imageResY]; % row, col


% ASSEMBLING MATRICIES
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize); 
disp(intrinsics); 
rot = [0 -1 0; ...
       0 0 -1; ...
      1 0 0]; 
%trans = [0.00508 -0.28448 0.0254];
trans = [0.00508 -0.28448 0.0254];
lidarToCam = rigid3d(rot, trans); 


imPts = projectLidarPointsOnImage(p1,intrinsics,lidarToCam);
%disp(imPts);   
figure
imshow(im)
hold on
plot(imPts(:,1),imPts(:,2),'.','Color','r')
hold off
