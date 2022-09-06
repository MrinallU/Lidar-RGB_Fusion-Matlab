
% Outster to ptcloud
zipFile = matlab.internal.examples.downloadSupportFile("lidar","data/ouster_RoadIntersection.zip");
saveFolder = fileparts(zipFile);
pcapFileName = [saveFolder filesep 'ouster_RoadIntersection' filesep 'ouster_RoadIntersection.pcap'];
calibFileName = [saveFolder filesep 'ouster_RoadIntersection' filesep 'OS1-128U.json'];
if ~(exist(pcapFileName,"file") && exist(calibFileName,"file"))
    unzip(zipFile,saveFolder);
end

ousterReader = ousterFileReader(pcapFileName,calibFileName);
frameTime = ousterReader.StartTime + seconds(3);
[ptCloud,pcatt] = readFrame(ousterReader,frameTime);
pcshow(ptCloud)

% Sample Lidar to RGB fusion, replace when matrices are given
%dataPath = fullfile(toolboxdir('lidar'),'lidardata','lcc','sampleColoredPtCloud.mat');
%gt = load(dataPath);
%im = gt.im;
%ptCloud = gt.ptCloud;

%pcshow(ptCloud)
%title('Original Point Cloud')

%intrinsics = gt.camParams;
%camToLidar = gt.tform;

% Values required are given. 
theta = 30;  
focalLengthX = 0; 
focalLengthY = 0; 
skew = 0; % This is probably 0
imageCenterX = 0; 
imageCenterY = 0; 

intrinsics = [ focalLengthX skew imageCenterX; ...
       0 focalLengthY imageCenterY; ...
      0 0 1]; 
rot = [ cosd(theta) sind(theta) 0; ...
       -sind(theta) cosd(theta) 0; ...
      0 0 1]; 
trans = [2 3 4];
camToLidar = rigid3d(rot, trans); 

ptCloudOut = fuseCameraToLidar(im,ptCloud,intrinsics,camToLidar);
pcshow(ptCloudOut)
title('Colored Point Cloud')


