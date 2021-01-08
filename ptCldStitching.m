close all; clear; clc; 

ptCloudRef = pcread('out0203.ply');
ptCloudCurrent = pcread('out0304.ply');

% gridSize = 0.01;
% fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
% moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

tform = pcregistericp(ptCloudRef, ptCloudCurrent, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(ptCloudCurrent,tform);

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

% Visualize the input images.
figure
subplot(2,2,1)
imshow(ptCloudRef.Color)
title('First input image','Color','w')
drawnow

subplot(2,2,3)
imshow(ptCloudCurrent.Color)
title('Second input image','Color','w')
drawnow

% Visualize the world scene.
subplot(2,2,[2,4])
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', 'MarkerSize', 200)
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
pcwrite(ptCloudScene, 'out.ply');