close all; clear; clc; 

ins1 = [3979.911 0 0; 0 3979.911 0; 1244.772 1019.507 1];
ins2 = [3979.911 0 0; 0 3979.911 0; 1369.115 1019.507 1];
leftname  = 'sample01/im0.png';
rightname = 'sample01/im1.png';
outname   = 'out0102.ply';

main(leftname, rightname, ins1, ins2, outname);

function main(leftname, rightname, ins1, ins2, outname)
    left  = imread(leftname);
    right = imread(rightname);

    % Load Camera Parameters
    ins1 = [3979.911 0 0; 0 3979.911 0; 1244.772 1019.507 1];
    ins2 = [3979.911 0 0; 0 3979.911 0; 1369.115 1019.507 1];
    cameraParams1 = cameraParameters('IntrinsicMatrix', ins1);
    cameraParams2 = cameraParameters('IntrinsicMatrix', ins2);
    
    % Undistort Images 
    left  = undistortImage(left, cameraParams1); 
    right = undistortImage(right, cameraParams2);

    % Detect Feature Points
    % imagePoints1 = detectMinEigenFeatures(rgb2gray(left), 'MinQuality', 0.1);
    imagePoints1 = detectSURFFeatures(rgb2gray(left), 'MetricThreshold', 1500);

    % Detect Point Correspondences
    tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
    imagePoints1 = imagePoints1.Location;
    initialize(tracker, imagePoints1, left);
    [imagePoints2, validIdx] = step(tracker, right);
    matchedPoints1 = imagePoints1(validIdx, :);
    matchedPoints2 = imagePoints2(validIdx, :);
    figure; showMatchedFeatures(left, right, matchedPoints1, matchedPoints2);

    % Estimate the Fundamental Matrix
%     [fMatrix, epipolarInliers] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'MSAC', 'NumTrials', 10000);
    [fMatrix, epipolarInliers] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'Norm8Point');
    %   [fMatrix, epipolarInliers] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'RANSAC', 'NumTrials', 10000, 'DistanceThreshold', 0.1, 'Confidence', 99.99);
    inlierPoints1 = matchedPoints1(epipolarInliers, :); 
    inlierPoints2 = matchedPoints2(epipolarInliers, :); 

    % Detect More Feature Points
    % imagePoints1 = detectMinEigenFeatures(rgb2gray(left), 'MinQuality', 0.001);
    imagePoints1 = detectSURFFeatures(rgb2gray(left), 'MetricThreshold', 10);
    tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
    imagePoints1 = imagePoints1.Location;
    initialize(tracker, imagePoints1, left);
    [imagePoints2, validIdx] = step(tracker, right);
    matchedPoints1 = imagePoints1(validIdx, :);
    matchedPoints2 = imagePoints2(validIdx, :);
    numPixels = size(left, 1) * size(left, 2);
    allColors = reshape(left, [numPixels, 3]);
    colorIdx = sub2ind([size(left, 1), size(left, 2)], round(matchedPoints1(:,2)), round(matchedPoints1(:, 1)));
    color = allColors(colorIdx, :);

    % Estimate the Camera Pose 
    [R, t] = relativeCameraPose(fMatrix, cameraParams1, cameraParams2, inlierPoints1, inlierPoints2);
    camMatrix1 = cameraMatrix(cameraParams1, eye(3), [0 0 0]);
    camMatrix2 = cameraMatrix(cameraParams2, R',-t*R');

    % 3D Reconstruction
    points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);
    ptCloud = pointCloud(points3D, 'Color', color);
    cameraSize = 0.3;
    figure
    plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
    hold on
    grid on
    plotCamera('Location', t, 'Orientation', R, 'Size', cameraSize, ...
        'Color', 'b', 'Label', '2', 'Opacity', 0);
    pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
        'MarkerSize', 80);
    camorbit(0, 0);
    camzoom(1.5);
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis')
    title('Up to Scale Reconstruction of the Scene');

    % Save Point Cloud
%     pcwrite(ptCloud, outname);
end
