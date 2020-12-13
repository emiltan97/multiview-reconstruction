close all; 
clear;
clc;
% Taking Input Images
left  = input('data/left.jpg'); 
right = input('data/right.jpg'); 
% Convert the images into Gray
grayL = gray(left); 
grayR = gray(right); 
% Finding point correspondences using HARRIS Feature Descriptor
[harrisPts1, harrisPts2] = harris(grayL, grayR); 
% Finding point correspondences using SURF Feature Descriptor 
% [surfPts1, surfPts2] = surf(grayL, grayR);
% Estimate the Fundamental Matrix using 8 Point Algorithm 
[fMatrix, inlierPoints1, inlierPoints2] = eightPoint(harrisPts1, harrisPts2);

function [fMatrix, inlierPoints1, inlierPoints2] = eightPoint(matchedPoints1, matchedPoints2) 
    [fMatrix, epipolarInliers] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'Norm8Point');

    inlierPoints1 = matchedPoints1(epipolarInliers, :); 
    inlierPoints2 = matchedPoints2(epipolarInliers, :); 
end

function [matchedPoints1, matchedPoints2] = surf(grayL, grayR) 
    points1 = detectSURFFeatures(grayL);
    points2 = detectSURFFeatures(grayR); 
    
    [f1, vpts1] = extractFeatures(grayL, points1); 
    [f2, vpts2] = extractFeatures(grayR, points2); 
    
    indexPairs = matchFeatures(f1, f2); 
    matchedPoints1 = vpts1(indexPairs(:, 1)); 
    matchedPoints2 = vpts2(indexPairs(:, 2)); 
    
    figure; 
    showMatchedFeatures(grayL, grayR, matchedPoints1, matchedPoints2); 
end

function [matchedPoints1, matchedPoints2] = harris(grayL, grayR)
    points1 = detectHarrisFeatures(grayL);
    points2 = detectHarrisFeatures(grayR); 

    [features1,valid_points1] = extractFeatures(grayL,points1);
    [features2,valid_points2] = extractFeatures(grayR,points2);

    indexPairs = matchFeatures(features1,features2);

    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);

    figure; 
    showMatchedFeatures(grayL, grayR, matchedPoints1, matchedPoints2); 
end

function res = gray(name) 
    res = rgb2gray(name); 
    figure;
    imshow(res); 
end

function res = input(name)
    res = imread(name);
    figure; 
    imshow(res); 
end