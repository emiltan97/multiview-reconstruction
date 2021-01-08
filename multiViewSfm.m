close all; clear; clc; 

leftname  = 'sample03/03.png';
rightname = 'sample03/04.png';
rotationMatrix1 = [-0.28437648559126744000 -0.30447985736374050000 -0.90907810612855156000;
    0.95818717180552493000 -0.05887271324279850400 -0.28002026823733883000; 
    0.03174117685280887500 -0.95069715013877931000 0.30848995461910772000];
rotationMatrix2 = [-0.29401781384102649000 0.10008332165785941000 -0.95054606727387558000;
    0.95254363718061841000 -0.05134047099598639700 -0.30004129466721602000;
    -0.07882994806514329000 -0.99365293826058598000 -0.08023903828591291700];
translationVector1 = [-0.0181396767412 -0.0277593369636 0.645855774902];
translationVector2 = [-0.021187474226 -0.0187242689354 0.634951273531];

outname = 'out0304.ply';

main(leftname, rightname, rotationMatrix1, rotationMatrix2, translationVector1, translationVector2, outname);

function main(leftname, rightname, rotationMatrix1, rotationMatrix2, translationVector1, translationVector2, outname) 
    left  = imread(leftname);
    right = imread(rightname);
    
    kernel1 = strel('square', 10);
    kernel2 = strel('square', 7);
    
    left = threshold(rgb2gray(left), 49);
    left = imcomplement(left);
    left = dilate(left, kernel1);
    left = erode(left, kernel2);
    right = threshold(rgb2gray(right), 49);
    right = imcomplement(right);
    right = dilate(right, kernel1);
    right = erode(right, kernel2);
    
    imagePoints1 = detectSURFFeatures(left, 'MetricThreshold', 10);
    imagePoints2 = detectSURFFeatures(right, 'MetricThreshold', 10);
    [features1,valid_points1] = extractFeatures(left,imagePoints1);
    [features2,valid_points2] = extractFeatures(right,imagePoints2);
    indexPairs = matchFeatures(features1,features2);
    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
    figure; showMatchedFeatures(left,right,matchedPoints1,matchedPoints2);
    legend('matched points 1','matched points 2');
%     tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
%     imagePoints1 = imagePoints1.Location;
%     initialize(tracker, imagePoints1, left);
%     [imagePoints2, validIdx] = step(tracker, right);
%     matchedPoints1 = imagePoints1(validIdx, :);
%     matchedPoints2 = imagePoints2(validIdx, :);
%     figure; showMatchedFeatures(left, right, matchedPoints1, matchedPoints2);
    
    ins1 = [3310.4 0 0; 0 3325.5 0; 316.73 200.55 1];
    ins2 = [3310.4 0 0; 0 3325.5 0; 316.73 200.55 1];
    cameraParams1 = cameraParameters('IntrinsicMatrix', ins1);
    cameraParams2 = cameraParameters('IntrinsicMatrix', ins2);
    camMatrix1 = cameraMatrix(cameraParams1, rotationMatrix1, translationVector1);
    camMatrix2 = cameraMatrix(cameraParams2, rotationMatrix2, translationVector2);
    
    pts3d = triangulate(matchedPoints1, matchedPoints1, camMatrix1, camMatrix2);
    ptCloud = pointCloud(pts3d);
    figure;
    pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 80);

    pcwrite(ptCloud, outname);
end
 
function res = threshold(img, val) 
    res = img < val; 
    figure;
    imshow(res);
end

function res = erode(img, kernel) 
    res = imerode(img, kernel); 
    figure; 
    imshow(res);
end 

function res = dilate(img, kernel)
    res = imdilate(img, kernel); 
    figure; 
    imshow(res);
end

function res = opening(img, kernel)
    res = imopen(img, kernel); 
    figure; 
    imshow(res); 
end 

function res = closing(img, kernel) 
    res = imclose(img, kernel); 
    figure; 
    imshow(res); 
end