close all; clear; clc;
 %% Main Program 
I1 = imread('sample03/07.png');
I2 = imread('sample03/08.png');
% I1 = imrotate(I1, 90);
% I2 = imrotate(I2, 90);
I1 = rgb2gray(I1); 
I2 = rgb2gray(I2); 

imagePoints1 = detectSURFFeatures(I1, 'MetricThreshold', 10);
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);
figure; showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);

k1 = [[3310.400000 0.000000 316.730000];
      [0.000000 3325.500000 200.550000];
      [0.000000 0.000000 1.000000]];
r1 = [-0.08338944722911310900 0.94609614197942327000 -0.31296611393889306000; 0.97085774118013590000 0.00632080735562845670 -0.23957467325849086000; -0.22468292908270854000 -0.32382424357508255000 -0.91905103946950217000];
t1 = [-0.0218586899664 0.0222092512892 0.631827530638];

k2 = [[3310.400000 0.000000 316.730000];
      [0.000000 3325.500000 200.550000];
      [0.000000 0.000000 1.000000]];
r2 = [0.02769409788772111600 0.94863445285422510000 -0.31515896329964094000; 0.98567438705285304000 0.02655922106302938800 0.16655807969272288000; 0.16637311017687328000 -0.31525749132326808000 -0.93430822517618106000]; 
t2 = [-0.0191341614402 0.032888465352 0.641227584108];

m1 = k1 * [r1 t1'];
m2 = k2 * [r2 t2']; 
x1 = (matchedPoints1(:, 1))'; 
y1 = (matchedPoints1(:, 2))';
x2 = (matchedPoints2(:, 1))';
y2 = (matchedPoints2(:, 2))';

triangulatedPoints = [];
for n = 1:1:length(matchedPoints1) 
    temp = estimateTriangulatedPoints(x1, y1, x2, y2, m1, m2, n);
    triangulatedPoints = [triangulatedPoints; temp];
end

points3D = triangulatedPoints(:, 1:3);
ptCloud = pointCloud(points3D);
figure;
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 80);

pcwrite(ptCloud, 'out0708.ply');

% figure;
% plot3(triangulatedPoints(:, 1), triangulatedPoints(:, 2), triangulatedPoints(:, 3), "b*");
% % origin = plot3(0, 0, 0, 'r*');
% xlabel('x')
% ylabel('y');
% zlabel('z');
% % hold on;
% grid on;
% absPoseL = rigid3d(r1, (-inv(r1)*t1')');
% absPoseR = rigid3d(r2, (-inv(r2)*t2')');
% plotCamera('AbsolutePose', absPoseL, 'Size', 0.01, 'Color', 'r', 'Label', '1', 'Opacity', 0);
% plotCamera('AbsolutePose', absPoseR, 'Size', 0.01, 'Color', 'b', 'Label', '2', 'Opacity', 0);

function res = estimateTriangulatedPoints(x1, y1, x2, y2, ml, mr, n)
    u   = x1(n); 
    v   = y1(n);
    up  = x2(n);
    vp  = y2(n);
    m1  = ml(1, :);
    m2  = ml(2, :);
    m3  = ml(3, :);
    mp1 = mr(1, :);
    mp2 = mr(2, :);
    mp3 = mr(3, :);
    Q   = [(u*m3 - m1); (v*m3 - m2); (up*mp3 - mp1); (vp*mp3 - mp2)];
    
    [U, E, V] = svd(Q, 0);
    V = V'; 
    if V(end) < 0
        V = -1 * V;
    end
    res = V(4, :);
end
