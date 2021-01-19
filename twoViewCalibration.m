close all;
clear; 
clc;
%% Main Program
left  = imread('mydata03/l3.png');
right = imread('mydata03/r3.png');
% ********************* Coordinates for l3 ***************************** %
xl = [213; 242; 389; 385; 457; 455; 329; 326; 243; 240];
yl = [442; 475; 515; 633; 663; 773; 692; 818; 520; 659];
% ********************* Coordinates for r3 ***************************** %
xr = [264; 290; 446; 441; 520; 517; 399; 396; 309; 305];
yr = [456; 488; 529; 648; 678; 792; 701; 833; 531; 671];
% ********************* Real World Coordinates ************************* %
P = [
        [0.021 0.042 0.084 0.084 0.084 0.084 -0.002 -0.002 -0.017 -0.017];
        [-0.021 -0.042 -0.063 -0.136 -0.136 -0.198 -0.136 -0.198 -0.063 -0.136]; 
        [0 0 0.109 0.109 0.168 0.168 0.168 0.168 0.109 0.109];
        [1 1 1 1 1 1 1 1 1 1];
];

showImageWithPoints(left, xl, yl);
% showImageWithPoints(right, xr, yr);

ml = estimateProjectionMatrix(xl, yl, P);
mr = estimateProjectionMatrix(xr, yr, P);

[kl, rl, tl] = estimateInsExParams(ml);
[kr, rr, tr] = estimateInsExParams(mr);

% Show the position of the camera in respect to the scene 
figure;
plot3(P(1, :), P(2, :), P(3, :), 'ro');
hold on;
origin = plot3(0, 0, 0, 'r*');
xlabel('x')
ylabel('y');
zlabel('z');
hold on;
grid on;
absPoseL = rigid3d(rl, (-inv(rl)*tl)');
absPoseR = rigid3d(rr, (-inv(rr)*tr)');
plotCamera('AbsolutePose', absPoseL, 'Size', 0.01, 'Color', 'r', 'Label', '1', 'Opacity', 0);
plotCamera('AbsolutePose', absPoseR, 'Size', 0.01, 'Color', 'b', 'Label', '2', 'Opacity', 0);

% Reprojecting the points using the projection matrix
% reprojectPoints(left, ml, kl, P);
% reprojectPoints(right, mr, kr, P); 

% Reprojecttion the points in 3D
triangulatedPoints = [];
for n = 1:1:10 
    temp = estimateTriangulatedPoints(xl, yl, xr, yr, ml, mr, n);
    triangulatedPoints = [triangulatedPoints; temp];
end
plot3(triangulatedPoints(:, 1), triangulatedPoints(:, 2), triangulatedPoints(:, 3), "b*");

f = estimateFundamentalMatrix(xl, yl, xr, yr);
% e = kl * f * kr;
   
%% All the functions
function showImageWithPoints(img, x, y) 
    figure; 
    imshow(img); 
    hold on; 
    plot(x, y, 'ro');
end

function res = estimateProjectionMatrix(x, y, P)
    % for loop to construct the Q matrix 
    Q = [];
    for n = 1: 1: 10
        Pk = P(:, n); % 4x1 vec
        zeroMat= zeros(size(Pk)); % 4x1 0
        xk = x(n); % x coordinate of a point in terms of pixels 
        yk = y(n); % y coordinate of a point in terms of pixels
        Qk = [Pk' zeroMat' -xk*Pk' ; zeroMat' Pk' -yk*Pk'];
        
        Q = [Q; Qk]; % stacking vertically
    end
    % SVD mode 2
    [U, E, V] = svd(Q, 0);
    % The projection matrix 
    % Converting the vector v12 into a 3x4 matrix
    V = V';
    res = [[V(12, 1), V(12, 2), V(12, 3),  V(12,4)];
           [V(12, 5), V(12, 6), V(12, 7),  V(12,8)];
           [V(12, 9), V(12,10), V(12, 11), V(12, 12)]];
end

function [K, R, t] = estimateInsExParams(M)
    a1    = M(1,1:end-1)'; 
    a2    = M(2,1:end-1)';
    a3    = M(3,1:end-1)';
    rho   = 1 / norm(a3);
    r3    = rho*a3; % the third row of rotation matrix 
    u0    = (rho^2)*(dot(a1,a3)); % the camera center in terms of x axis
    v0    = (rho^2)*(dot(a2,a3)); % the camera center in terms of y axis 
    theta = acos(-((dot(cross(a1, a3), cross(a2, a3))) / ((norm(cross(a1, a3))) * (norm(cross(a2, a3))))));
    alpha = (rho^2)*(norm(cross(a1, a3))) * sin(theta); 
    beta  = (rho^2)*(norm(cross(a2, a3))) * sin(theta);
    r1    = (1/(norm(cross(a2,a3)))) * (cross(a2,a3)); % the first row of the rotation matrix
    r2    = cross(r3, r1); % the second row of the rotation matrix
    b     = M(:, 4); 
    K     = [[alpha -alpha*cot(theta) u0];
             [0 beta/sin(theta) v0];
             [0 0 1]];
    t     = rho * inv(K) * b;
    R     = [r1'; r2'; r3'];
%     f     = alpha / 1429;
end

function reprojectPoints(img, M, K, P)
    m1 = M(1, :);
    m2 = M(2, :);
    m3 = M(3, :);
    u0 = K(1,3);
    v0 = K(2,3);
    u  = [];
    v  = [];
    for n = 1: 1: 10
        uprime = (m1 * P(:, n)) / (m3 * P(:, n));
        vprime = (m2 * P(:, n)) / (m3 * P(:, n));
        u = [u; uprime]; 
        v = [v; vprime];
    end
    figure; 
    imshow(img); 
    hold on; 
    plot(u0, v0, 'c*');
    plot(u, v, 'r*');
end

function res = estimateFundamentalMatrix(x1, y1, x2, y2) 
    u = -1 * ones(length(x1));
    X = [x1.*x2 x1.*y2 x1 y1.*x2 y1.*y2 y1 x2 y2];
    f = X \ u;
    
    res = [[f(1, 1), f(2, 1), f(3, 1)];
           [f(4, 1), f(5, 1), f(6, 1)];
           [f(7, 1), f(8, 1), 1      ]];
end

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