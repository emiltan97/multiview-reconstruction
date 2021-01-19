close all; clear; clc;

left  = imread('mydata03/l3.png');
right = imread('mydata03/r3.png');

[p1, p2, p3] = select3NoncollinearPoints(left);
[q1, q2, q3] = select3NoncollinearPoints(right);

[a, b, c, d, e, f] = solveParameters(p1, p2, p3, q1, q2, q3);

y = (q1(2) - f + (d*c / a) - (d*q1(1) / a)) / (e - (d*b / a));
x = (q1(1) - b*y - c) / a;

% function verifyDistance() 
%     
% end

function [a, b, c, d, e, f] = solveParameters(p1, p2, p3, q1, q2, q3)
    A = [[p1(1) p1(2) 1];
         [p2(1) p2(2) 1];
         [p3(1) p3(2) 1]];
    B = [[q1(1) q1(2)];
         [q2(1) q2(2)];
         [q3(1) q3(2)]];
    x = inv(A) * B;
    
    a = x(1, 1); 
    b = x(2, 1); 
    c = x(3, 1); 
    d = x(1, 2); 
    e = x(2, 2); 
    f = x(3, 2); 
end

function [p1, p2, p3] = select3NoncollinearPoints(img)
    [rows, columns, dimensions] = size(img); 
    x = floor(rand(1, 3) * rows) + 1; 
    y = floor(rand(1, 3) * columns) + 1; 
    mat = [[1 x(1) y(1)];
           [1 x(2) y(2)];
           [1 x(3) y(3)]];
    while det(mat) == 0 
        x = floor(rand(1, 3) * numel(rows)) + 1; 
        y = floor(rand(1, 3) * numel(columns)) + 1;
        mat = [[1 x(1) y(1)];
               [1 x(2) y(2)];
               [1 x(3) y(3)]];
    end
    p1 = [x(1) y(1)];
    p2 = [x(2) y(2)];
    p3 = [x(3) y(3)];
end 