close all; clear; clc; 

i1 = input('mydata02/01.png');
i2 = input('mydata02/02.png');
i3 = input('mydata02/03.png');
i4 = input('mydata02/04.png');
i5 = input('mydata02/05.png');
i6 = input('mydata02/06.png');
i7 = input('mydata02/07.png');
i8 = input('mydata02/08.png');
i9 = input('mydata02/09.png');
i10 = input('mydata02/10.png');
i11 = input('mydata02/11.png');
i12 = input('mydata02/12.png');
i13 = input('mydata02/13.png');
i14 = input('mydata02/14.png');
i15 = input('mydata02/15.png');


function res = input(filename) 
res = imread(filename);
figure; 
imshow(res); 
end