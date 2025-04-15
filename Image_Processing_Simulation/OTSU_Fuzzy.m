clear;

test = imread('D:\智能车\2022华东赛\图像处理仿真\test.jpg');
figure(1);
imshow(test);
figure(2)
image1=rgb2gray(test);
image2=imresize(image1,[120 188]);
imshow(image2);


X = 0;
Y = 0;
First = 0;Last = 0;
Treshold = -1;
BestEntropy = 0;

for First = 1:1