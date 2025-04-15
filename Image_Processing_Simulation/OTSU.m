clear;

test = imread('D:\智能车\2022华东赛\图像处理仿真\test.jpg');
figure(1);
imshow(test);
figure(2)
image1=rgb2gray(test);
image2=imresize(image1,[120 188]);
imshow(image2);

sum = 188*120;
num = zeros(1,256);
possi = zeros(1,256);
PA = 0;
PB = 0;
PC = 0;
mA = 0;
mB = 0;
mC = 0;

for i=1:188
   for j=1:120
       gray = image2(j,i);
       num(1,gray+1) = num(1,gray+1) + 1;
   end
end

for i=1:80
    temp = OTSU_fun1(i);
    mA = mA + i*num(1,i)*temp;
    PA = PA + num(1,i)*temp;
end
mA = mA/PA;
PA = PA/sum;

for i=81:220
    temp = OTSU_fun2(i);
    mB = mB + i*num(1,i)*temp;
    PB = PB + num(1,i)*temp;
end
mB = mB/PB;
PB = PB/sum;

for i=180:256
    temp = OTSU_fun3(i);
    mC = mC + i*num(1,i)*temp;
    PC = PC + num(1,i)*temp;
end
mC = mC/PC;
PC = PC/sum;

S = PA*PB*PC*(mC-mB)*(mC-mA)*(mB-mA);
thresholds = (mC)/2;
image3 = im2bw(image2,thresholds/255);
figure(3);
imshow(image3);





