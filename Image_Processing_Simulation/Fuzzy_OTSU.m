clear;

test = imread('D:\智能车\2022华东赛\图像处理仿真\test.jpg');
figure(1);
imshow(test);
figure(2)
image1=rgb2gray(test);
image2=imresize(image1,[120 188]);
imshow(image2);

[height,width]=size(image2);
GrayScale=256;N=120*188;
Histogram=zeros(1,256);
threshold=0;

L1=130;L2=150;L3=0;L4=240;
uA=zeros(1,L2);uB=zeros(1,L4);uC=zeros(1,GrayScale);
PA=0;PB=0;PC=0;
PAA=zeros(1,L2);PBB=zeros(1,L4);PCC=zeros(1,GrayScale);

mA=0;mB=0;mC=0;
S=0;preS=0;

gray1=0;gray2=0;

temp_point=zeros(1,256);temp_k=0;temp_i=0;Break_Point=0;

temp1=0;temp2=0;

for i=1:height
    for j=1:width
        if(image2(i,j)==0)
            image2(i,j)=1;
        end
        Histogram(image2(i,j))=Histogram(image2(i,j))+1;
    end
end


for i=2:GrayScale
    if Histogram(i)>=Histogram(i-1)
        temp_point(i)=temp_point(i-1)+1;
    else
        temp_point(i)=temp_point(i-1)-1;
    end
end

for i=11:1:GrayScale-11
    if temp_point(i)<temp_point(i+10)&&temp_point(i)<temp_point(i-10)
        temp_i=temp_i+i;
        temp_k=temp_k+1;
    end
end

Break_Point=round(temp_i/temp_k);

for i=1:Break_Point
    if (temp1<Histogram(i))
        temp1=Histogram(i);
        gray1=i;
    end
end

for i=Break_Point:240
    if (temp2<Histogram(i))
        temp2=Histogram(i);
        gray2=i;
    end
end

L1=gray1;
L2=gray1+30;
L4=gray2;
if(L4<=L2)
    L4=L2+20;
end


for i=1:L2
    if(i<=L1)
        uA(i)=1;
    elseif(i>L1&&i<=L2)
        uA(i)=(i/(L1-L2))-(L2/(L1-L2));
    end
    PA=PA + uA(i)*Histogram(i);
end


for i=1:L2
    PAA(i)=uA(i)*Histogram(i)/PA;
    mA = mA+i*PAA(i);
end


PA = PA/N;

for L3=L2:L4
    mB=0;mC=0;PB=0;PC=0;
    for i=L1:L4
        if(i>=L1&&i<L2)
            uB(i)=(i/(L2-L1))-(L1/(L2-L1));
        elseif(i>=L2&&i<=L3)
            uB(i)=1;
        elseif(i>L3&&i<=L4)
            uB(i)=(i/(L3-L4))-(L4/(L3-L4));
        end
    end
    PB = PB + uB(i)*Histogram(i);
end


for i=L3:GrayScale
    if(i<L4&&i>=L3)
        uC(i)=(i/(L4-L30))-(L3/(L4-L3));
    elseif(i>=L4)
        uC(i)=1;
    end
    PC = PC + uC(i)*Histogram(i);
end


for i = L1:L4
    PBB(i)=uB(i)*Histogram(i)/PB;
    mB=mB+i*PBB(i);
end


for i=L3:GrayScale
    PCC(i)=uC(i)*Histogram(i)/PC;
    mC=mC+i*PCC(i);
end

PB = PB/N;
PC = PC/N;

S = -PA*PB*PC*(mC-mB)*(mC-mA)*(mB-mA);

if(S > preS)
    preS = S;
    threshold=(L1+L2+L3)/3;
end
        
image3 = im2bw(image2,threshold/255);
figure(3);
imshow(image3);        