clear;
tic
test = imread('D:\智能车\2022华东赛\图像处理仿真\test1.jpg');
figure(1);
imshow(test);
figure(2)
image1=rgb2gray(test);
image2=imresize(image1,[120 188]);
imshow(image2);

image3 = image2;

for i = 1:120
   for j = 1:188
       if(image2(i,j) < 205)
           image3(i,j) = 0;
       else
           image3(i,j) = 255;
       end
   end
end

% image3 = imbinarize(image2);
figure(3);
imshow(image3);

type = 1;
type_sort = zeros(100,100);
sort_Image = zeros(120,188);
temp = zeros(1,4);
last_min = 0;

for i = 2:119
   for j = 2:187
       if(image3(i,j)==255)
           temp(1,1) = sort_Image(i-1,j);
           temp(1,2) = sort_Image(i+1,j);
           temp(1,3) = sort_Image(i,j-1);
           temp(1,4) = sort_Image(i,j+1);
           B = sort(temp,2);
           if(temp(1,1) == 0 && temp(1,2) == 0 && temp(1,3) == 0 && temp(1,4) == 0)
               sort_Image(i,j) = type;
               type = type + 1;
           else
               for k = 1:4
                   if(B(1,k) == 0)
                       continue;
                   else
                       min = B(1,k);
                       break;
                   end
               end
               sort_Image(i,j) = min;    
           end
           if sort_Image(i,j) ~= sort_Image(i,j-1) && sort_Image(i,j-1) ~= 0 && ~ismember(sort_Image(i,j-1),type_sort)
               type_sort(sort_Image(i,j),100) = type_sort(sort_Image(i,j),100) + 1;
               type_sort(sort_Image(i,j),type_sort(sort_Image(i,j),100)) = sort_Image(i,j-1);
           end
           if sort_Image(i,j) ~= sort_Image(i,j+1) && sort_Image(i,j+1) ~= 0 && ~ismember(sort_Image(i,j+1),type_sort)
               type_sort(sort_Image(i,j),100) = type_sort(sort_Image(i,j),100) + 1;
               type_sort(sort_Image(i,j),type_sort(sort_Image(i,j),100)) = sort_Image(i,j+1);
           end    
           if sort_Image(i,j) ~= sort_Image(i-1,j) && sort_Image(i-1,j) ~= 0 && ~ismember(sort_Image(i-1,j),type_sort)
               type_sort(sort_Image(i,j),100) = type_sort(sort_Image(i,j),100) + 1;
               type_sort(sort_Image(i,j),type_sort(sort_Image(i,j),100)) = sort_Image(i-1,j);
           end    
           if sort_Image(i,j) ~= sort_Image(i+1,j) && sort_Image(i+1,j) ~= 0 && ~ismember(sort_Image(i+1,j),type_sort)
               type_sort(sort_Image(i,j),100) = type_sort(sort_Image(i,j),100) + 1;
               type_sort(sort_Image(i,j),type_sort(sort_Image(i,j),100)) = sort_Image(i+1,j);
           end    
       end
   end
end
           
           
for i = 1:100
    for j = 1:100
       if type_sort(i,j) ~= 0
           for k = 1:100
               if type_sort(type_sort(i,j),k) ~= 0
                   type_sort(i,100) = type_sort(i,100) + 1;
                   type_sort(i,type_sort(i,100)) = type_sort(type_sort(i,j),k);
               else
                   break;
               end
           end
       else
           break;
       end
    end
end


for i = 1:120
    for j = 1:188
        for k = 1:100
            TYPE_SORTK = type_sort(k,:);
            if ismember(sort_Image(i,j),TYPE_SORTK) && sort_Image(i,j) ~= 0
                sort_Image(i,j) = k;
                break;
            end    
        end
    end
end


for i = 1:120
    for j = 1:188
        sort_Image(i,j) = sort_Image(i,j)*8;
    end
end
toc

mytime = toc;
mytime
figure(4);
image4 = gray2color(sort_Image);
imshow(image4);