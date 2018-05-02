clear 
close all

data = dlmread('../data/data_t.txt');

data_num = size(data);

row = data(:,1);
col = data(:,2);
angle = data(:,3);

for index = 1: length(angle)
    image(row(index)+1, col(index)+1) = angle(index);
end

row_center = 512;
col_center = 512;

r = sqrt((row - row_center).^2 + (col - col_center).^2);

figure (1)
plot(r, angle, 'b.');
xlabel('image coordinate: pixel');                
ylabel('standard variance: degree'); 

figure (2)
surface(image);
shading interp ;
xlabel('x-axis coordinate: pixel');                
ylabel('y-axis coordinate: pixel'); 
zlabel('standard variance: degree'); 

