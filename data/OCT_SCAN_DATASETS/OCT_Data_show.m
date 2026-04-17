clear all
clear
close all
clc

OCT_data_num = 1;   % Choose from 5 datasets, 1, 2, 3, 4, or 5
% OCT_data_num = 2;   % Choose from 5 datasets, 1, 2, 3, 4, or 5
% OCT_data_num = 3;   % Choose from 5 datasets, 1, 2, 3, 4, or 5
% OCT_data_num = 4;   % Choose from 5 datasets, 1, 2, 3, 4, or 5
% OCT_data_num = 5;   % Choose from 5 datasets, 1, 2, 3, 4, or 5

File_path = ['./cornea_data_',num2str(OCT_data_num),'.mat'];  % Please change this path for your case


dx = 0.01/255;
dz = 0.004593E-3;





load(File_path);


OCT_mag_img = log10(abs(cornea_oct_img));   % Magnitude image for surface detection


X_vec_mm = dx*(0:size(OCT_mag_img,2)-1)*1000;

Z_vec_mm = dz*(0:size(OCT_mag_img,1)-1)*1000;


figure
imagesc(X_vec_mm, Z_vec_mm, OCT_mag_img);
xlim([X_vec_mm(1) X_vec_mm(end)]);
ylim([Z_vec_mm(1) Z_vec_mm(end)]);
colormap jet
colorbar;
title('Log abs OCT Image')
xlabel('X (mm)');
ylabel('Z (mm)');
axis image

