clc
clear
close all

data = load('hehe.txt');


data = data(2:end, :);
[m,n] = size(data);
for i = 1 : m
    time(i) = ((((data(i,4)*60)+data(i,5))*60)+data(i,6));
end
time = time - time(1);
figure
subplot(6,1,1); plot(time, data(:,7),'linewidth',3); ylabel('Temperature'); xlim([0 time(end)]);
subplot(6,1,2); plot(time, data(:,8),'linewidth',3); ylabel('Pressure'); xlim([0 time(end)]);
subplot(6,1,3); plot(time, data(:,9),'linewidth',3); ylabel('Humidity'); xlim([0 time(end)]);
subplot(6,1,4); plot(time, data(:,10),'linewidth',3); ylabel('VOC'); xlim([0 time(end)]);
subplot(6,1,5); plot(time, data(:,11),'linewidth',3); ylabel('Altitude'); xlim([0 time(end)]);
subplot(6,1,6); plot(time, data(:,12),'linewidth',3); ylabel('CO2'); xlim([0 time(end)]);
set(gcf,'outerposition',get(0,'screensize'));

figure
subplot(8,1,1); plot(time, data(:,13),'linewidth',3); ylabel('Light'); xlim([0 time(end)]);
subplot(8,1,2); plot(time, data(:,14),'linewidth',3); ylabel('Rotate'); xlim([0 time(end)]);
subplot(8,1,3); plot(time, data(:,15),'linewidth',3); ylabel('Red'); xlim([0 time(end)]);
subplot(8,1,4); plot(time, data(:,16),'linewidth',3); ylabel('Green'); xlim([0 time(end)]);
subplot(8,1,5); plot(time, data(:,17),'linewidth',3); ylabel('Blue'); xlim([0 time(end)]);
subplot(8,1,6); plot(time, data(:,18),'linewidth',3); ylabel('Violet'); xlim([0 time(end)]);
subplot(8,1,7); plot(time, data(:,19),'linewidth',3); ylabel('Orange'); xlim([0 time(end)]);
subplot(8,1,8); plot(time, data(:,20),'linewidth',3); ylabel('Yellow'); xlim([0 time(end)]);
set(gcf,'outerposition',get(0,'screensize'));