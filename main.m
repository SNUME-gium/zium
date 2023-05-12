clear all; close all; clc

x1=load("/home/b03/Downloads/data.txt");
x2=load("/home/b03/Downloads/data.csv");

x=x1
figure(1);
plot(x(:,1),x(:,2))
hold on;
plot(x(:,1),x(:,2), 'ro')
hold off;

