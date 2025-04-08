clear
close all
clc

t = 0:1000;
N = length(t);
y = zeros(1,N);

yOld = y;
time = 1000;
figure(1),
while time > 0
  left_stick_x =  str2double(fileread("left_stick_x"));
  left_stick_y =  str2double(fileread("left_stick_y"));
  right_stick_x = str2double(fileread("right_stick_x"));
  right_stick_y = str2double(fileread("right_stick_y"));
  new_val = left_stick_x;
  y(1:end-1) = yOld(2:end);
  y(end) = new_val;
  figure(1),
  plot(t,y)
  yOld = y;
  pause(0.1);
  drawnow
  time = time-1;
end

