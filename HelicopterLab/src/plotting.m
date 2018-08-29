
raw = load('../problem2_4.mat');

data = raw.data';
t = data(:,1);
travel = data(:,2);
travel_rate = data(:,3);
pitch = data(:,4);
pitch_rate = data(:,5);

figure();
subplot(5,1,2);
plot(t,travel);
subplot(5,1,3);
plot(t,travel_rate);
subplot(5,1,4);
plot(t,pitch);
subplot(5,1,5);
plot(t,pitch_rate);


