A=load("wide\wide.txt");
[m,n]=size(A);

time=A(:,2);
Accl=A(:,3:5);
accl=sqrt(Accl(:,1).*Accl(:,1)+Accl(:,2).*Accl(:,2)+Accl(:,3).*Accl(:,3));
Gyro=A(:,6:8);
gyro=sqrt(Gyro(:,1).*Gyro(:,1)+Gyro(:,2).*Gyro(:,2)+Gyro(:,3).*Gyro(:,3));


figure(1);
subplot(2,1,1);
plot(time,accl,'.');
title("Accl");
xlabel("time");
ylabel("m/s");
grid on;
subplot(2,1,2);
plot(time,gyro,'.');
title("Gyro");
xlabel("time");
ylabel("rad");
grid on;

