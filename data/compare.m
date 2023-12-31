pos=load("wide.pos");
A=load("wide\\wide.sins");
[m,n]=size(A);
[p,q]=size(pos);
%自己解算
T=A(:,1);
B=A(:,2);
L=A(:,3);
H=A(:,4);
VN=A(:,5);
VE=A(:,6);
VD=A(:,7);
Yaw=A(:,8);
Pitch=A(:,9);
Roll=A(:,10);
E=A(:,11);
N=A(:,12);
U=A(:,13);
X=A(:,14);
Y=A(:,15);
Z=A(:,16);

%POSMind解算
Tp=pos(:,2);
Bp=pos(:,3);
Lp=pos(:,4);
Hp=pos(:,5);

VEp=pos(:,6);
VNp=pos(:,7);
VDp=pos(:,8);

yaw=pos(:,9);
pitch=pos(:,10);
roll=pos(:,11);
Ep=pos(:,12);
Np=pos(:,13);
Up=pos(:,14);
Xp=pos(:,15);
Yp=pos(:,16);
Zp=pos(:,17);

%索引 作差
index=find(T==Tp(1));
dyaw=Yaw(index:index+p-1)-yaw;
dpitch=Pitch(index:index+p-1)-pitch;
droll=Roll(index:index+p-1)-roll;
dB=B(index:index+p-1)-Bp;
dL=L(index:index+p-1)-Lp;
dH=H(index:index+p-1)-Hp;
dVN=VN(index:index+p-1)-VNp;
dVE=VE(index:index+p-1)-VEp;
dVD=VD(index:index+p-1)+VDp;
dE=E(index:index+p-1)-Ep;
dN=N(index:index+p-1)-Np;
dU=U(index:index+p-1)-Up;
dX=X(index:index+p-1)-Xp;
dY=Y(index:index+p-1)-Yp;
dZ=Z(index:index+p-1)-Zp;

figure(1);
subplot(3,1,1);
plot(Tp,dyaw,'.');
title("dYaw");
xlabel("time");
ylabel("°");
grid on;
subplot(3,1,2);
plot(Tp,dpitch,'.');
title("dPitch");
xlabel("time");
ylabel("°");
grid on;
subplot(3,1,3);
plot(Tp,droll,'.');
title("dRoll");
xlabel("time");
ylabel("°");
grid on;


figure(2);
subplot(3,1,1);
plot(Tp,dB,'.');
title("dB");
xlabel("time");
ylabel("°");
grid on;
subplot(3,1,2);
plot(Tp,dL,'.');
title("dL");
xlabel("time");
ylabel("°");
grid on;
subplot(3,1,3);
plot(Tp,dH,'.');
title("dH");
xlabel("time");
ylabel("m");
grid on;

figure(3);
subplot(3,1,1);
plot(Tp,dVN,'.');
title("dVN");
xlabel("time");
ylabel("m/s");
grid on;
subplot(3,1,2);
plot(Tp,dVE,'.');
title("dVE");
xlabel("time");
ylabel("m/s");
grid on;
subplot(3,1,3);
plot(Tp,dVD,'.');
title("dVD");
xlabel("time");
ylabel("m/s");
grid on;

% figure(4);
% subplot(3,1,1);
% plot(Tp,dE,'.');
% title("dE");
% xlabel("time");
% ylabel("m");
% grid on;
% subplot(3,1,2);
% plot(Tp,dN,'.');
% title("dN");
% xlabel("time");
% ylabel("m");
% grid on;
% subplot(3,1,3);
% plot(Tp,dU,'.');
% title("dU");
% xlabel("time");
% ylabel("m");
% grid on;

figure(5);
subplot(3,1,1);
plot(Tp,dX,'.');
title("dX");
xlabel("time");
ylabel("m");
grid on;
subplot(3,1,2);
plot(Tp,dY,'.');
title("dY");
xlabel("time");
ylabel("m");
grid on;
subplot(3,1,3);
plot(Tp,dZ,'.');
title("dZ");
xlabel("time");
ylabel("m");
grid on;