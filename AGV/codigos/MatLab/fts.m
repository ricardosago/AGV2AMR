%%
t = data.t1;
u = data.u1;
y1 = data.y1;
y2 = data.y2;
y3 = data.y3;
y4 = data.y4;
hold all
plot(t, u)
plot(t, y1)
plot(t, y2)
plot(t, y3)
plot(t, y4)
xlabel('Tiempo [s]')
ylabel('Velocidad [RPM]')
title('Toma de datos con encoders')
grid on
%% Para el motor 1
num1 = 2.664045321443494;
den1 = [0.036417810907120 1];
M1s = tf(num1,den1);
hold all
step(27.8*M1s)
plot(t,u)
plot(t, y1)
grid on
title('Motor 1 entrada de 27.8% de PWM')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% Para el motor 2
num2 = 2.664045321443494;
den2 = [0.041417810907120 1];
M2s = tf(num2,den2);
hold all
step(27.8*M2s)
plot(t,u)
plot(t, y2)
grid on
title('Motor 2 entrada de 27.8% de PWM')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% Para el motor 3
num3 = 2.664045321443494;
den3 = [0.041417810907120-0.005 1];
M3s = tf(num3,den3);
hold all
step(27.8*M3s)
plot(t,u)
plot(t, y3)
grid on
title('Motor 3 entrada de 27.8% de PWM')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% Para el motor 4
num4 = 2.664045321443494;
den4 = [0.041417810907120-0.005 1];
M4s = tf(num4,den4);
hold all
step(27.8*M4s)
plot(t,u)
plot(t, y4)
grid on
title('Controladores')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% Control para el motor 1 (0.12 ts y 0.9 csi)
kp1 = 0.227835281266832;
ki1 = 6.256149823170409;
NumC1 = [kp1 ki1];
DenC1 = [1 0];
C1s = tf(NumC1, DenC1)
step(80*feedback(C1s*M1s,1))
hold on
%% Control para el motor 2 (0.12 ts y 0.9 csi)
kp2 = 0.259116030382684;
ki2 = 6.256149823170408;
NumC2 = [kp2 ki2];
DenC2 = [1 0];
C2s = tf(NumC2, DenC2)
step(80*feedback(C2s*M2s,1))
hold on
%% Control para el motor 3 (0.12 ts y 0.9 csi)
kp3 = 0.227835281266832;
ki3 = 6.256149823170409;
NumC3 = [kp3 ki3];
DenC3 = [1 0];
C3s = tf(NumC3, DenC3)
step(80*feedback(C3s*M3s,1))
hold on
%% Control para el motor 4 (0.12 ts y 0.9 csi)
kp4 = 0.227835281266832;
ki4 = 6.256149823170409;
NumC4 = [kp4 ki4];
DenC4 = [1 0];
C4s = tf(NumC4, DenC4)
step(80*feedback(C4s*M4s,1))
hold on
%%
grid on
xlabel('Tiempo')
ylabel('Velocidad RPM')
title('Controladores')