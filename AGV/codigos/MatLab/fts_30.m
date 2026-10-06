%% graficas
t = Datosfiltrados.tiempo;
u = Datosfiltrados.u3;
m1 = Datosfiltrados.m1_30;
m2 = Datosfiltrados.m2_30;
m3 = Datosfiltrados.m3_30;
m4 = Datosfiltrados.m4_30;
hold all
plot(t, u)
plot(t, m1)
plot(t, m2)
plot(t, m3)
plot(t, m4)
xlabel('Tiempo [s]')
ylabel('Velocidad [RPM]')
title('Toma de datos con encoders')
grid on
%% FT m1
num1 = 7.600033504418935;
den1 = [1,3.196493510851294];
M1s = tf(num1,den1);
hold all
step(30*M1s)
plot(t,u)
plot(t, m1)
grid on
title('Motor 1 entrada de 30% de PWM')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% FT m2
num2 = 7.152100873091166;
den2 = [1,3.353674124712550];
M2s = tf(num2,den2);
hold all
step(30*M2s)
plot(t,u)
plot(t, m2)
grid on
title('Motor 2 entrada de 30% de PWM')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% FT m3
num3 = 5.921479957154323;
den3 = [1,2.537303852659705];
M3s = tf(num3,den3);
hold all
step(30*M3s)
plot(t,u)
plot(t, m3)
grid on
title('Motor 3 entrada de 30% de PWM')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% FT m4
num4 = 6.241879837541462;
den4 = [1,2.656256358411671];
M4s = tf(num4,den4);
hold all
step(30*M4s)
plot(t,u)
plot(t, m4)
grid on
title('Motor 4 entrada de 30% de PWM')
legend('Función de transferencia', 'Entrada [%PWM]','Salida [RPM]', 'location', 'southeast')
xlabel('Tiempo')
%% Control para el motor 1 (0.8 ts y 0.9 csi)
kp1 = 0.328945918270809;
ki1 = 1.051473493173661;
NumC1 = [kp1 ki1];
DenC1 = [1 0];
C1s = tf(NumC1, DenC1)
step(70*feedback(C1s*M1s,1))
hold on
%% Control para el motor 2 (0.6 ts y 0.9 csi)
kp2 = 0.349547642624270;
ki2 = 1.172268884423284;
NumC2 = [kp2 ki2];
DenC2 = [1 0];
C2s = tf(NumC2, DenC2)
step(70*feedback(C2s*M2s,1))
hold on
%% Control para el motor 3 (0.6 ts y 0.9 csi)
kp3 = 0.422191752414783;
ki3 = 1.071228759963182;
NumC3 = [kp3 ki3];
DenC3 = [1 0];
C3s = tf(NumC3, DenC3)
step(70*feedback(C3s*M3s,1))
hold on
%% Control para el motor 4 (0.6 ts y 0.9 csi)
kp4 = 0.400520366470992;
ki4 = 1.063884770111944;
NumC4 = [kp4 ki4];
DenC4 = [1 0];
C4s = tf(NumC4, DenC4)
step(70*feedback(C4s*M4s,1))
hold on
%%
grid on
xlabel('Tiempo')
ylabel('Velocidad RPM')
title('Controladores')