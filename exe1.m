%PARTE A
pkg load control;
pkg load signal;

ft_num = [1];
ft_den = [1 3 3 1];

%---1
G = tf(ft_num, ft_den);

%---2a
figure;
step(G);
title('Resposta ao Degrau em Malha Aberta');
xlabel('Tempo');
ylabel('Amplitude');
grid on;

%---2b
[y, t] = step(G);

final_value = dcgain(G);

L = 0.8066;
T = 3.6946;

printf("Atraso (L): %.4f\n", L);
printf("Constante de tempo (T): %.4f\n", T);

%---2c
Kp = (0.9 * T) / L;
Ki = (0.3 * T) / (L^2);

printf("Ganho proporcional (Kp): %.4f\n", Kp);
printf("Ganho integral (Ki): %.4f\n", Ki);

s = tf("s");
C = Kp + (Ki/s);

closed_ft_pi = feedback(C * G, 1);

figure;
step(closed_ft_pi);
title('Resposta ao Degrau em Malha Fechada com Controlador PI');
xlabel('Tempo');
ylabel('Amplitude');
grid on;

%---3
Kd = 0.6 * L;
printf("Ganho derivativo (Kd): %.4f\n", Kd);
C = Kp + (Ki/s) + (Kd * s);

closed_ft_pid = feedback(C * G, 1);
figure;
step(closed_ft_pid);
title('Resposta ao Degrau em Malha Fechada com Controlador PID');
xlabel('Tempo');
ylabel('Amplitude');
grid on;

%---4

