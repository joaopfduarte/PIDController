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
C_pi = Kp + (Ki/s);

closed_ft_pi = feedback(C_pi * G, 1);

figure;
step(closed_ft_pi);
title('Resposta ao Degrau em Malha Fechada com Controlador PI');
xlabel('Tempo');
ylabel('Amplitude');
grid on;

%---3
Kd = 0.6 * L;
printf("Ganho derivativo (Kd): %.4f\n", Kd);
C_pid = Kp + (Ki/s) + (Kd * s);

closed_ft_pid = feedback(C_pid * G, 1);
figure;
step(closed_ft_pid);
title('Resposta ao Degrau em Malha Fechada com Controlador PID');
xlabel('Tempo');
ylabel('Amplitude');
grid on;

%---4
closed_ft = feedback(G, 1);
closed_ft_pi = feedback(C_pi * G, 1);
closed_ft_pid = feedback(C_pid * G, 1);

figure;
step(closed_ft, 'b', closed_ft_pi, 'r', closed_ft_pid, 'g');
title('Resposta ao Degrau em Malha Fechada');
xlabel('Tempo');
ylabel('Amplitude');
legend('Sem controlador', 'Com controlador PI', 'Com controlador PID');
grid on;

%---5
function [Mp, tr, ts, ess] = calculate_performance(y,t)
  final_value = y(end);
  setpoint = 1;

    % Máximo sobressinal (Mp)
    Mp = (max(y) - final_value) / final_value* 100;

    % Tempo de subida (tr)
    idx_10 = find(y >= 0.1 * final_value, 1);
    idx_90 = find(y >= 0.9 * final_value, 1);
    tr = t(idx_90) - t(idx_10);

    % Tempo de acomodação (ts)
    idx_ts = find(abs(y - final_value) >= 0.02 * final_value, 1, 'last');
    ts = t(idx_ts);

    % Erro de estado estacionário (ess)
    ess = setpoint - final_value;
end

[y1, t1] = step(closed_ft);
[y2, t2] = step(closed_ft_pi);
[y3, t3] = step(closed_ft_pid);

[Mp1, tr1, ts1, ess1] = calculate_performance(y1, t1);
[Mp2, tr2, ts2, ess2] = calculate_performance(y2, t2);
[Mp3, tr3, ts3, ess3] = calculate_performance(y3, t3);

disp("----------------------------");
printf("Sem controlador:\n");
printf("  Mp: %.2f%%\n", Mp1);
printf("  tr: %.4f s\n", tr1);
printf("  ts: %.4f s\n", ts1);
printf("  ess: %.4f\n", ess1);

disp("-------------------");
printf("Com controlador PI:\n");
printf("  Mp: %.2f%%\n", Mp2);
printf("  tr: %.4f s\n", tr2);
printf("  ts: %.4f s\n", ts2);
printf("  ess: %.4f\n", ess2);

disp("-------------------");
printf("Com controlador PID:\n");
printf("  Mp: %.2f%%\n", Mp3);
printf("  tr: %.4f s\n", tr3);
printf("  ts: %.4f s\n", ts3);
printf("  ess: %.4f\n", ess3);

