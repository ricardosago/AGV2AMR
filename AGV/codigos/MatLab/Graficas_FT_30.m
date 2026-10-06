%% Preparación de datos
t = Datosfiltrados.tiempo;
u = Datosfiltrados.u3;
m = {Datosfiltrados.m1_30, Datosfiltrados.m2_30, Datosfiltrados.m3_30, Datosfiltrados.m4_30};

numeradores = [7.600033504418935, 7.152100873091166, 5.921479957154323, 6.241879837541462];
denominadores = {
    [1, 3.196493510851294],
    [1, 3.353674124712550],
    [1, 2.537303852659705],
    [1, 2.656256358411671]
};

%% Subplot 2x2 de los 4 motores con sus funciones de transferencia
figure;

for i = 1:4
    subplot(2,2,i)
    
    % Función de transferencia del motor i
    sys = tf(numeradores(i), denominadores{i});
    
    % Respuesta al escalón (30% PWM)
    [y_est, t_est] = step(30 * sys, t);

    % Graficar
    plot(t_est, y_est, 'LineWidth', 2); hold on;
    plot(t, u, '--', 'LineWidth', 1.5);
    plot(t, m{i}, 'k:', 'LineWidth', 2);
    
    % Títulos y etiquetas
    title(sprintf('Motor %d - Entrada 30%% PWM', i));
    xlabel('Tiempo [s]');
    ylabel('Velocidad [RPM]');
    legend('FT estimada', 'Entrada [%PWM]', 'Salida real [RPM]', 'Location', 'southeast');
    xlim([0 5]); % <-- Límites del eje X
    grid on;
end

sgtitle('Comparación entrada, salida real y FT para los 4 motores (0 a 5 s)');
