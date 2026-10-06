%% Funciones de transferencia de los motores
num = [7.600033504418935, 7.152100873091166, 5.921479957154323, 6.241879837541462];
den = {
    [1,3.196493510851294],
    [1,3.353674124712550],
    [1,2.537303852659705],
    [1,2.656256358411671]
};

% Controladores PI
kp = [0.328945918270809, 0.349547642624270, 0.422191752414783, 0.400520366470992];
ki = [1.051473493173661, 1.172268884423284, 1.071228759963182, 1.063884770111944];

% Tiempo común de simulación
t = 0:0.01:5;  % 0 a 5 segundos, paso de 0.01

% Crear figura con subplots
figure;
for i = 1:4
    subplot(2,2,i)

    % Planta del motor
    G = tf(num(i), den{i});
    
    % Lazo cerrado sin controlador
    G_open = feedback(G, 1);
    
    % Controlador PI
    C = tf([kp(i), ki(i)], [1, 0]);
    
    % Lazo cerrado con controlador
    G_closed = feedback(C * G, 1);

    % Simular respuesta al escalón de 60%
    [y_open, ~] = step(60 * G_open, t);
    [y_closed, ~] = step(60 * G_closed, t);

    % Graficar ambas curvas
    plot(t, y_open, '', 'LineWidth', 1.5); hold on;
    plot(t, y_closed, 'k', 'LineWidth', 1.5); hold off;

    grid on
    title(sprintf('Motor %d - Referencia 60 RPMs', i))
    xlabel('Tiempo [s]')
    ylabel('Velocidad [RPM]')
    legend('Sin controlador', 'Con PI', 'Location', 'southeast')
end

sgtitle('Comparación de lazo cerrado con y sin PI (entrada del 60% PWM)')
