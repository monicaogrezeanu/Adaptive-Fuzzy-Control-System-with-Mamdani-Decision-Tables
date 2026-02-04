
Ts = 0.1;           
Tsim = 100;         
N = Tsim / Ts;      
trials = 30;      
 
P_nom = tf(1, [0.0004 0.28 0.0501]);  
M = tf(1, [1 2 1]);        

t = 0:Ts:Tsim;

gamma = 0.01;

e_count = 11;           
de_count = 7;           
e_points = linspace(-1, 1, e_count);      
de_points = linspace(-1, 1, de_count);    

tabela = zeros(e_count * de_count, 1); 
init_fuzzy = 0;                        

Yplant_all = zeros(trials, length(t));

y_plant_first = [];
y_plant_inter1 = [];
y_plant_inter2 = [];
y_plant_inter3 = [];
y_plant_last = [];

[y_model, ~] = step(M, t);

for trial = 1:trials
    y_process = zeros(1, length(t));
    
    indici_prev = [1, 1];
    ponderi_prev = [1, 0, 0, 0];
    
    for k = 3:length(t)
        e = y_model(k-1) - y_process(k-1);
        de = ((y_model(k-1) - y_process(k-1)) - (y_model(k-2) - y_process(k-2))) / Ts;
        
        e_norm = max(min(e, 1), -1);
        de_norm = max(min(de, 1), -1);
        
        [u_fuzzy, init_fuzzy, tabela, ponderi_out, indici_out] = fuzzyUpdate( ...
            e_norm, de_norm, init_fuzzy, tabela, gamma, indici_prev, ponderi_prev, ...
            e_count, de_count, e_points, de_points, e);
        
        indici_prev = indici_out;
        ponderi_prev = ponderi_out;
        
        K = 0.8;
        T = 20;
        y_process(k) = y_process(k-1) + Ts * (-y_process(k-1)/T + K * u_fuzzy / T);
    end
    
    Yplant_all(trial, :) = y_process;
    
    if trial == 1
        y_plant_first = y_process;
    elseif trial == 5
        y_plant_inter1 = y_process;
    elseif trial == 10
        y_plant_inter2 = y_process;
    elseif trial == 20
        y_plant_inter3 = y_process;
    elseif trial == trials
        y_plant_last = y_process;
    end
end

figure; hold on; grid on;
plot(t, y_model, 'k-', 'LineWidth', 2);
plot(t, y_plant_first, 'r--', 'LineWidth', 1.2);
plot(t, y_plant_inter1, 'g--', 'LineWidth', 1.2);
plot(t, y_plant_inter2, 'm--', 'LineWidth', 1.2);
plot(t, y_plant_inter3, 'c--', 'LineWidth', 1.2);
plot(t, y_plant_last, 'b--', 'LineWidth', 1.2);
title('Raspunsul sistemului la treapta unitara');
legend('Model de referinta', ...
       'Proces - iteratia 1', ...
       'Proces - iteratia 5', ...
       'Proces - iteratia 10', ...
       'Proces - iteratia 20', ...
       'Proces - iteratia 30', ...
       'Location', 'best');
xlabel('Timp [s]'); ylabel('Iesire');
hold off;


function [u, init_out, tabela_out, ponderi_out, indici_out] = fuzzyUpdate(e, de, ...
    init_in, tabela_in, gamma, indici_in, ponderi_in, e_count, de_count, e_points, de_points, epsilon)

    init_out = init_in;
    tabela_out = tabela_in;
    ponderi_out = [];
    indici_out = indici_in;

    if init_in == 0
        init_out = 1;
        tabela_out(:) = 0;
    else
        i_prev = indici_in(1);
        j_prev = indici_in(2);

        idx_11 = (i_prev - 1) * de_count + j_prev;
        idx_21 = (min(i_prev + 1, e_count) - 1) * de_count + j_prev;
        idx_12 = (i_prev - 1) * de_count + min(j_prev + 1, de_count);
        idx_22 = (min(i_prev + 1, e_count) - 1) * de_count + min(j_prev + 1, de_count);

        tabela_out(idx_11) = tabela_out(idx_11) + gamma * ponderi_in(1) * epsilon;
        if i_prev < e_count
            tabela_out(idx_21) = tabela_out(idx_21) + gamma * ponderi_in(2) * epsilon;
        end
        if j_prev < de_count
            tabela_out(idx_12) = tabela_out(idx_12) + gamma * ponderi_in(3) * epsilon;
        end
        if (i_prev < e_count) && (j_prev < de_count)
            tabela_out(idx_22) = tabela_out(idx_22) + gamma * ponderi_in(4) * epsilon;
        end
    end

    i = max(1, min(findClosestIndex(e_points, e), e_count));
    j = max(1, min(findClosestIndex(de_points, de), de_count));
    indici_out = [i, j];

    idx = (i - 1) * de_count + j;

    if i < e_count && j < de_count
        x1 = e_points(i);
        x2 = e_points(i+1);
        y1 = de_points(j);
        y2 = de_points(j+1);

        w_ij = (x2 - e)*(y2 - de)/((x2 - x1)*(y2 - y1));
        w_i1j = (e - x1)*(y2 - de)/((x2 - x1)*(y2 - y1));
        w_ij1 = (x2 - e)*(de - y1)/((x2 - x1)*(y2 - y1));
        w_i1j1 = (e - x1)*(de - y1)/((x2 - x1)*(y2 - y1));

        ponderi_out = [w_ij, w_i1j, w_ij1, w_i1j1];

        u = w_ij * tabela_out(idx) + w_i1j * tabela_out(idx + de_count) + ...
            w_ij1 * tabela_out(idx + 1) + w_i1j1 * tabela_out(idx + de_count + 1);
    elseif i == e_count && j == de_count
        ponderi_out = [1, 0, 0, 0];
        u = tabela_out(idx);
    elseif i == e_count
        w1 = (de_points(j+1) - de) / (de_points(j+1) - de_points(j));
        w2 = 1 - w1;
        ponderi_out = [w1, w2, 0, 0];
        u = w1 * tabela_out(idx) + w2 * tabela_out(idx + 1);
    elseif j == de_count
        w1 = (e_points(i+1) - e) / (e_points(i+1) - e_points(i));
        w2 = 1 - w1;
        ponderi_out = [w1, 0, w2, 0];
        u = w1 * tabela_out(idx) + w2 * tabela_out(idx + de_count);
    else
        ponderi_out = [1, 0, 0, 0];
        u = tabela_out(idx);
    end
end

function idx = findClosestIndex(array, val)
    [~, idx] = min(abs(array - val));
end
