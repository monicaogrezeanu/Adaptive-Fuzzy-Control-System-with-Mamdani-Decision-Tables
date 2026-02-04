clear
clc

%Configurare parametri 
gamma_local = 0.01;
e_count_local = 11; 
de_count_local = 7;  

e_range = [-1, 1];
de_range = [-1, 1];

e_step = (e_range(2) - e_range(1)) / (e_count_local - 1);
de_step = (de_range(2) - de_range(1)) / (de_count_local - 1);

e_points_local = e_range(1):e_step:e_range(2)
de_points_local = de_range(1):de_step:de_range(2)

% Creare obiecte de tip Simulink Parameter
e_count = Simulink.Parameter(e_count_local);
de_count = Simulink.Parameter(de_count_local);

e_points = Simulink.Parameter(e_points_local);
de_points = Simulink.Parameter(de_points_local);

dim_tabela_local = e_count_local*de_count_local;
dim_tabela = Simulink.Parameter(dim_tabela_local);
gamma = Simulink.Parameter(gamma_local);

% Salvarea parametrilor in workspace
assignin('base', 'e_count', e_count);
assignin('base', 'de_count', de_count);
assignin('base', 'e_points', e_points);
assignin('base', 'de_points', de_points);
assignin('base', 'dim_tabela', dim_tabela);
assignin('base', 'gamma', gamma);

% Incarcarea si rularea cronometrata a modelului
model = 'regulator_fuzzy_adaptiv';
load_system(model);
tic;
sim(model);
timp = toc;

% Afisarea tabelei Mamdani de la ultima iteratie, a timpului si a raspunsului
TabelaMamdaniFinala = reshape(MamdaniFinal, e_count_local, de_count_local);

disp('Tabela Mamdani Finala:');
disp(TabelaMamdaniFinala);

fprintf('Modelul %s a rulat în %.2f secunde\n', model, timp);

open_system([model, '/Iesire']);
