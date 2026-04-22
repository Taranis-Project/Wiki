%% Conversion Excel -> MATLAB iddata

% Nom du fichier Excel
nomFichier = 'DONNEES.xlsx';

% Lecture du fichier Excel (toutes les cellules)
brut = readcell(nomFichier);

% Supprimer la première ligne (en-têtes)
brut = brut(2:end,:);

% Conversion des virgules en points + conversion en nombres
for i = 1:size(brut,1)
    for j = 1:2   % seulement colonnes 1 et 2
        if ischar(brut{i,j}) || isstring(brut{i,j})
            brut{i,j} = strrep(brut{i,j}, ',', '.'); % remplacer , par .
            brut{i,j} = str2double(brut{i,j});       % convertir en nombre
        end
    end
end

% Extraction des colonnes
u = cell2mat(brut(:,1)); % entrée
y = cell2mat(brut(:,2)); % sortie

% Création de l'objet iddata (Ts = 1 par défaut)
data = iddata(y,u,1);

% Sauvegarde dans un fichier MAT
save('DONNEES.mat','data','u','y')

disp('Conversion terminée. Fichier DONNEES.mat créé.')
