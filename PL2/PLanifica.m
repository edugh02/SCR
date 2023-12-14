%Definir la posicion de destino
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
endLocation = [13 5];

%Cargar el mapa
%%%%%%%%%%%%%%%

load OnlineSLAM_mapa_simpleRooms.mat 
show(map);

%Paramos el robot, para que no avance mientras planificamos
%Hacemos una copia del mapa, para “inflarlo” antes de planificar
cpMap= copy(map);
inflate(cpMap,0.25);

%Crear el objeto PRM y ajustar sus parámetros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
planner = mobileRobotPRM;
planner.Map = cpMap;
planner.NumNodes = 1000;
planner.ConnectionDistance = 3;

%Obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
%en una figura
startLocation = [3 -5];
ruta = findpath(planner,startLocation,endLocation);
figure; show(planner);