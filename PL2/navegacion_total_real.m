%Definir la posicion de destino
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
endLocation = [14 2];

%Cargar el mapa
%%%%%%%%%%%%%%%

load OnlineSLAM_mapa.mat 
show(map);

%Crear el objeto VFH…y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Inicializar el localizador AMCL (práctica 1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;

tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'/base_link','/laser_frame');
sensorTransform = getTransform(tftree,'/base_link','/laser_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 50000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = true;      % global = true      local=false
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = diag([1 1 1])*0.5; % Covariance of initial pose %eye(3)*0.5;

visualizationHelper = ExampleHelperAMCLVisualization(map);

fig_laser=figure; title('LASER')
fig_vfh=figure; title('VFH')

VFH=controllerVFH;
VFH.SafetyDistance=0.5;

VFH.UseLidarScan=true;

msg_vel.Linear.X = 0.1;	%Indicamos que la velocidad lineal en el eje X sea 0.1
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = 0;

%Crear el objeto PurePursuit y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller=controllerPurePursuit;
controller.LookaheadDistance = 2;
controller.DesiredLinearVelocity= 0.1;
controller.MaxAngularVelocity = 0.5;

%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
%Bucle de control infinito

while(1)
    % Receive laser scan and odometry message.
    scan = receive(sub_laser);
    odompose = sub_odom.LatestMessage;
    
    %Crear objeto para almacenar el escaneo LiDAR 2-D
    scans = lidarScan(scan);

    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);
    
    % Drive robot to next pose.
    %wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
 %Leer y dibujar los datos del láser en la figura ‘fig_laser’

 %Leer la odometría


 %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
 %Ejecutar amcl para obtener la posición estimada estimatedPose y la
 %covarianza estimatedCovariance (mostrar la última por pantalla para
 %facilitar la búsqueda de un umbral)
 %Si la covarianza está por debajo de un umbral, el robot está localizado y
 %finaliza el programa

 if (estimatedCovariance(1,1)<0.01 && estimatedCovariance(2,2)<0.01 && estimatedCovariance(3,3)<0.01)
    disp('Robot Localizado');
    msg_vel.Linear.X = 0;	
    msg_vel.Angular.Z = 0;
    send(pub_vel,msg_vel);
    break;
 end
 
 figure(fig_laser)
 leer_sensores;
  
 %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
 %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
 %en la figura mfig_vfh’
 scans=lidarScan(msg_laser);
 steeringDir = VFH(scans,0)*4;
 figure(fig_vfh);
 show(VFH)
 
 %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
 %valor proporcional a la dirección anterior (K=0.1)
 msg_vel.Angular.Z=0.3*steeringDir;
 %Publicar el mensaje de velocidad
 send(pub_vel,msg_vel);
 %Esperar al siguiente periodo de muestreo
 waitfor(r);

end

%%%%%%%%%%% AL SALIR DE ESTE BUCLE EL ROBOT YA SE HA LOCALIZADO %%%%%%%%%%
%%%%%%%%%%% COMIENZA LA PLANIFICACIÓN GLOBAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Paramos el robot, para que no avance mientras planificamos
%Hacemos una copia del mapa, para “inflarlo” antes de planificar
cpMap= copy(map);
inflate(cpMap,0.35);

%Crear el objeto PRM y ajustar sus parámetros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
planner = mobileRobotPRM;
planner.Map = cpMap;
planner.NumNodes = 2000;
planner.ConnectionDistance = 1;

%Obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
%en una figura
startLocation = [estimatedPose(1) estimatedPose(2)];
ruta = findpath(planner,startLocation,endLocation);
figure(); show(planner);

%%%%%%%%%%% COMIENZA EL BUCLE DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Indicamos al controlador la lista de waypoints a recorrer (ruta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller.Waypoints = ruta;

%Bucle de control
%%%%%%%%%%%%%%%%%
while(1)
    %Leer el láser y la odometría
    scan = receive(sub_laser);
    scans = lidarScan(scan);
    odompose = sub_odom.LatestMessage;

    %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    %Ejecutar amcl para obtener la posición estimada estimatedPose
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);  

    %Dibujar los resultados del localizador con el visualizationHelper
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    %Ejecutar el controlador PurePursuit para obtener las velocidades lineal
    %y angular
    [lin_vel,ang_vel] = controller(estimatedPose);

    %Llamar a VFH pasándole como “targetDir” un valor proporcional a la
    %velocidad angular calculada por el PurePursuit
    targetdir = ang_vel;
    direccion = VFH(scans,targetdir);
    ang_vel_vfh = 0.8*direccion;

    %Calcular la velocidad angular final como una combinación lineal de la
    %generada por el controlador PurePursuit y la generada por VFH
    msg_vel.Linear.X = lin_vel;
    msg_vel.Angular.Z = ang_vel+ang_vel_vfh;

    %Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);

    %Comprobar si hemos llegado al destino, calculando la distancia euclidea
    %y estableciendo un umbral
    destino_alcanzado = sqrt((estimatedPose(1)-endLocation(1))^2+(estimatedPose(2)-endLocation(2))^2)
    if (destino_alcanzado<0.25)
        disp('Localización alcanzda');
        msg_vel.Linear.X = 0;	
        msg_vel.Angular.Z = 0;
        send(pub_vel,msg_vel);
        break;
     end   

  
    %Esperar al siguiente periodo de muestreo
    waitfor(r);

end