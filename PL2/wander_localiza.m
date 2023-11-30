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
waitForTransform(tftree,'/robot0','/robot0_laser_1');
sensorTransform = getTransform(tftree,'/robot0','/robot0_laser_1');

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
 if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
    disp('Robot Localizado');
 break;
end
 %Dibujar los resultados del localizador con el visualizationHelper
 %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
 %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
 %en la figura ‘fig_vfh’

 %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
 %valor proporcional a la dirección anterior (K=0.1)

 %Publicar el mensaje de velocidad

 %Esperar al siguiente periodo de muestreo
end