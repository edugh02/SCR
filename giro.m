
angulo = pi/2;

if(angulo>0) velocidad = 0.1;
else velocidad = -0.1;
end;

msg_vel.Linear.X=0;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=velocidad;

%Leemos la primera posición
initori = sub_odom.LatestMessage.Pose.Pose.Orientation;
yawini = quat2eul([initori.W initori.X initori.Y initori.Z]);   %yawini = posición sobre el eje Z
yawini = yawini(1);

%Bucle de control infinito
while(1)
    %Obtenemos posición inicial
    ori = sub_odom.LatestMessage.Pose.Pose.Orientation;

    yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw = yaw(1);
    disp(sprintf('\nOrientación actual: X=%f', yaw));

    %Calculamos el ángulo girado
    ang = angdiff(yawini, yaw);
    disp(sprintf('\tAngulo girado (grados): %f', ang));

    %Si hemos girado el ángulo indicado, detenemos el robot y salimos del
    %bucle
    if (abs(ang)>abs(angulo))
        msg_vel.Angular.Z=0;
        send(pub_vel, msg_vel);
        break;
    else
        send(pub_vel,msg_vel);

    end
    leer_sensores;
	%vfh_dir;
	waitfor(r);
end