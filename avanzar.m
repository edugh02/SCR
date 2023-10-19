
distancia=1;

%Rellenamos los campos del mensaje para que el robot avance a 0.1m/s

%[Introducimos 'msg_vel' en la línea de comandos para conocer los parámetros y saber cómo podemos escribir la velocidad]

msg_vel.Linear.X=0.1;	%Indicamos que la velocidad lineal en el eje X sea 0.1
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=0;

%Obtenemos posición inicial
initpos= sub_odom.LatestMessage.Pose.Pose.Position;

%Bucle de control infinito
while(1)
	%obtenemos posición actual
	pos=sub_odom.LatestMessage.Pose.Pose.Position;
	disp(sprintf('\nPosición actual: X=%f, Y=%f',pos.X, pos.Y));

	%Calculamos la distancia euclídea que se ha desplazado
	dist=sqrt((initpos.X-pos.X)^2+(initpos.Y-pos.Y)^2);
	disp(sprintf('\tDistancia avanzada: %f', dist));

	if(dist>distancia)
		msg_vel.Linear.X=0;
		send(pub_vel, msg_vel);
		break;
	else
		send(pub_vel, msg_vel);
	end
	leer_sensores;
	%vfh_dir;
	waitfor(r);
end
