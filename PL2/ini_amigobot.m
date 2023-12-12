%Iniciamos el simulador suscribiéndonos a los topics que necesitamos para
%llevar a cabo la ejecución del robot


%[Para saber el tipo de mensaje, podemos introducir el comando 'rostopic info /robot0/*topic*']

sub_odom = rossubscriber('/pose', 'nav_msgs/Odometry');	%Suscripción a Odometria, para obtener la posición del robot

sub_laser = rossubscriber('/scan','sensor_msgs/LaserScan'); %Suscripción a laser

sub_sonar0 = rossubscriber('/sonar_0','sensor_msgs/Range') %Suscripción a los sonares
sub_sonar1 = rossubscriber('/sonar_1','sensor_msgs/Range');
sub_sonar2 = rossubscriber('/sonar_2','sensor_msgs/Range');
sub_sonar3 = rossubscriber('/sonar_3','sensor_msgs/Range');
sub_sonar4 = rossubscriber('/sonar_4','sensor_msgs/Range');
sub_sonar5 = rossubscriber('/sonar_5','sensor_msgs/Range');
sub_sonar6 = rossubscriber('/sonar_6','sensor_msgs/Range');
sub_sonar7 = rossubscriber('/sonar_7','sensor_msgs/Range');	

%Declaración de publishers: Velocidad
pub_vel = rospublisher('/cmd_vel','geometry_msgs/Twist');
pub_enable = rospublisher('/cmd_motor_state', 'std_msgs/Int32');    %Encender los motores, a modo de seguridad

%Generación de mensajes
msg_vel = rosmessage(pub_vel);
msg_enable_motor = rosmessage(pub_enable);

%Definimos la periodicidad del bucle
r=rateControl(10);	%se publica un mensaje cada 10Hz

%Activar mtores enviando enable_motor=1
msg_enable_motor.Data=1;    %Encendemos motores
send(pub_enable, msg_enable_motor);     %Publicamos el encendido de motores

disp('Inicialización del amigobot finalizada correctamente');
