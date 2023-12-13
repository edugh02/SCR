clear all;
close all;

%Iniciamos el simulador suscribiéndonos a los topics que necesitamos para
%llevar a cabo la simulación


%[Para saber el tipo de mensaje, podemos introducir el comando 'rostopic info /robot0/*topic*']

sub_odom = rossubscriber('/robot0/odom', 'nav_msgs/Odometry');	%Suscripción a Odometria. El primer parámetro es el topic, el segundo es el tipo de mensaje

sub_laser = rossubscriber('/robot0/laser_1','sensor_msgs/LaserScan'); %Suscripción a laser

sub_sonar0 = rossubscriber('/robot0/sonar_0','sensor_msgs/Range') %Suscripción a los sonares
sub_sonar1 = rossubscriber('/robot0/sonar_1','sensor_msgs/Range');
sub_sonar2 = rossubscriber('/robot0/sonar_2','sensor_msgs/Range');
sub_sonar3 = rossubscriber('/robot0/sonar_3','sensor_msgs/Range');
sub_sonar4 = rossubscriber('/robot0/sonar_4','sensor_msgs/Range');
sub_sonar5 = rossubscriber('/robot0/sonar_5','sensor_msgs/Range');
sub_sonar6 = rossubscriber('/robot0/sonar_6','sensor_msgs/Range');
sub_sonar7 = rossubscriber('/robot0/sonar_7','sensor_msgs/Range');	

%Declaración de publishers: Velocidad
pub_vel = rospublisher('/robot0/cmd_vel','geometry_msgs/Twist');

%Generación de mensajes
msg_vel = rosmessage(pub_vel);

%Definimos la periodicidad del bucle
r=rateControl(10);	%se publica un mensaje cada 10Hz


disp('Inicialización del simulador finalizada correctamente');

