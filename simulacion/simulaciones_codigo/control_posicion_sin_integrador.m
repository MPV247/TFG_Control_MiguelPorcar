%% MODELADO DEL SISTEMA. 
%Parámetros experimentales
mc = 1.08;     %Kg , masa del carro.
mp = 0.12;     %Kg , masa del péndulo.
M = mp + mc;   %Kg, masa total del sistema.
J = 0.0139;    %Kgm², momento de inercia del sistema.
cp = 0.0002 ;  %Nm.rad/s, coeficiente de rozamiento del péndulo.
cc = 4.08;     %Ns/m , coeficiente de rozamiento del carro.
g = 9.81;      % m/s², aceleración debida a la gravedad
lcm = 0.3;     %m, Distancia entre el centro de masas y el eje de rotación.  

%% Obtención de la representación en SS a partir de la dinámica

%Matrices M, C, G, B (Definen la dinámica del sistema). 

M_bar = [M,          -mp*lcm;
        -mp*lcm,      J]; 

C_bar = [cc,           0;
          0,          cp       ]; 

G_bar= [  0              0; 
          0,         -mp*g*lcm]; 

%A: 4x4, se tienen cuatros estados. 
A= [zeros(2),        eye(2);
    -M_bar\G_bar ,-M_bar\C_bar];

%B: 4x1, una acción de control para cuatro estados.
B = [0;0;[M_bar\[1;0]]];
B_volt = 0.64*B; %Relación lineal entre Newtons y voltios

%C: 4X4, disponibles todas las mediciones.
C = eye(4); 

%D: 1x1
D = 0; 



%% Control con optimización LQR.
G_BA = ss(A, B_volt, C, D)
n = 4; % N estados
m = 1; % N acciones de control 

%--- LQR ---
% 1. Definición de pesos
Q = diag([1, 1, 1, 1]); 
r = 0.1;                

R = r * eye(m);       

% 2. Resolver la optimización
[K_lqr, S, E] = lqr(A, B_volt, Q, R);

% --- Asignacion de polos en BC ---
p = [-2.5, -3, -3.5, -4]; 
K_plc = place(A, B_volt, p);

%% Control de posicion del sistema (Realimentación del estado)
N=12;
T=500e-3;
x=zeros(4,N);
x(2,1)=3.05;
u=zeros(1,N);
t=zeros(1,N);
u_max = 2.91;           %Saturacion de la acción de control (N)
ref = [0, 3.14, 0, 0];  %Punto control intestable

for k=1:N-1
   % Accion de control de posición:
   u(k) = -K_plc * (x(:,k) - ref'); %Aquí falta poner que la ref del ángulo es pi

   %Saturación de la acción de control:
   if (u(k) > u_max)
       u(k) = u_max;
   elseif (u(k) < -u_max)
       u(k) = -u_max;
   end
  
   
   % Simulación física (no lineal, "realidad")
   M_t = [M,          +mp*lcm*cos(x(2,k));
   +mp*lcm*cos(x(2,k)),      J];

   C_t = [cc,   -mp*lcm*x(4,k)*sin(x(2,k))
     0,          cp        ];

   G_t = [0; +mp*g*lcm*sin(x(2,k))];

   f1=x(3,k);
   f2=x(4,k);
   f34=M_t\([u(k);0]-C_t*x(3:4,k)-G_t);

   %Cálculo del valor de los estados en la siguiente iteración.
   x(:,k+1)=x(:,k)+T*[f1;f2;f34];
   t(k+1)=t(k)+T;
end

dist = x(1, :);
theta = x(2, :) -3.14; %Graficar todo en 0
vel = x(3, :);
theta_dot = x(4, :);

%Obtener la gráfica de la simulación
figure, plot(t, dist'), hold on
plot(t,theta')
plot(t, u')
legend('x', 'theta', 'u')

exportgraphics(gcf, 'sim_discreta_500ms.pdf', 'ContentType', 'vector');