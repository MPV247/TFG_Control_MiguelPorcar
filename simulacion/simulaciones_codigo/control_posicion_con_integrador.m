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

%% Sistema con integrador (Solo para la posición X)
% Seleccionamos solo la primera fila de C (que corresponde a la posición x)
C_x = [1 0 0 0]; 

Ae = [A,         zeros(4,1); 
     C_x,       0];

Be = [B_volt; 0];


%% Control con optimización LQR.
%--- LQR ---
% 1. Definición de pesos
Q = diag([5, 1, 0.7, 0.1, 3]); 
r = 0.001;                

R = r * eye(1);       

% 2. Resolver la optimización
[K_lqr, S, E] = lqr(Ae, Be, Q, R);


%% Control de posicion del sistema (Realimentación del estado)
N=6000;
T=1e-3;
x=zeros(4,N);
x(2,1)=3.05;
u=zeros(1,N);
t=zeros(1,N);
u_max = 2.91;           %Saturacion de la acción de control (N)
ref = [0, 3.14, 0, 0];  %Punto control intestable
Ie = 0.0; 

for k=1:N-1
   % 1. Error integral
   Ie = Ie + (ref(1) - x(1,k)) * T;
   
   % 2. Crear el vector de estado aumentado (5x1)
   x_aug = [ref' - (x(:,k)); Ie];
   
   % 3. Acción de control con las 5 ganancias de K_lqr
   u_calc = K_lqr * x_aug;

   if (u_calc > u_max || u_calc <-u_max)
        %Anti-windup
        Ie = Ie - (ref(1) - x(1, k)) * T;
        x_aug = [(ref' - x(:, k)); Ie];
        if (u_calc > u_max)
            u(k) = u_max;
        elseif (u_calc < -u_max)
            u(k) = -u_max; 
        end
   else
        u(k) = K_lqr * x_aug;
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
