function [sys,x0,str,ts] = sfuntmpl(t,x,u,flag)     %四输入：u(1)=>速度给定；u(2)=>速度反馈；u(3)=>电流Iq反馈；u(4)=>电流Id反馈
                                                    %二输出：out(1)=>Vq电压给定；out(2)=>Vd电压给定

%速度PI调节器参数
Kp_speed= 1 ;         %1 0.1586 for M      1 for M（2倍惯量）
Ki_speed= 0.06;       %0.0075 for M      0.06 for M（2倍惯量）

%Kp_speed=1.936;         
%Ki_speed= 2.0283;     

Kc_speed=0.5 ;
Kd_speed=0  ;
umin_speed=-18.6  ;
umax_speed= 18.6  ;
%电流Iq调节器参数
Kp_Iq=19.5 ;        %19.5
Ki_Iq=0.225;       %0.225
Kc_Iq= 0.5  ;
Kd_Iq=0  ;
umin_Iq=-180  ;
umax_Iq=180;
%电流Id调节器参数
Kp_Id=Kp_Iq  ;
Ki_Id=Ki_Iq  ;
Kc_Id=Kc_Iq  ;
Kd_Id=Kd_Iq  ;
umin_Id=umin_Iq  ;
umax_Id=umax_Iq  ;
Id_ref=0;


%PR控制器参数
%%对一次谐波 参数
Kr = 5;    %default: 5
fai = pi*(10)/180; %补偿角度fai，pi*n/180代表n度 default:10
f = 200/15; %基波频率， Nref/15，200代表200rpm
w0=2*pi*f; %基波电角速度
Ts = 1e-3;
den = 4 + (w0*Ts)^2;
com_num1 =  2 * Ts * cos(fai);
com_num2 = w0 * (Ts^2) * sin(fai);
b0 = Kr*(com_num1-com_num2) / den;
b1 = -Kr * 2 * com_num2/den;
b2 = -Kr * (com_num1 + com_num2) / den;
a1 = (2 * (w0 * Ts)^2 - 8) / den;

%%对二次谐波 参数
Kr_2nd = 15;    %配合陷波，default: 15
fai_2nd = pi*(30)/180; %配合陷波，pi*n/180代表n度 default:30
f_2nd = 2*f; %基波频率， Nref/15，200代表200rpm
w0_2nd=2*pi*f_2nd; %基波电角速度
den_2nd = 4 + (w0_2nd*Ts)^2;
com_num1_2nd = 2*Ts*cos(fai_2nd);
com_num2_2nd = w0_2nd*(Ts^2)*sin(fai_2nd);
b0_2nd = Kr_2nd*(com_num1_2nd - com_num2_2nd) / den_2nd;
b1_2nd = -Kr_2nd * 2 * com_num2_2nd / den_2nd;
b2_2nd = -Kr_2nd * (com_num1_2nd + com_num2_2nd) / den_2nd;
a1_2nd = (2 * (w0_2nd * Ts)^2-8)/den_2nd;

%%对六次谐波 参数
Ts_c = 1e-4; %电流环采样频率
Kr_6nd = 250;    %default:1e5
fai_6nd = pi*(30)/180; %补偿角度fai，pi*n/180代表n度 default:-20(速度检测无低通） -5（有低通）
f_6nd = 6*f; %基波频率， Nref/15，200代表200rpm
w0_6nd=2*pi*f_6nd; %基波电角速度
den_6nd = 4 + (w0_6nd*Ts_c)^2;
com_num1_6nd = 2*Ts_c*cos(fai_6nd);
com_num2_6nd = w0_6nd*(Ts_c^2)*sin(fai_6nd);
b0_6nd = Kr_6nd*(com_num1_6nd - com_num2_6nd) / den_6nd;
b1_6nd = -Kr_6nd * 2 * com_num2_6nd / den_6nd;
b2_6nd = -Kr_6nd * (com_num1_6nd + com_num2_6nd) / den_6nd;
a1_6nd = (2 * (w0_6nd * Ts_c)^2 - 8)/den_6nd;

%%卡尔曼滤波器 && RLS参数
b = 0.02; %摩擦系数
Kt = 0.363; %机电常数
C = [1,0];
lambda = 1;
NN = 100;

%%notch filter para
a0_NF = 0.997920436754797;
a1_NF = -1.99577083601781;
a2_NF = 0.997920436754797;
b0_NF = 1;
b1_NF = -1.995770836017815;
b2_NF = 0.995840873509595;

a0_2nd_NF = 0.995849504536408;
a1_2nd_NF = -1.99141944539032;
a2_2nd_NF = 0.995849504536408;
b0_2nd_NF = b0_NF;
b1_2nd_NF = -1.99141944539032;
b2_2nd_NF = 0.991699009072816;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,Kp_speed,Ki_speed,Kd_speed,Kc_speed,umin_speed,umax_speed,Kp_Iq,Ki_Iq,Kd_Iq,Kc_Iq,umin_Iq,umax_Iq,Kp_Id,Ki_Id,Kd_Id,Kc_Id,umin_Id,umax_Id,Id_ref,...
    b0,b1,b2,a1,b0_2nd,b1_2nd,b2_2nd,a1_2nd,b0_6nd,b1_6nd,b2_6nd,a1_6nd,Ts,b,Kt,C,lambda,NN,a0_NF,a1_NF,a2_NF,b0_NF,b1_NF,b2_NF,a0_2nd_NF,a1_2nd_NF,a2_2nd_NF,b0_2nd_NF,b1_2nd_NF,b2_2nd_NF);
 
  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 13;
sizes.NumOutputs     = 11;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
global theta;
theta=0;
global theta_1;
theta_1=0;
global speed;
speed=0;
global speed_1;
speed_1=0;
global out_R
out_R=0;
global out_R_1
out_R_1=0;
global out_R_2
out_R_2=0;
global out_R_2nd
out_R_2nd = 0;
global out_R_2nd_1
out_R_2nd_1 = 0;
global out_R_2nd_2
out_R_2nd_2 = 0;
global iqfdb
iqfdb=0;
global iqfdb_1
iqfdb_1=0;
global out_R_6nd
out_R_6nd = 0;
global out_R_6nd_1
out_R_6nd_1 = 0;
global out_R_6nd_2
out_R_6nd_2 = 0;
global X_hat
X_hat = zeros(2,1);
global X
X = zeros(2,1);
global X_1
X_1 = zeros(2,1);
global P_hat
P_hat = eye(2);
global P
P = eye(2);
global P_1
P_1 =eye(2);
global J0
J0 = 2*5.2e-4;
global Q
Q = [1e-1,0;0,1e-1];
global r
r = 1;
global n 
n = 0;
global err
err = 0;
global Inno_est
Inno_est = 0;
global avg_Q
avg_Q = 0;
global Ident_P_1
Ident_P_1 = eye(2);
global Omega_1
Omega_1 = 0;
global Te_1
Te_1 = 0;
global K
K = 0;
global Ident_theta_1
%Ident_theta_1 = [exp(-B*T/J) 1/B*(1-exp(-B*T/J))]';
Ident_theta_1 = [0,0]';
global J_est
J_est = 0;
global t_2
t_2 = 0;
global t_1
t_1 = 0;
global t_i
t_i = 0;
global out_NF
out_NF = 0;
% initialize the initial conditions
%
x0  = zeros(1,13);
%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [1e-4 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

global speed_e_i
global speed_e_i1
global Iq_ref
global speed_op
global Iq_e_i
global Iq_e_i1
global Vq_ref
global Iq_op

global Id_e_i
global Id_e_i1
global Vd_ref
global Id_op

global J0
global X_hat
global X
global P_hat
global P
global P_1
global Q
global r
global n
global err
global K
global Inno_est
global avg_Q
global Ident_P_1
global Omega_1
global Te_1
global Ident_theta_1
global J_est
global X_1
global t_2
global t_1
global t_i
global out_NF
sys(1)=speed_e_i    ;
sys(2)=speed_e_i1   ;
sys(3)=Iq_ref       ;
sys(4)=speed_op     ;
sys(5)=Iq_e_i       ;
sys(6)=Iq_e_i1      ;
sys(7)=Vq_ref       ;
sys(8)=Iq_op        ;
sys(9)=Id_e_i       ;
sys(10)=Id_e_i1      ;
sys(11)=Vd_ref       ;
sys(12)=Id_op        ;
%程序执行10次x（13）输出为0
x(13)=x(13)+1;
if x(13)==10
    x(13)=0;
end
sys(13)=x(13);

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,Kp_speed,Ki_speed,Kd_speed,Kc_speed,umin_speed,umax_speed,Kp_Iq,Ki_Iq,Kd_Iq,Kc_Iq,umin_Iq,umax_Iq,Kp_Id,Ki_Id,Kd_Id,Kc_Id,umin_Id,umax_Id,Id_ref,...
    b0,b1,b2,a1,b0_2nd,b1_2nd,b2_2nd,a1_2nd,b0_6nd,b1_6nd,b2_6nd,a1_6nd,Ts,b,Kt,C,lambda,NN,a0_NF,a1_NF,a2_NF,b0_NF,b1_NF,b2_NF,a0_2nd_NF,a1_2nd_NF,a2_2nd_NF,b0_2nd_NF,b1_2nd_NF,b2_2nd_NF)                      
%四输入：u(1)=>速度给定；u(2)=>速度反馈；u(3)=>电流Iq反馈；u(4)=>电流Id反馈 u(5)=>theta
%二输出：out(1)=>Vq电压给定；out(2)=>Vd电压给定

                                                    
global speed_e_i
global speed_e_i1
global Iq_ref
global speed_op
global Iq_e_i
global Iq_e_i1
global Vq_ref
global Iq_op
global theta;
global theta_1;
global speed;
global speed_1;
global Id_e_i
global Id_e_i1
global Vd_ref
global Id_op

global iqfdb
global iqfdb_1
global out_R
global out_R_1
global out_R_2

global out_R_2nd
global out_R_2nd_1
global out_R_2nd_2

global out_R_6nd
global out_R_6nd_1
global out_R_6nd_2

global X_hat
global X
global X_1
global P_hat
global P
global P_1
global Q
global r
global n 
global Inno_est
global avg_Q
global err
global K
global J0
global Ident_P_1
global Omega_1
global Te_1
global Ident_theta_1
global J_est
global t_2
global t_1
global t_i
global out_NF
theta=u(5);
iqfdb=u(3);

if t>=0.05
    [out_NF, t_i] = NF(iqfdb,t_1,t_2,a0_NF,a1_NF,a2_NF,b0_NF,b1_NF,b2_NF);
    t_2 = t_1;
    t_1 = t_i;
    %{
    [out_NF_2nd, t_2nd] = NF(out_NF,t_1,t_2,a0_2nd_NF,a1_2nd_NF,a2_2nd_NF,b0_2nd_NF,b1_2nd_NF,b2_2nd_NF);
    t_2nd_2 = t_2nd_1;
    t_2nd_1 = t_2nd;
    %}
end
iqfdb = iqfdb*0.5+0.5*iqfdb_1;
iqfdb_1=iqfdb;


if x(13)==0                   %到了速度环采样时间
    speed=(theta-theta_1)*pi/5;  %10000ppr速度采样
    %speed=(theta-theta_1)*2000*pi/2^17;  %17位编码器速度采样，此时模型中位置采样也要改
    %%===========================================================================
    %Kalman Filter
    
    A = [1-b*Ts/J0,-Ts/J0;0,1];
    B = [Ts/J0;0];
    Te = Kt * iqfdb;
    %Te = Kt * out_NF;
    X_hat = A*X + B*Te;
    P_hat = A*P*A' + Q;
    S = C*P_hat*C' + r;
    K = (1/S)*P_hat*C';
    err = speed - C*X_hat;
    X = X_hat + K*err;
    P = (eye(2)-K*C)*P_hat;
    if (n<NN)
        n = n + 1;
        Inno_est = Inno_est + ( speed - X(1) )^2;
        avg_Q = avg_Q + (speed - X_hat(1))^2;
        if n == NN-1
            Inno_est = Inno_est/NN;
            r = Inno_est + P(1,1);
            %avg_Q = K*Inno_est*K';
            %Q(1,1) = avg_Q(1,1);
            %Q(2,2) = avg_Q(2,2);
            avg_Q = K*avg_Q*K'/NN;
            Q(1,1) = avg_Q(1,1);
            Q(2,2) = avg_Q(2,2);
            n = 0;
            Inno_est = 0;
            avg_Q = 0;
        end
    end
    X_1 = X;
    P_1 = P;
    
    %Kalman Filter    
    
    %%===========================================================================
    speed=(0.6*speed+0.4*speed_1);
    %speed_e_i=u(1)-speed;         %使用测量速度当作速度反馈    
    speed_e_i=u(1)-X(1);  %使用KF滤波转速作速度反馈
    %%%%%PI控制器
   [Iq_ref,speed_op]=pid_s(speed_e_i,x(1),x(2),Kp_speed,Ki_speed,Kd_speed,Kc_speed,umin_speed,umax_speed,x(3),x(4));      
    %x(1)=>speed_e_i-1；x(2)=>speed_e_i-2；x(3)=>Iq_ref(i-1)即Iq_ref前一时刻的值；x(4)=>Iq_ref_out-Iq_ref_out_presat(前一时刻值)
    %speed_op=>Iq_ref_out-Iq_ref_out_presat（当前值）
    speed_e_i1=x(1);            %speed_e_i1=>speed_e_i-1（用于更新）
    theta_1=theta;
    speed_1=speed;
    %%===========================================================================
    %速度环PR控制器
     if t>0.05
         
        [out_R] =  PR(speed_e_i,x(1),x(2),out_R_1,out_R_2,b0,b1,b2,a1);
        out_R_2 = out_R_1;
        out_R_1 = out_R;

        [out_R_2nd] =  PR(speed_e_i,x(1),x(2),out_R_2nd_1,out_R_2nd_2,b0_2nd,b1_2nd,b2_2nd,a1_2nd);
        out_R_2nd_2 = out_R_2nd_1;
        out_R_2nd_1 = out_R_2nd;

        Iq_ref = Iq_ref + out_R_2nd;
        %Iq_ref = Iq_ref + out_R + out_R_2nd;
        %Iq_ref = Iq_ref + X(2)/Kt; %负载转矩前馈
        
    %}
    %速度环PR控制器
    %%===========================================================================
     end
    
     %{
    if t>=0.2
        iqfdb = iqfdb - X(2)/Kt;
    end
     %}
    %%===========================================================================  
    %RLS
    %{
    Ident_phi = [Omega_1 Te_1]';
    Ident_K = Ident_P_1*Ident_phi/(Ident_phi'*Ident_P_1*Ident_phi+lambda);
    Ident_P = (1/lambda)*(eye(2)-Ident_K*Ident_phi')*Ident_P_1;
    Ident_theta = Ident_theta_1 + Ident_K*(speed - Ident_phi'*Ident_theta_1);
    Ident_a = Ident_theta(1);
    Ident_b = Ident_theta(2);
    Ident_P_1 = Ident_P;
    Ident_theta_1 = Ident_theta;
    Te_1 = Te;
    Omega_1 = speed;
    B_est = (1-Ident_a)/Ident_b;          
    if (Ident_a>0)
        J_est = -B_est*Ts/log(Ident_a);
    end
    %}
    %RLS  
    %%===========================================================================  
else                            %未到速度环采样时间
    Iq_ref=x(3);
    speed_e_i=x(1);
    speed_e_i1=x(2);
    speed_op=x(4);
end

%Iq_ref=sin(180*t);

Iq_e_i=Iq_ref-iqfdb;     


if t<=0.05
    Iq_e_i=Iq_ref-iqfdb;             
else
    Iq_e_i=Iq_ref-out_NF;     
end
%}
[Vq_ref,Iq_op]=pid_s(Iq_e_i,x(5),x(6),Kp_Iq,Ki_Iq,Kd_Iq,Kc_Iq,umin_Iq,umax_Iq,x(7),x(8));        
%x(5)=>Iq_e_i-1；x(6)=>Iq_e_i-2；x(7)=>Vq_ref(i-1)即Vq_ref前一时刻的值；x(8)=>Vq_ref_out-Vq_ref_out_presat(前一时刻值)
%Iq_op=>Vq_ref_out-Vq_ref_out_presat（当前值）
Iq_e_i1=x(5);                    %Iq_e_i1=>Iq_e_i-1（用于更新）

%==========================================================================================================
%电流环PR控制器
if t>0.05
    
    [out_R_6nd] =  PR(Iq_e_i,x(5),x(6),out_R_6nd_1,out_R_6nd_2,b0_6nd,b1_6nd,b2_6nd,a1_6nd);
    out_R_6nd_2 = out_R_6nd_1;
    out_R_6nd_1 = out_R_6nd;

    
    %Vq_ref = Vq_ref + out_R_6nd;
    %}
end
%电流环PR控制器
%==========================================================================================================

%电流Id调节
Id_e_i=Id_ref-u(4);             %Id偏差（当前值）
[Vd_ref,Id_op]=pid_s(Id_e_i,x(9),x(10),Kp_Id,Ki_Id,Kd_Id,Kc_Id,umin_Id,umax_Id,x(11),x(12));        
%x(9)=>Id_e_i-1；x(10)=>Id_e_i-2；x(11)=>Vd_ref(i-1)即Vd_ref前一时刻的值；x(12)=>Vd_ref_out-Vd_ref_out_presat(前一时刻值)
%Id_op=>Vd_ref_out-Vd_ref_out_presat（当前值）
Id_e_i1=x(9);                    %Id_e_i1=>Id_e_i-1（用于更新）

sys(1) = Vq_ref;
sys(2)=Vd_ref;
sys(3)=Iq_ref;
sys(4)=speed;
sys(5)=out_NF;
sys(6)=X(1); % KF观测角速度
sys(7)=X(2); % KF观测负载转矩
sys(8)=J_est/2/5.2e-4;
sys(9)=Q(1,1);
sys(10)=Q(2,2);
sys(11)=r;


function [out_i,op_i]=pid_s(e_i,e_i1,e_i2,Kp,Ki,Kd,Kc,umin,umax,out_i1,op_i1)
%out_i=>output presently;   op_i=>out-out_presat;   e_ix=>e(i-x)  x=0,1,2
%out_i1=>output(i-1) previously;    op_i=>out-out_presat previously
deta_e=e_i-e_i1;                    %deta_e=e(i)-e(i-1)
deta_double_e=e_i-2*e_i1+e_i2;      %deta_double_e=e(i)-2e(i-1)+e(i-2)
deta_u_p=Kp*deta_e;                 %比例输出增量
deta_u_i=Ki*(e_i+Kc*op_i1);           %积分输出增量
deta_u_d=Kd*deta_double_e;          %微分输出增量
out_presat=out_i1+deta_u_p+deta_u_i+deta_u_d;           %饱和环节前的输出
if      out_presat<umin
    out_i=umin;
elseif  out_presat>umax
    out_i=umax;
else
    out_i=out_presat;
end
op_i=out_i-out_presat;

function [out_R] = PR(x_i,x_i1,x_i2,y_i1,y_i2,bo,b1,b2,a1)

out_R = -a1*y_i1-y_i2+bo*x_i+b1*x_i1+b2*x_i2;

function [out_NF,t_i] = NF(x_i,t_1,t_2,a0,a1,a2,b0,b1,b2)
%Notch Filter
t_i =( x_i - b1*t_1 - b2*t_2)/b0;
out_NF = a0*t_i + a1*t_1 + a2*t_2;

%end pid_s

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
