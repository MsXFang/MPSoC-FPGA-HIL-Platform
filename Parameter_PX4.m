%% 清理内存
clear all;close all;clc;

%% Sensor Model Parameters Defination
% Board Rotation
% BoardRotation = [0 0 0];% unit:degree

%% 采样率 要与开发板中的保持一致 
Ts=1/2000;
load MavLinkStruct;
load('noise_gain.mat');

%% Motor Model
ModelParam_motorWb=22.83;
ModelParam_motorCr=842.1;
ModelParam_motorT= 0.0214;%0.0261;
ModelInit_RPM = 0; %Initial motor speed (rad/s)
ModelParam_motorMinThr=0.05;

%% Force and Moment Model
ModelParam_uavMass=1.515;%%1.5;
ModelParam_uavR=0.225;
ModelInit_GroundSlope=[0,0];
ModelParam_uavType = int8(3); %这里是四旋翼X型，具体定义见文档"机型定义文档.docx"
ModelParam_motorJm =0.0001287;
ModelParam_rotorCm=2.783e-07;
ModelParam_rotorCt=1.681e-05;
ModelParam_uavCd = 0.055;
ModelParam_uavCCm = [0.0035 0.0039 0.0034];
ModelParam_uavDearo = 0.12;%%unit m
% Propeller Model Version New
TypeMotorNonVector=int8([3;3;4;4;6;6;6;8;8;8]);
TypeMotorDirMatrix=[...
    [-1 -1 -1 0 0 0 0 0];... % Tri 3*1 X
    [1 1 1 0 0 0 0 0];...    % Tri 3*1 +
    [-1 -1 1 1 0 0 0 0];... % Quad 4*1 X
    [-1 -1 1 1 0 0 0 0];... % Quad 4*1 +
    [1 -1 1 -1 -1 1 0 0];... % Hex 6*1 X
    [1 -1 1 -1 -1 1 0 0];... % Hex 6*1 +
    [1 -1 1 -1 1 -1 0 0];... % HexCoa 3*2
    [1 1 -1 -1 -1 -1 1 1];... % Oct 8*1 X
    [1 1 -1 -1 -1 -1 1 1];... % Oct 8*1 +
    [-1 1 -1 1 -1 1 -1 1];... % OctCoa 4*2 X
]';
TypeMotorAngMatrix=[...
    [pi/3 pi -pi/3 0 0 0 0 0];...
    [pi/3 pi -pi/3 0 0 0 0 0];...
    [pi/4 pi/4+pi pi/4+3*pi/2 pi/4+pi/2 0 0 0 0];...
    [pi/2 3*pi/2 0 pi 0 0 0 0];...
    [pi/2 3/2*pi 3/2*pi+pi/3 pi/2+pi/3 pi/2-pi/3 3/2*pi-pi/3 0 0];...
    [0 pi pi+pi/3 pi/3 -pi/3 pi-pi/3 0 0];...
    [pi/3 pi/3 pi pi -pi/3 -pi/3 0 0];...
    [pi/8 pi+pi/8 pi/8+pi/4 pi-pi/8 -pi/8 3/2*pi-pi/8 3/2*pi+pi/8 pi/2+pi/8];...
    [0 pi pi/4 3/4*pi -pi/4 pi+pi/4 3/2*pi pi/2];...
    [pi/4 -pi/4 pi+pi/4 pi-pi/4 -pi/4 pi/4 pi-pi/4 pi+pi/4];...
]';

%% Environment Model
ModelParam_envAltitude = -100;     %参考高度，负值  -50
ModelParam_noiseUpperWindBodyRatio=0;%风波动系数，风速*(1+该系数)
ModelParam_GlobalNoiseGainSwitch =0.4;
ModelParam_envLongitude = 116.259368300000;
ModelParam_envLatitude = 40.1540302;
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];
TemperatureConstant = 25.0;

%% 6DOF
ModelInit_PosE=[0,0,0];
ModelInit_VelB=[0,0,0];
ModelInit_AngEuler=[0,0,0];
ModelInit_RateB=[0,0,0];
% ModelParam_uavMass --> Force and Moment Model
ModelParam_uavJxx = 0.0211;%%1.491E-2;
ModelParam_uavJyy = 0.0219;%%1.491E-2;
ModelParam_uavJzz = 0.0366;%%2.712E-2;
ModelParam_uavJ= [ModelParam_uavJxx,0,0;0,ModelParam_uavJyy,0;0,0,ModelParam_uavJzz];

%% SensorModel1
% ModelParam_uavMass --> Force and Moment Model
% ModelParam_uavType --> Force and Moment Model
% HILSensorMavModel1
% ModelParam_GlobalNoiseGainSwitch --> Environment Model
% ModelParam_envAltitude --> Environment Model
%GPS Parameter
ModelParam_GPSEphFinal=0.3;
ModelParam_GPSEpvFinal=0.4;
ModelParam_GPSFix3DFix=3;
ModelParam_GPSSatsVisible=10;
%HILSensorMavModel1:
ModelParam_noiseBaroCoupleWithSpeed=0;%气压计测量高度与动压关系，也就是风速与气压计掉高模型的系数，当前参数0.008飞机10m/s掉高1m

%% Fault Paraments
ACCScale = 1.0;
GyroScale = 1.0;
MagScale = 1.0;
A_GRotation = [0 0 0];
MagRotation = [0 0 0];
ACCDrift = 0.0;
GyroDrift = 0.0;
MagDrift = 0.0;
MyNoiseGain = [2.5;3;4].*3;
ACCNoise = 1.0;
GyroNoise = 1.0;
MagNoise = 1.0;
BaroNoise = 1.0;
ACC_UserFault = 0;
ACC_bias = [0 0 0];
Gyro_bias = [0 0 0];
% accnoisegain = [1 1 1];
accnoisegain = 0;
PWMNoiseGain = [10;15;3];
Hzisact = 0.0;
GPSNoise = 1.0;

%% not use
% % Define the 32-D FaultInParams vector for external modification
% FaultParamAPI.FaultInParams = zeros(32,1);
% % initialize the params used in Simulink model
% FaultParamAPI.FaultInParams(3)=1;
% ModelInit_Inputs = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
% ModelParam_uavMotNumbs = int8(4);
% %ModelParam_ControlMode = int8(1); %整型 1表示Auto模式，0表示Manual模式
% ModelParam_timeSampBaro = 0.01;
% ModelParam_timeSampTurbWind = 0.01;
% ModelParam_BusSampleRate = 0.001;
% ModelParam_BattHoverMinutes=18;
% ModelParam_BattHoverThr=0.609;
% %Noise Parameter
% ModelParam_noisePowerAccel = [0.001,0.001,0.003];%顺序 xyz 下同  不要修改这里
% ModelParam_noiseSampleTimeAccel = 0.001;
% ModelParam_noisePowerOffGainAccel = 0.04;
% ModelParam_noisePowerOffGainAccelZ = 0.03;
% ModelParam_noisePowerOnGainAccel = 0.8;
% ModelParam_noisePowerOnGainAccelZ = 4.5; 
% ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001];%不要修改这里
% ModelParam_noiseSampleTimeGyro = 0.001;
% ModelParam_noisePowerOffGainGyro = 0.02;
% ModelParam_noisePowerOffGainGyroZ = 0.025;
% ModelParam_noisePowerOnGainGyro = 2;%3.2;
% ModelParam_noisePowerOnGainGyroZ = 1;
% ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];%不要修改这里
% ModelParam_noiseSampleTimeMag = 0.01;
% ModelParam_noisePowerOffGainMag = 0.02;
% ModelParam_noisePowerOffGainMagZ = 0.035;
% ModelParam_noisePowerOnGainMag = 0.025;
% ModelParam_noisePowerOnGainMagZ = 0.05;
% ModelParam_noisePowerIMU=0;%IMU噪声，这里是白噪声，这里是经过归一化
% ModelParam_noiseUpperGPS=0.5;  %GPS定位误差噪声，均匀噪声，这里填x,y,z的波动上限，单位是m
% ModelParam_noiseGPSSampTime=0.01;%默认0.05 
% ModelParam_noiseUpperBaro=0; %气压计噪声，均匀噪声，这里填高度的波动上限，单位是m
% ModelParam_noiseBaroSampTime=0.01;%气压计噪声更新频率，，默认0.05 
% ModelParam_noiseWindSampTime=0.001;
% ModelParam_envAirDensity = 1.225;    %还没有用到
% ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar
% ModelParam_noiseTs = 0.001; e 


