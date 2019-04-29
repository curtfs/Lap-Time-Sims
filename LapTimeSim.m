clear
clc
%Omar Hossam
%Ahmed Mokhtar
%Vehicle's data/parameters
param = zeros(1,15);
param(1) = 1.2;        %amax: Acceleration on which sliping is about to happen
param(2)=0.23;         %rdyn
param(3)=3;             %ie Gear Ratio 
param(4)=280;           %mf 108 Batteries 
param(5)=9.81;          %g 
param(6)= 0.6;          %Cd 
param(7)= 0.9;          %eta 
param(8)= 0.03;         %fr 
param(9)= 1.23;         %rho 
param(10)= 1.4;         %A 
param(11)=1.25;          %myu 
param(12)= 300e-3;      %h 
param(13)= 1.60;       %Lwb 
param(14)= 0.6;        %wbr 
param(15)=45600;        %max mechanical power
endurancePowerLimit = 0.7;


%/****************************************Importing Data/Simulating******************************************/%
data=readtable('Skidpad.xlsx');
[Tteam(1) En(1) lmat1 vmat1 maxLongA1 maxLatA1, currentDrag1]=Sim(data, param);
Tteam(1)=Tteam(1)/2;
data=readtable('Acceleration.xlsx');
[Tteam(2) En(2) lmat2 vmat2 maxLongA2 maxLatA2, currentDrag2, mpower]=Sim(data, param);
data=readtable('Endurance.xlsx');
[Tteam(3) En(3) lmat3 vmat3 maxLongA3 maxLatA3, currentDrag3, mpower]=Sim(data, param);
data=readtable('Endurance.xlsx');
param(15)=param(15)*endurancePowerLimit;       %P
[Tteam(4) En(4) lmat4 vmat4 maxLongA4 maxLatA4, currentDrag4]=Sim(data, param);
Tteam(4) = 20*Tteam(4);
En(4)= 20*En(4);
%/**********************************************************************************************************/%
timeArr = zeros(length(currentDrag3));
last=0;
for i=1:Tteam(3)
    timeArr(i)=i;
    last=i;
end
for l=1:length(currentDrag3)
timeArr(last+l)=last;
end


plot(timeArr, currentDrag3);
%/******************************************Evaluation and Scoring******************************************/%



%Some Parametric Evaluations.
ePower=mpower*(1/param(7))+3000;
maxmpower=max(mpower);
maxepower=max(ePower);
avgCurrentDrag = sum(currentDrag3)/length(currentDrag3);
maxCurrentDrag = max(currentDrag1);
maxmpower=max(mpower);


% Omar Hussam & Ahmed Mokhtar
% TE for Endurance -- TA for Acceleration-- TS Skidpad -- TC Autocross
% These max values belong to Michigan 2014 , but scoring equations belong
% to FSUK 19.

Tmax = [4.847 4.131 49.890 997.8];
ENmax=1.984;
%Arranger
 
% Scoring
%Skidpad 
    
 SS  =3.5+ 71.5 * (((1.25*Tmax(1)/Tteam(1)))^2 - 1)/(0.5625);
if SS > 75 
    SS=75; 
end
 
%Acceleration
AS  = 3.5 + 71.5 *(((1.5*Tmax(2) / Tteam(2)) - 1 )/ 0.5 );
if AS > 75 
    AS=75; 
end

%Autocross
ACS = 4.5 + 95.5 * (((1.25*Tmax(3)/Tteam(3))-1)/ 0.25);
if ACS > 100 
    ACS=100; 
end

%Endurance 
ES = 300 *(((1.333*Tmax(4)/Tteam(4))-1) / 0.333);
if ES > 300 
    ES=300; 
end
%Efficiency
effFactor=(Tmax(4)*ENmax)/(Tteam(4)*En(4));
effFactorM=0.693;
EF = 100*(((0.1/effFactor)-1)/((0.1/effFactorM)-1)); 

% TOTAL score 
Sc = [SS , AS,ACS, ES, EF];
Events = ["SkidPad" , "Acceleration" , "Autocross" , "Endurance Event", "Fuel Efficiency"];
Eveno = [1,2,3,4,5];
TS = SS +AS +ACS + ES+EF; 
fprintf ('The total score for the dynamic events is %.0f / 650 \n', TS);
fprintf ('The total energy consumed is %f kwh', En(4));
for i=1:5
fprintf('\n%s:\t\t\t%d', Events(i),Sc(i));    

end
g = bar(Sc);

