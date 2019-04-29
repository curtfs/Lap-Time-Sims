function [V,lapTime, maxLong, Energy, currentDrag, mpower] = accSim(pervV, len, vrev, param, action, consTime, En)

%Vehicle's data/parameters
amax = param(1);
rdyn  =param(2);
ie = param(3);
mf = param(4);
g =  param(5);
Cd = param(6);
eta = param(7);
fr = param(8);
rho = param(9);
A = param(10);
myu =  param(11);
h = param(12);
Lwb = param(13);
wbr = param(14);
P = param(15);
Energy=En;
currentDrag=0;
mpower=0;
if(action=="Reverse")
 t = zeros (1,len*10);
        a = t;
        V = t;
        mtrans = t;
        FL = t;
        Fzex= t;
        T=t;
        %Assuming start of any track of to be a straight
        mtrans(1) = ((amax)*h)/Lwb*mf;
        FL(1) = ((wbr *mf)+mtrans(1))*g* myu;
        
        a(1) =   (mf * g *fr)  + 0.5 *rho*Cd*A*(0)  / mf;
        V(1) = sqrt (pervV^2 + 2*a(1)*0.1);
        for i =2:len*10
            %Calculate traction limitation then adjust the new torque if
            %exceeded the limit.
            Fzex(i) =  ( mf * g *fr)  + 0.5 *rho*Cd* A*(V(i-1)^2);
            mtrans(i) = ((a(i-1)/9.81)*h)/Lwb*mf;
            FL(i)=(mtrans(i)*g +wbr *mf*g)* myu;
            
            %Calculate the new acceleration.
            a(i) = ( mf * g *fr)  + 0.5 *rho*Cd* A*(V(i-1)^2)/ mf;
            %Calculate the new velocity.
            V(i) = sqrt(V(i-1)^2 + 2*a(i)*0.1);
        end
        maxLong = a;
end

if(action=="Forward" && consTime==false)
     t = zeros (1,len*10);
        a = t;
        V = t;
        mtrans = t;
        FL = t;
        Fzex= t;
        T=t;
        %Assuming start of any track of to be a straight
        mtrans(1) = ((amax)*h)/Lwb*mf;
        FL(1) = ((wbr *mf)+mtrans(1))*g* myu;
        
        a(1) = min(amax,(mf * g *fr)  + 0.5 *rho*Cd*A*(0)  / mf);
        V(1) = sqrt (pervV^2 + 2*a(1)*0.1);
        for i =2:len*10
            %Calculate traction limitation then adjust the new torque if
            %exceeded the limit.
            
            Fzex(i) = (T(i-1)*ie / rdyn) * eta  -( mf * g *fr)  - 0.5 *rho*Cd* A*(V(i-1)^2);
            mtrans(i) = ((a(i-1)/9.81)*h)/Lwb*mf;
            FL(i)=(mtrans(i)*g +wbr *mf*g)* myu;
            
            if(Fzex(i)~=FL(i))
                T(i-1)= (FL(i) + (mf*g*fr) + 0.5 *rho*Cd* A*(V(i-1)^2))*(rdyn/(ie*eta));
            end
            
          
            %Calculate power limitation then adjust the new torque if
            %exceeded the limit.
            
            if((T(i-1)/rdyn)*V(i-1)*ie >= P)
                T(i)=(P*rdyn)/(V(i-1)*ie);
            else
                T(i)=T(i-1);
            end
              mpower = max(mpower,(V(i-1)*(60/2/pi/rdyn)*T(i)*ie/9550)*1000);
             currentDrag=max(currentDrag, T(i));
            %Calculate the new acceleration.
            a(i) = ((T(i)*ie / rdyn) * eta  -( mf * g *fr)  - 0.5 *rho*Cd* A*(V(i-1)^2))/ mf;
            
            %Calculate the new velocity.
            V(i) = sqrt(V(i-1)^2 + 2*a(i)*0.1);
            V(i) = min(V(i), vrev(i));
            %Check if acceleration went to zero or not because of the step
            %division.
            if(a(i)<= 0)
                a(i)=0;
            end
           
        end
         maxLong = max(a);
end



if(action=="Forward" && consTime==true)
            
     t = zeros (1,len*10);
        a = t;
        V = t;
        mtrans = t;
        FL = t;
        Fzex= t;
        T=t;
        time=t;
        %Assuming start of any track of to be a straight
        mtrans(1) = ((amax)*h)/Lwb*mf;
        FL(1) = ((wbr *mf)+mtrans(1))*g* myu;
        
        a(1) = min(amax*g,(mf * g *myu)  + 0.5 *rho*Cd*A*(0)  / mf);
        V(1) = min(sqrt (pervV^2 + 2*a(1)*0.1),vrev(1));
        t(1) = (V(1)-pervV)/a(1);
        for i =2:len*10
            %Calculate traction limitation then adjust the new torque if
            %exceeded the limit.
            
            Fzex(i) = (T(i-1)*ie / rdyn) * eta  -( mf * g *fr)  - 0.5 *rho*Cd* A*(V(i-1)^2);
            mtrans(i) = ((a(i-1)/g)*h)/Lwb*mf;
            FL(i)=(mtrans(i)*g +wbr *mf*g)* myu;
            
            if(Fzex(i)~=FL(i))
                T(i-1)= (FL(i) + (mf*g*fr) + 0.5 *rho*Cd* A*(V(i-1)^2))*(rdyn/(ie*eta));
            end
        
            
            
            %Calculate power limitation then adjust the new torque if
            %exceeded the limit.
            
            if((T(i-1)/rdyn)*V(i-1)*ie >= P)
                T(i)=(P*rdyn)/(V(i-1)*ie);
            else
                T(i)=T(i-1);
            end
            mpower = max(mpower,(V(i-1)*(60/2/pi/rdyn)*T(i)*ie/9550)*1000);
             currentDrag=Fzex(i)*1.1*rdyn;
            
            
            %Calculate the new acceleration.
            a(i) = ((T(i)*ie / rdyn) * eta  -( mf * g *fr)  - 0.5 *rho*Cd* A*(V(i-1)^2))/ mf;
            
            if(a(i)>11.855)
            a(i)=11.85;
            end
            %Calculate the new velocity.
            V(i) = sqrt(V(i-1)^2 + 2*a(i)*0.1);
            V(i) = min(V(i), vrev(i));
    %Check if acceleration went to zero or not because of the step
            %division.
            if(a(i) <= 0)
                t(i) = 0.1 / V(i) ;
            else
                t(i) =  (V(i) - V(i-1)) / a(i);
            end
            time(i)=time(i-1)+t(i);
            Energy= Energy + Fzex(i)*0.1;
        end
        avg = sum(T)/length(T);
        maxLong=max(a);
        maxi=max(Fzex); 
        Tr = FL .* rdyn ; 
maxtr = max(Tr); 

end
     
lapTime = sum(t);
