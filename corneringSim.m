function [V, lapTime, maxLat, maxLong, Energy, currentDrag, mpower] = corneringSim(len, rad, pervV, vrev,param, action, consTime, En)

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
t(1)=0;
maxLong=0;
Energy=En;
currentDrag = 0;
mpower =0;

if(action=="Reverse")
    vrr=inf(1,len*10);
    vrr(1) = min(pervV,sqrt(myu*g*rad));
    for i =2:len*10
        %Right cornering apply cornering simulation%Left cornering apply cornering simulation % % The equations must be the same as
        %the left  as we are using Bicyclemodel
        Flat= (mf * (vrr(i-1) ^2) ) / rad ;
        Fgrip= sqrt(((myu*mf*g)^2) - (Flat)^2) ;
        if imag(Fgrip)~= 0
            Fgrip = 0;
        end
        Fdrag = 0.5 * Cd*rho* A*(vrr(i-1)) ^2 ;
        Froll = ( mf * g *fr);
        % A concept is used to determine the minimum speed needed to take a curve
        % to be able to brake before reaching another curve
        Fxr = Fgrip + Froll + Fdrag ;
        ar  = Fxr / mf ;
        vrr(i) = sqrt(vrr(i-1)^2  + 2*ar*0.1);
        vrr(i) = min(vrr(i),sqrt(myu*g*rad));
    end
    V = vrr;
end

if(action=="Forward" && consTime == false)
    V=inf(1,len*10);
    Flatr=V;
     V(1) = min(pervV,vrev(1));
     Flatr(1)= (mf *V(1)^2)/ rad;
            mtrans = 0;
            Fgrip= sqrt(((myu*(wbr*mf+mtrans)*g)^2) - (Flatr(1))^2) ;
            if imag(Fgrip)~= 0
                %lateralforce
                Fgrip = myu *(wbr*mf+mtrans)*g;
            end
            
            Fmotor=  P / V(1) ;
            Fdrag = 0.5 * Cd*rho* A*(V(1)) ^2 ;
            Froll = ( mf * g *fr);
            % Forward
            Fxl = min (Fgrip , Fmotor) - Fdrag -Froll;
            maxLong=max(maxLong,Fxl/mf);
            
            Tr=Fxl*rdyn/ie;
 
            currentDrag=max(currentDrag, Tr);
            a = (Fxl / mf);
   
        for i =2:len*10
            %Right cornering apply cornering simulation%Left cornering apply cornering simulation % % The equations must be the same as
            %the left  as we are using Bicyclemodel
            Flatr(i)= (mf * (V(i-1) ^2) ) / rad;
            mtrans = (a*h)/Lwb*mf;
            Fgrip= sqrt(((myu*(wbr*mf+mtrans)*g)^2) - (Flatr(i))^2) ;
            if imag(Fgrip)~= 0
                %lateralforce
                Fgrip = myu *(wbr*mf+mtrans)*g;
            end
           
            Fmotor=  P / V(i-1) ;
            Fdrag = 0.5 * Cd*rho* A*(V(i-1)) ^2 ;
            Froll = ( mf * g *fr);
            % Forward
            Fxl = min (Fgrip , Fmotor) - Fdrag -Froll;
            maxLong=max(maxLong,Fxl/mf);
              Tr=Fxl*rdyn/ie;
          
            mpower = max(mpower,(V(i)*(60/2/pi/rdyn)*Tr*ie/9550)*1000);
            currentDrag=max(currentDrag, Tr);
            a = (Fxl / mf);
            V(i) = sqrt(V(i-1)^2  + 2*a*0.1) ;
            % Take the minimum of both the reverse velocity and the and the forward
            % velocity, to be able to brake to the next maximum corner velocity
            % after exiting the first corner ?
            V(i) = min(V(i),vrev(1));
        end
end

if(action=="Forward" && consTime == true)
    V=inf(1,len*10);
    t=zeros(1,len*10);
    Flatr=V;    
    V(1) = min(pervV,vrev(1));
       Flatr(1)= (mf *V(1)^2) / rad;
            mtrans = 0;
            Fgrip= sqrt(((myu*(wbr*mf+mtrans)*g)^2) - (Flatr(1))^2) ;
            if imag(Fgrip)~= 0
                %lateralforce
                Fgrip = myu *(wbr*mf+mtrans)*g;
            end
            maxLong=max(maxLong,Fgrip/mf);
   
            Fmotor=  P / V(1) ;
            Fdrag = 0.5 * Cd*rho* A*(V(1)) ^2 ;
            Froll = ( mf * g *fr);
            % Forward
            Fxl = min (Fgrip , Fmotor) - Fdrag -Froll;
            maxLong=max(maxLong,Fxl/mf);
      
              Tr=Fxl*rdyn/ie;
            currentDrag=max(currentDrag,  Tr);
            a = (Fxl / mf);
        for i =2:len*10
            %Right cornering apply cornering simulation%Left cornering apply cornering simulation % % The equations must be the same as
            %the left  as we are using Bicyclemodel
            Flatr(i)= (mf * (V(i-1) ^2) ) / rad;
             mtrans = ((a/9.81)*h)/Lwb*mf;
            Fgrip= sqrt(((myu*(wbr*mf+mtrans)*g)^2) - (Flatr(i))^2) ;
            if imag(Fgrip)~= 0
                %lateralforce
                Fgrip = myu *(wbr*mf+mtrans)*g;
            end
            
            Fmotor=  P / V(i-1);
            Fdrag = 0.5 * Cd*rho* A*(V(i-1)) ^2 ;
            Froll = ( mf * g *fr);
            % Forward
            Fxl = min (Fgrip , Fmotor) - Fdrag -Froll;
            maxLong=max(maxLong,Fxl/mf);
          
               Tr=Fxl*rdyn/ie;
            currentDrag=max(currentDrag, Tr);
            a = (Fxl / mf);
            V(i) = sqrt(V(i-1)^2  + 2*a*0.1) ;
            % Take the minimum of both the reverse velocity and the and the forward
            % velocity, to be able to brake to the next maximum corner velocity
            % after exiting the first corner ?
            V(i) = min(V(i),vrev(i));
            mpower = max(mpower,(V(i)*(60/2/pi/rdyn)*Tr*ie/9550)*1000);
            if(V(i) == V(i-1))
                t(i)=0.1/V(i);
            else
                t(i) = (V(i) - V(i-1)) / a;
            end       
            Energy=Energy+Fxl*0.1;
        end
end
lapTime = sum(t);
maxLat = (max(V))^2 / rad;
