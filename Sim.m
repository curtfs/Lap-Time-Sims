function [Time, Energy, lmat, vmat, maxLongA, maxLatA, currentDrag, mpower] = Sim(data, param)
type=cell2mat(data.type);
len=data.len;
rad=data.rad;
maxLongA=zeros(1,length(type));
maxLatA=zeros(1,length(type));
currentDrag = zeros(1,length(type));
mpower=currentDrag;


%Reverse
p=length(type);
vmat=(length(type));
lmat=(length(type));
vrev = inf(p ,max(len)*10);
U = 0;
lt=0;
Energy=0;
for p=1:length(type)
    if(type(p)=='S' || type(p)=='s')
        [V, lapTime, maxLong, en,current] = accSim(U, len(p), vrev, param, "Reverse", false, Energy);
        U = V(len(p)*10);        
        vrev(p, 1: length(V)) = V;
    end
    if(type(p)=='R' || type(p)=='r')
        [V, lapTime, maxLat,  maxLong,Energy, current] = corneringSim(len(p), rad(p), U, vrev, param, "Reverse", false, Energy);
        U = V(end);
        vrev(p, 1:length(V)) = V;
    end
    if(type(p)=='L' || type(p)=='l')
        [V, lapTime, maxLatA, maxLong, Energy, current] = corneringSim(len(p), rad(p), U, vrev, param, "Reverse", false, Energy);
        U = V(end);
        vrev(p, 1:length(V)) = V;
    end
end


%Forward
U = 0;
Tlen=0;
for p=1:length(type)
    
    if(type(p)=='s')
       [V, lapTime, maxLong, Energy, current, mpower(p)] = accSim(U, len(p), vrev(p,:), param, "Forward", false, Energy);
        U= V(len(p)*10);
        maxLongA=maxLong;
        maxLatA(p)=0;
         lmat(p)=Tlen;
        vmat(p)=U;
        currentDrag(p)=current;
    end
    if(type(p)=='r' || type(p)=='l')
        [V, lapTime, maxLat, maxLong,Energy,current,mpower(p)] = corneringSim(len(p), rad(p), U, vrev(p,:), param, "Forward", false,Energy);
        U = V(end);
        lmat(p)=Tlen;
        vmat(p)=U;
        maxLongA(p)=maxLong;
        maxLatA(p)=maxLat;
        currentDrag(p)=current;
    end
    if(type(p)=='S')
        [V, lapTime, maxLong, Energy, current,mpower(p)] = accSim(U, len(p), vrev(p,:), param, "Forward", true, Energy);
        U= V(len(p)*10);
        vmat(p)=U;
        lmat(p)=Tlen;
        lt=lt+lapTime;
        maxLongA(p)=maxLong;
        maxLatA(p)=0;
        currentDrag(p)=current;
    end
    if(type(p)=='R' || type(p)=='L')
        [V, lapTime, maxLat, maxLong,Energy,current,mpower(p)] = corneringSim(len(p), rad(p), U, vrev(p,:), param, "Forward", true,Energy);
         U= V(len(p)*10);
         vmat(p)=U;
        lt=lt+lapTime;
        vmat(p)=U;
        lmat(p)=Tlen;
        maxLatA(p)=maxLat;
        maxLongA(p)=maxLong;
        currentDrag(p)=current;
    end
    Tlen=Tlen+len(p);
end

Energy = (Energy*2.77778e-7);
Time = lt;