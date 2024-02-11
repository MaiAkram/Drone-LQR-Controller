function[pos] = StartGoal_allocation(UAVmap3D, initial_r, final_r)

int = true;
% while int == true
%     x0 = randi([10 20],1);
%     y0 = randi([10 20],1);
%     z0 = randi([10 20],1);
%     start = [x0 y0 z0];
% 
%     xf = randi([150 200],1);
%     yf = randi([150 200],1);
%     zf = randi([30 35],1);
%     goal  = [xf yf zf];
% 
%     if (checkOccupancy(UAVmap3D,start) == 0) && (checkOccupancy(UAVmap3D,goal) == 0)
%         int = false;
%     end
% end

while int == true
    x = randi([initial_r(1) final_r(1)],1);
    y = randi([initial_r(2) final_r(2)],1);
    z = randi([initial_r(3) final_r(3)],1);
    pos = [x y z];

    if (checkOccupancy(UAVmap3D,pos) == 0)
        int = false;
    end
end

%start = [x0 y0 z0 0.7 0.2 0 0.1];   % [x y z qw qx qy qz]
%goal  = [xf yf zf 0.3 0 0.1 0.6];

