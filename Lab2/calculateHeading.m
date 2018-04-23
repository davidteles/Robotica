function [heading,distance] = calculateHeading(c_x,c_y,d_x,d_y)
    
    t_x=d_x-c_x;
    t_y=d_y-c_y;
    
    
    distance=sqrt(t_x^2+t_y^2);
    
    
    heading=atan2d(t_y,t_x);
   
    
    
    %heading=round(heading,2);
    heading=degtorad(heading);
    
end

