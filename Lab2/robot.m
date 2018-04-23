function [x,y,w] = robot(c_x,c_y,c_w,linearspeed,angularspeed,interval,dist)


    
    d=linearspeed*interval;
    w=c_w+angularspeed*interval;

    x=round(c_x+d*cos(w),1);
    y=round(c_y+d*sin(w),1);

end