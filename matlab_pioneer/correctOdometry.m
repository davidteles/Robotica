function [x,y,w] = correctOdometry(pose,d,error_w)

    x=pose(1)*cos(error_w*(d/1000))-pose(2)*sin(error_w*(d/1000)^2);
    y=pose(1)*sin(error_w*(d/1000))+pose(2)*cos(error_w*(d/1000));
    w=pose(3)+error_w*(d/1000);
    
end