function [x,y,w] = correctOdometry(pose,d,error_w,heading_strengh)
    %heading_strengh=1;
%     x=pose(1)*cos(error_w*(d/1000))+pose(2)*sin(error_w*(d/1000));
%     y=pose(2)*cos(error_w*(d/1000))-pose(1)*sin(error_w*(d/1000));
    x=pose(1)+pose(2)*sin(error_w*(d/1000));
    y=pose(2)-pose(1)*sin(error_w*(d/1000));
    w=pose(3)-(error_w*(d/1000))*heading_strengh;
    
end