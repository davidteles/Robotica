close all;
clear waypoints;
if(exist('SP')~=1)
    SP=serial_port_start('/dev/cu.usbserial');
    pioneer_init(SP);
end


%waypoints=[0 0;2 0;2 18;17 18;17 3;2 3;2 0;0 0];
pose=pioneer_read_odometry()
waypoints=[pose(1)/1000 pose(2)/1000,pose(1)/1000+1 pose(2)/1000];
waypoints=waypoints.*1000;
proportional_heading_error=10;
proportional_heading_step=100;%percentage
waypoint_threshold=250;%Value in mm
maxspeed=200;
linearspeed=0;
angularspeed=0;
distancerun=0;
error_w=0;
T=0.5;
current_position=[pose(1)/1000 pose(2)/1000];
current_heading=degtorad((360*pose(3))/4096);;
hold on
scatter(waypoints(:,1),waypoints(:,2),'b')
scatter(current_position(1),current_position(2),'r');
for i = 1:length(waypoints)
    
    [w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));
    if(i<length(waypoints))
        while(d>waypoint_threshold)

            
            error=(current_heading-w)/(pi);
            if(error>1)
                error=(error-2)*pi;
                
            elseif(error<-1)
                error=(error+2)*pi;
            else
                
                error=error*pi;
            end
            

            m=max(0,min(((d-500)/2500),1))*min(1,1/(proportional_heading_error*abs(error)));
            a=50;
            linearspeed=min(maxspeed,(a+m*maxspeed));

            angularspeed=-((error)*(proportional_heading_step)/100)/T;
            
            %[pose(1),pose(2),pose(3)]=robot(current_position(1),current_position(2),current_heading,linearspeed,angularspeed,T,distancerun);
            pioneer_set_controls(SP,round(linearspeed,0),round(angularspeed,0));
            pose=pioneer_read_odometry();
            pose(3)=degtorad((360*pose(3))/4096);
            scatter(pose(1),pose(2),'r');
            [pose(1),pose(2),pose(3)]=correctOdometry(pose,distancerun,error_w);

            distancerun=updatedistance(current_position(1),current_position(2),pose(1),pose(2),distancerun);
            current_position(1)=pose(1);
            current_position(2)=pose(2);
            current_heading=pose(3);
%             current_heading=current_heading-(error)*(proportional_heading_step/100);
%             current_position(1)=current_position(1)+(linearspeed*cos(current_heading));
%             current_position(2)=current_position(2)+linearspeed*sin(current_heading);

            scatter(current_position(1),current_position(2),'g');
            [w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));
            pause(0.05);
        end
    else
        while(d>waypoint_threshold/2)
            error=(current_heading-w)/(pi);
            if(error>1)
                error=(error-2)*pi;
                
            elseif(error<-1)
                error=(error+2)*pi;
            else
                
                error=error*pi;
            end
            
           
            
            m=min(((d-50)/1500),1)*min(1,1/(proportional_heading_error*abs(error)));
            a=50;
            linearspeed=min(maxspeed,(a+m*maxspeed));
            
            angularspeed=-((error)*(proportional_heading_step)/100)/T;
            
            %[pose(1),pose(2),pose(3)]=robot(current_position(1),current_position(2),current_heading,linearspeed,angularspeed,T,distancerun);
            pioneer_set_controls(SP,round(linearspeed,0),round(angularspeed,0));
            pose=pioneer_read_odometry();
            pose(3)=degtorad((360*pose(3))/4096);
            scatter(pose(1),pose(2),'r');
            [pose(1),pose(2),pose(3)]=correctOdometry(pose,distancerun,error_w);
            
            distancerun=updatedistance(current_position(1),current_position(2),pose(1),pose(2),distancerun);
            current_position(1)=pose(1);
            current_position(2)=pose(2);
            current_heading=pose(3);  
%             current_heading=current_heading-(error)*(proportional_heading_step/100);
%             current_position(1)=current_position(1)+(linearspeed*cos(current_heading));
%             current_position(2)=current_position(2)+linearspeed*sin(current_heading);
            
            scatter(current_position(1),current_position(2),'g');
            [w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));
            pause(0.05);
        end
        
    end
    
end

pioneer_set_controls(SP,0,0);
           

hold off;