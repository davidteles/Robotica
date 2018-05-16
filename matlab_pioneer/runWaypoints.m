close all;
clear waypoints;
if(exist('SP')~=1)
    SP=serial_port_start('/dev/cu.usbserial');
    
end
pioneer_init(SP);
%[999999 9999999;9999999 9999999];
correction_waypoints=[999999 9999999;9999999 9999999];%[3 8.5;3 14.20;10.35 18.10;17.42 16.10;17.14 3.21];

%measure points
waypoints=[0 0;3 0;3 18;10.35 18;18.42 18;18.42 3.21;3 3.21;3 0;0 0];

%waypoints=[0 0;3 0;3 4;3 18.5;17 18.5;17 4;3 4;3 0;0 0];
%waypoints=[0 0;3 0;3 4;17 4;17 18.5;3 18.5;3 4;3 0;0 0];

%waypoints=[0 0;3 0;3.2 4;3.39 18;17.8 17.9;18.4 2.95;3.6 4.1;3.6 -0.5;0.5 0.0];
pose=pioneer_read_odometry()
%waypoints=[pose(1)/1000 pose(2)/1000;pose(1)/1000+1 pose(2)/1000;pose(1)/1000+1 pose(2)/1000+1];
correction_waypoints=correction_waypoints.*1000;
waypoints=waypoints.*1000;
proportional_heading_error=10;
proportional_heading_step=50;%percentage
waypoint_threshold=350;%Value in mm
maxspeed=250;
linearspeed=0;
angularspeed=0;
distancerun=0;
error_w=degtorad(-0.35);
%error_w=0;
T=0.5;
offset = [0 0];
current_position=[pose(1)/1000 pose(2)/1000];
current_heading=degtorad((360*pose(3))/4096);
hold on
scatter(waypoints(:,1),waypoints(:,2),'b')
scatter(current_position(1),current_position(2),'r');
for i = 1:length(waypoints)
    
    [w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));
    
    if(i<length(waypoints))
        
        while(d>waypoint_threshold)
            
            
            for j=1:length(correction_waypoints)
                [b,d_temp]=calculateHeading(current_position(1),current_position(2),correction_waypoints(j,1),correction_waypoints(j,2));
               
                if(d_temp<400)
                    sonar=pioneer_read_sonars();
                    if(length(sonar)>7)
                        %read sonar 1-Left and 8-Right to correct the
                        %offset
                        
                        if(j==1)
                            offset(1)=((current_position(1)-((3000-650)+sonar(1)+150))+(current_position(1)-((3000-650+1700)-(sonar(8)+150))))/2
                        elseif(j==2)
                            offset(1)=((current_position(1)-((3000-650)+sonar(1)+150))+(current_position(1)-((3000-650+1700)-(sonar(8)+150))))/2
                        elseif(j==3)
                            offset(2)=((current_position(2)-((18750)+sonar(1)+150))+(current_position(2)-((18750-1700)-(sonar(8)+150))))/2
                        elseif(j==4)
                            offset(1)=((current_position(1)-((17420)+sonar(1)+150))+(current_position(1)-((17420-1700)-(sonar(8)+150))))/2       
                        elseif(j==5)
                            offset(2)=((current_position(2)-((2360)+sonar(1)+150))+(current_position(2)-((2360-1700)-(sonar(8)+150))))/2
                        end
                            
                        
                    end
                end
            end
            
            current_position=current_position+offset;
            scatter(current_position(1),current_position(2),'b');
            
            [w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));

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

            angularspeed=-radtodeg(((error)*(proportional_heading_step)/100)/T);
            
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
            %[w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));
            pause(0.05);
        end
    else
        while(d>waypoint_threshold/2)
            current_position=current_position+offset;
            
            [w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));

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
            
            angularspeed=-radtodeg(((error)*(proportional_heading_step)/100)/T);
            
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
            %[w,d]=calculateHeading(current_position(1),current_position(2),waypoints(i,1),waypoints(i,2));
            pause(0.05);
        end
        
    end
    
    
    
end

pioneer_set_controls(SP,0,0);
pioneer_close(SP);

hold off;