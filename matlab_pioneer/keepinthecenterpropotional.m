%linear speed proportional to the available distance in front of the robot
%angular speed and time proportional to the difference between the readings
%of the side firing sonars (1,2-Left 7,8-Right)
global linearspeed;
global angularspeed;

angularspeed = 0;
linearspeed = 0;
if(exist('SP')~=1)
    SP=serial_port_start('/dev/cu.usbserial');
    pioneer_init(SP);
end
lastcorrection=0;


maxangularcorrection=45;%Degrees
maxspeed=400;
forwardspeed=50;
%pioneer_set_controls(SP,100,0);

%Tentar chamar em nova thread
forwardspeed=progressivespeed(SP,forwardspeed,100);
flag=0;
while(flag==0)
    sonar=pioneer_read_sonars();
    odometry=pioneer_read_odometry()
    if(length(sonar)>7)

        pioneer_set_controls(SP,linearspeed,angularspeed);
        if(sonar(4)<500 || sonar(5)<500)
            %Stops running
            flag=1;
            
        else
            
            m=(maxspeed-0)/(5000-1000);
            a=maxspeed*0.25;
            
            newspeed=round(min(maxspeed,(-a+m*(min(sonar(4),sonar(5))))),0);
            
            if(linearspeed<newspeed)
                linearspeed=linearspeed+5;
            elseif(linearspeed>newspeed)
                linearspeed=linearspeed-5;
            end
        end
        

            anglecorretion=maxangularcorrection*min(1,(((sonar(1)-sonar(8))*0.5+(sonar(2)-sonar(7))*0.5))*0.0001);
            tempangularspeed=round((anglecorretion)/1,0);
            
            if(tempangularspeed==0)
                angularspeed=0;
            elseif(angularspeed<tempangularspeed)
                angularspeed=angularspeed+1;
            elseif(angularspeed>tempangularspeed)
                angularspeed=angularspeed-1;
            end
        
    end
end

pioneer_set_controls(SP,0,0);