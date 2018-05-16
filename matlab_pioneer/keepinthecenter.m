%Tries to keep in the center of two walls, harcode times and angular
%velocity
if(exist('SP')~=1)
    SP=serial_port_start('/dev/cu.usbserial');
    pioneer_init(SP);
end


forwardspeed=50;
%pioneer_set_controls(SP,100,0);
progressivespeed(SP,forwardspeed,100,1);
flag=0;
while(flag==0)
    sonar=pioneer_read_sonars()
    
    if(length(sonar)>7)
        if(sonar(8)-sonar(1)>250 || sonar(7)-sonar(2)>350)
            display('Virar Direita');
            forwardspeed=progressivespeed(SP,forwardspeed,100,0.1);
            progressiveturn(SP,forwardspeed,-50,0.1)
            %pioneer_set_controls(SP,100,-100);
            %pause(0.2);
        elseif(sonar(1)-sonar(8)>250 || sonar(2)-sonar(7)>350)
            display('Virar Esquerda');
            forwardspeed=progressivespeed(SP,forwardspeed,100,0.1);
            progressiveturn(SP,forwardspeed,50,0.10)
            %pioneer_set_controls(SP,100,100);
            %pause(0.2);
        elseif(sonar(4)<1000 || sonar(5)<1000)
            %Stops running
            flag=1;
            
        else
            forwardspeed=progressivespeed(SP,forwardspeed,150,1);
            
        end
        
    end
end
forwardspeed=progressivespeed(SP,forwardspeed,0,0.5);
pioneer_set_controls(SP,0,0);