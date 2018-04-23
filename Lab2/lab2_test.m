
if(exist('SP')~1)
    SP=serial_port_start('/dev/cu.usbserial');
    pioneer_init(SP);
end


forwardspeed=50;
%pioneer_set_controls(SP,100,0);
progressivespeed(SP,forwardspeed,100,1);
flag=0;
while(flag==0)
    sonar=pioneer_read_sonars()
    
    for i=1:min(length(sonar),8)
        if(sonar(i) < 1000 && i<=3)
            display('Virar Direita');
            forwardspeed=progressivespeed(SP,forwardspeed,100,0.1);
            progressiveturn(SP,forwardspeed,-100,0.25)
            %pioneer_set_controls(SP,100,-100);
            %pause(0.2);
        elseif(sonar(i) < 1000 && i>=6)
            display('Virar Esquerda');
            forwardspeed=progressivespeed(SP,forwardspeed,100,0.1);
            progressiveturn(SP,forwardspeed,100,0.25)
            %pioneer_set_controls(SP,100,100);
            %pause(0.2);
        elseif(sonar(i) < 1250)
            %Stops running
            flag=1;
            
        else
            forwardspeed=progressivespeed(SP,forwardspeed,150,1);
            
        end
        
    end
end
forwardspeed=progressivespeed(SP,forwardspeed,0,0.5);
pioneer_set_controls(SP,0,0);