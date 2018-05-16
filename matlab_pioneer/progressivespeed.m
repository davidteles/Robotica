%Increases/decreases the speed of the robot with small increments
function speed = progressivespeed(SP,from,to)
    global linearspeed;
    global angularspeed;
    valuesinterval = abs(from-to);

    if(valuesinterval<50)
        speed =from;
        if(exist('linearspeed'))
            linearspeed=speed;
        end
        
        return ;
    end
    
    for i=1:round(valuesinterval/10,0)
        if(to>from)
            speed=from+i*10;
            if(exist('linearspeed') && exist('angularspeed'))
                %display('Using Global');
                linearspeed=speed;
                pioneer_set_controls(SP,linearspeed,angularspeed);
            else
                pioneer_set_controls(SP,speed,0);
            end

        else
            speed=from-i*10;
            if(exist('linearspeed') && exist('angularspeed'))
                linearspeed=speed;
                pioneer_set_controls(SP,linearspeed,angularspeed);
            else
                pioneer_set_controls(SP,speed,0);
            end
            
          
        end
  
    end


end