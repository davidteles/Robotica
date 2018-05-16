%Negative values turn right
%Positive values turn left


function progressiveturn(SP,forwardspeed,maxturnspeed)
    global linearspeed;
    global angularspeed;
    %display('Working?');
    if(maxturnspeed==0)
        %display('0 Turn?');
        return;
    end

    for i=1:abs(round(maxturnspeed/5,0))
        if(maxturnspeed>0)
            if(exist('linearspeed') && exist('angularspeed'))
                %display('Using Global');
                angularspeed=i*5;
                pioneer_set_controls(SP,linearspeed,angularspeed);
            else
                pioneer_set_controls(SP,forwardspeed,i);
            end
            
            
        else
            if(exist('linearspeed') && exist('angularspeed'))
                %display('Using Global');
                angularspeed=-i*5;
                pioneer_set_controls(SP,linearspeed,angularspeed);
            else
                pioneer_set_controls(SP,forwardspeed,-i);
            end
        end

    end
    %pause(0.2)
    for i=0:abs(round(maxturnspeed/5,0))
        
         if(maxturnspeed>0)
             
            if(exist('linearspeed') && exist('angularspeed'))
                %display('Using Global');
                angularspeed=maxturnspeed-i*5;
                pioneer_set_controls(SP,linearspeed,angularspeed);
            else
                pioneer_set_controls(SP,forwardspeed,maxturnspeed-i);
            end

        else
            if(exist('linearspeed') && exist('angularspeed'))
                %display('Using Global');
                angularspeed=maxturnspeed+i*5;
                pioneer_set_controls(SP,linearspeed,angularspeed);
            else
                pioneer_set_controls(SP,forwardspeed,maxturnspeed+i);
            end
         end

    end

end