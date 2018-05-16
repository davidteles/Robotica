close all;
if(exist('SP')~=1)
    SP=serial_port_start('/dev/cu.usbserial');
    
    
end
pioneer_init(SP);
pause(0.1);
pose=pioneer_read_odometry();
figure;
hold on
scatter(pose(1),pose(2),'r');
pause(1);
key='';
flag = 0;
while(flag==0)
    pause(0.1);
    pose=pioneer_read_odometry();
    scatter(pose(1),pose(2),'b');
    key=get(gcf,'CurrentCharacter');
    
    if(key=='w')
        pioneer_set_controls(SP,200,0);
    elseif(key=='s')
        pioneer_set_controls(SP,0,0);
        
    elseif(key=='a')
        pioneer_set_controls(SP,0,15);
        
        
    elseif(key=='d')
        pioneer_set_controls(SP,0,-15);
        
        
    elseif(key=='x')
        pioneer_set_controls(SP,0,0);
        flag=1;
    end
    key;
end

pioneer_close(SP);