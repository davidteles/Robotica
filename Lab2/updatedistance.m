function d = updatedistance(o_x,o_y,c_x,c_y,d,multiplier)
    d=d+(sqrt((o_x-c_x)^2+(o_y-c_y)^2)*multiplier);

end