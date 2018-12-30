function landed = islanded(position)
    if size(position, 2) == 2
        r = 2;
    else size(position, 2) == 3
        r = 3;
    end
    
    if position(r) <= 0
        landed = true;
    else
        landed = false;
    end
end