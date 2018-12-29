function landed = islanded(state)
    if state.pos(3) >= 0
        landed = true;
    else
        landed = false;
    end
end