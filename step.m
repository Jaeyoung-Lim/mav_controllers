function state = step(state, roll_rate, pitch_rate, wind_gradient)
    
    constants();
    [gamma_a, psi_a, phi, pos, vel, v_a] = getstate(state);
    
    %% Add Dynamics
    
    
    state = setstate(gamma_a, psi_a, phi, pos, vel, v_a);
end
