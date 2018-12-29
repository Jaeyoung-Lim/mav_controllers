function rotmat = rpy2rotmat(phi, gamma, psi)
    
    Lz = [cos(psi), -sin(psi), 0;
          sin(psi), cos(psi), 0;
                 0,        0, 1];
    Ly = [cos(gamma), 0, sin(gamma);
          0         , 1,          0;
          -sin(gamma), 0, cos(gamma)];
    Lx = [1, 0, 0;
          0, cos(phi), -sin(phi)
          0, sin(phi),  cos(phi)];
    
    rotmat = Lz * Ly * Lx;
end