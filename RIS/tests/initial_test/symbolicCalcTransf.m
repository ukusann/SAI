function symbolic_matrix = symbolicCalcTransf(alpha, a, d, theta)
    symbolic_matrix = [
        cos(theta)               -sin(theta)               0.0               a;
        cos(alpha).*sin(theta)     cos(theta)*cos(alpha)   -sin(alpha)     -sin(alpha)*d;
        sin(theta)*sin(alpha)     cos(theta)*sin(alpha)    cos(alpha)      cos(alpha)*d;
        0.0                       0.0                      0.0             1.0
        ];

end 