syms omega_1 omega_2 omega_3 omega_4
syms c_bar p_bar q_bar r_bar

A = [1,  1,  1,  1;
     1, -1, -1,  1;
     1,  1, -1, -1;
     1, -1,  1, -1];
 
b = [c_bar; p_bar; q_bar; r_bar];

disp(A \ b)