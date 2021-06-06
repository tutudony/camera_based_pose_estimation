function A = A_matrix(x, y, z, q_x, q_y, q_z, v_x, v_y, v_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z, w_x, w_y, w_z, a_x, a_y, a_z, ng_x, ng_y, ng_z, na_x, na_y, na_z, nbg_x, nbg_y, nbg_z, nba_x, nba_y, nba_z)

%Got the jacobian from MATLAB using sysm function but since it takes a lot
%of time to use the subs function in matlab, we are creating a function for
%the same

A = [ 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, (sin(q_y)*(bg_x + ng_x - w_x))/(cos(q_y)^2 + sin(q_y)^2) - (cos(q_y)*(bg_z + ng_z - w_z))/(cos(q_y)^2 + sin(q_y)^2), 0, 0, 0, 0, -cos(q_y)/(cos(q_y)^2 + sin(q_y)^2), 0, -sin(q_y)/(cos(q_y)^2 + sin(q_y)^2), 0, 0, 0 ; 0, 0, 0, (cos(q_x)*cos(q_y)*(bg_z + ng_z - w_z))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2) - (cos(q_x)*sin(q_y)*(bg_x + ng_x - w_x))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2) + (cos(q_y)*sin(q_x)*(sin(q_x)*cos(q_y)^2 + sin(q_x)*sin(q_y)^2)*(bg_z + ng_z - w_z))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2)^2 - (sin(q_x)*sin(q_y)*(sin(q_x)*cos(q_y)^2 + sin(q_x)*sin(q_y)^2)*(bg_x + ng_x - w_x))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2)^2, - (cos(q_y)*sin(q_x)*(bg_x + ng_x - w_x))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2) - (sin(q_x)*sin(q_y)*(bg_z + ng_z - w_z))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, 0, 0, 0, -(sin(q_x)*sin(q_y))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), -1, (cos(q_y)*sin(q_x))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, 0, 0; 0, 0, 0, (sin(q_y)*(sin(q_x)*cos(q_y)^2 + sin(q_x)*sin(q_y)^2)*(bg_x + ng_x - w_x))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2)^2 - (cos(q_y)*(sin(q_x)*cos(q_y)^2 + sin(q_x)*sin(q_y)^2)*(bg_z + ng_z - w_z))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2)^2, (cos(q_y)*(bg_x + ng_x - w_x))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2) + (sin(q_y)*(bg_z + ng_z - w_z))/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, 0, 0, 0, sin(q_y)/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, -cos(q_y)/(cos(q_x)*cos(q_y)^2 + cos(q_x)*sin(q_y)^2), 0, 0, 0 ; 0, 0, 0, cos(q_x)*sin(q_y)*sin(q_z)*(ba_x - a_x + na_x) - cos(q_x)*cos(q_y)*sin(q_z)*(ba_z - a_z + na_z) - cos(q_z)*sin(q_x)*(ba_y - a_y + na_y), (cos(q_z)*sin(q_y) + cos(q_y)*sin(q_x)*sin(q_z))*(ba_x - a_x + na_x) - (cos(q_y)*cos(q_z) - sin(q_x)*sin(q_y)*sin(q_z))*(ba_z - a_z + na_z), (cos(q_y)*sin(q_z) + cos(q_z)*sin(q_x)*sin(q_y))*(ba_x - a_x + na_x) + (sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z)*sin(q_x))*(ba_z - a_z + na_z) - cos(q_x)*sin(q_z)*(ba_y - a_y + na_y), 0, 0, 0, 0,  0, 0, sin(q_x)*sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z), cos(q_x)*cos(q_z), - cos(q_z)*sin(q_y) - cos(q_y)*sin(q_x)*sin(q_z) ; 0, 0, 0, cos(q_z)*sin(q_x)*(ba_y - a_y + na_y) + cos(q_x)*cos(q_y)*cos(q_z)*(ba_z - a_z + na_z) - cos(q_x)*cos(q_z)*sin(q_y)*(ba_x - a_x + na_x), (sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z)*sin(q_x))*(ba_x - a_x + na_x) - (cos(q_y)*sin(q_z) + cos(q_z)*sin(q_x)*sin(q_y))*(ba_z - a_z + na_z), cos(q_x)*sin(q_z)*(ba_y - a_y + na_y) - (cos(q_z)*sin(q_y) + cos(q_y)*sin(q_x)*sin(q_z))*(ba_z - a_z + na_z) - (cos(q_y)*cos(q_z) - sin(q_x)*sin(q_y)*sin(q_z))*(ba_x - a_x + na_x), 0, 0, 0, 0, 0, 0, - cos(q_y)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y), -cos(q_x)*cos(q_z), cos(q_y)*cos(q_z)*sin(q_x) - sin(q_y)*sin(q_z) ; 0, 0, 0, cos(q_y)*sin(q_x)*(ba_z - a_z + na_z) - cos(q_x)*(ba_y - a_y + na_y) - sin(q_x)*sin(q_y)*(ba_x - a_x + na_x), cos(q_x)*sin(q_y)*(ba_z - a_z + na_z) + cos(q_x)*cos(q_y)*(ba_x - a_x + na_x), 0, 0, 0, 0, 0, 0, 0, cos(q_x)*sin(q_y), -sin(q_x), -cos(q_x)*cos(q_y) ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

end
