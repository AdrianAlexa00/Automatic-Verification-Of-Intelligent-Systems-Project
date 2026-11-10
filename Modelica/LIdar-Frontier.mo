block LocalFrontier
  import Modelica.Math.*;
  import K;

  parameter Integer id = 1;
  parameter Real T = 0.001;

  C.InputReal x, y, z;
  C.InputReal Vx, Vy, Vz;
  C.InputReal  swarmBus[K.N, 3];

  C.OutputReal grid[K.grid_size];
  C.InputReal headingCtr;
  


protected
  discrete Real p_drone[3];
  discrete Real heading(start = 0.0);
  discrete Real rel_x, rel_y, rel_z;
  discrete Real dist, azim, elev;
  discrete Integer bin_h, bin_v, index;
  Real res_h = K.FOV_horiz / K.grid_H;
  Real res_v = K.FOV_vert / K.grid_V;

algorithm
when sample(0, K.T) then
  p_drone := {x, y, z};
  heading := pre(headingCtr);

  for idx in 1:K.grid_size loop
    grid[idx] := 0.0;
  end for;

  for i in 1:K.N loop
    if i <> id then
      rel_x := swarmBus[i, 1] - p_drone[1];
      rel_y := swarmBus[i, 2] - p_drone[2];
      rel_z := swarmBus[i, 3] - p_drone[3];
      
      dist := sqrt(rel_x^2 + rel_y^2 + rel_z^2);

      if dist <= K.sensorRange and dist > 0.1 then
          azim := atan2(sin(atan2(rel_y, rel_x) - heading), cos(atan2(rel_y, rel_x) - heading));
          elev := atan2(rel_z, sqrt(rel_x^2 + rel_y^2));
          
        if azim >= -K.FOV_horiz/2 and azim <= K.FOV_horiz/2 and
           elev >= -K.FOV_vert/2 and elev <= K.FOV_vert/2 then
          
          bin_h := min(K.grid_H, max(1, integer(floor((azim + K.FOV_horiz/2) / res_h)) + 1));
          bin_v := min(K.grid_V, max(1, integer(floor((elev + K.FOV_vert/2) / res_v)) + 1));
          
          index := (bin_v - 1) * K.grid_H + bin_h;
          grid[index] := max(grid[index], (K.sensorRange - dist) / K.sensorRange);
        end if;
      end if;
    end if;
  end for;
 end when;
end LocalFrontier;
