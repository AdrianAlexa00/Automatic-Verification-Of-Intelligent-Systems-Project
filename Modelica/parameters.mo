package K
  constant Real g = 9.81 "Gravity acceleration [m/s^2]";
  constant Real T = 0.02 "Integration time step [s]";
  constant Integer N = 10 "Total number of drones in the swarm";

  // LiDAR Sensors & 27 Bin Grid
  constant Real FOV_horiz = 3.14159265359 "Horizontal FOV 180 degrees [rad]";
  constant Real FOV_vert = 1.0471975512 "Vertical FOV 60 degrees [rad]";
  constant Real sensorRange = 20.0 "Maximum LiDAR range [m]"; 

  constant Integer grid_H = 9 "Horizontal bins";
  constant Integer grid_V = 3 "Vertical bins";
  constant Integer grid_size = 27 "Total grid size (9x3)";

  // Map and Confinement
  constant Real dome_radius = 50.0 "Dome radius [m]";
  constant Real dome_center[3] = {0.0, 0.0, 0.0};
  constant Real dome_max_height = 50.0;

  // Drone Physics
  constant Real m = 5.0 "Drone mass [kg]";
  constant Real Cd = 0.5 "Air drag coefficient";
  constant Real Area_side = 0.15 "Frontal area [m^2]";
  constant Real Area_top = 0.30 "Top area [m^2]";
  constant Real rho = 1.225 "Air density [kg/m^3]";
  
  constant Real V_nom = 22.2 "6S battery pack nominal voltage [V]";
  constant Real Bat_Capacity_Ah = 16.0 "Total battery capacity [Ah]";
  constant Real Bat_Energy_Wh = 355.2 "Nominal stored energy [Wh]";
  constant Real Bat_Energy_J = 1278720.0 "Total stored energy [Joules]";
  constant Real P_hover = 750.0 "Estimated electrical power in static hover [W]";
  


  constant Real P_avionics = 15.0 "Constant power draw of sensors and onboard computer [W]";
  constant Real Max_Usable_SoC = 0.85 "Safety discharge limit (85% of total energy)";
  
end K;
