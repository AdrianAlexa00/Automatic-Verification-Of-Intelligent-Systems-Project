block UAV
  import C.*;
  import K;

  parameter Integer id = 1;

  C.InputReal p_target_global[3] "Target fisico per Hero (id=1)";
  C.InputReal setvr;
  C.InputReal setvtheta;
  C.InputReal setvphi;

  C.InputReal swarmBus[K.N, 3];

  C.OutputReal x, y, z;
  C.OutputReal Vx, Vy, Vz;
  C.OutputReal Trustx, Trusty, Trustz;

  Controller ctr(id=id);
  UAV_HW plant;
  LocalFrontier frontier(id=id);

equation
  // --- BUS -> FRONTIER LIDAR ---
  connect(swarmBus, frontier.swarmBus);
  
  
  connect(ctr.heading_out,frontier.headingCtr);

  // --- SETPOINT -> CONTROLLER ---
  connect(p_target_global, ctr.p_target_global);
  connect(setvr, ctr.setvr);
  connect(setvtheta, ctr.setvtheta);
  connect(setvphi, ctr.setvphi);

  // --- FEEDBACK HARDWARE -> CONTROLLER ---
  connect(plant.Vx, ctr.Vx);
  connect(plant.Vy, ctr.Vy);
  connect(plant.Vz, ctr.Vz);
  connect(plant.x, ctr.x);
  connect(plant.y, ctr.y);
  connect(plant.z, ctr.z);

  // --- CONTROLLER -> HARDWARE (THRUST) ---
  connect(ctr.Trustx, plant.Trustx);
  connect(ctr.Trusty, plant.Trusty);
  connect(ctr.Trustz, plant.Trustz);

  // --- HARDWARE -> FRONTIER LIDAR ---
  connect(plant.x, frontier.x);
  connect(plant.y, frontier.y);
  connect(plant.z, frontier.z);
  connect(plant.Vx, frontier.Vx);
  connect(plant.Vy, frontier.Vy);
  connect(plant.Vz, frontier.Vz);

  // --- FRONTIER LIDAR -> CONTROLLER ---
  connect(frontier.grid, ctr.grid);

  // --- ESPOSIZIONE USCITE UAV ---
  connect(plant.x, x);
  connect(plant.y, y);
  connect(plant.z, z);
  connect(plant.Vx, Vx);
  connect(plant.Vy, Vy);
  connect(plant.Vz, Vz);
  connect(plant.Trustx, Trustx);
  connect(plant.Trusty, Trusty);
  connect(plant.Trustz, Trustz);

end UAV;
