block UAV_HW

parameter Real CFx = 1.0;
parameter Real CFy = 1.0;
parameter Real CFz = 2.0;

Real Vair_x "Airspeed X-component";
Real Vair_y  "Airspeed Y-component";
Real Vair_z "Airspeed Z-component";


C.OutputReal x;
C.OutputReal y;
C.OutputReal z;
C.OutputReal Vx;
C.OutputReal Vy;
C.OutputReal Vz;

C.InputReal Trustx;
C.InputReal Trusty;
C.InputReal Trustz;




parameter Real x_init = 0.0 annotation(Evaluate=false);
parameter Real y_init = 0.0 annotation(Evaluate=false);
parameter Real z_init = 0.0 annotation(Evaluate=false);

//Velocity magnitude
Real V_mag "Total airspeed magnitude";


initial equation

x = x_init;
  y = y_init;
  z = z_init;

    Vx = 0;
    Vy = 0;
    Vz= 0;
    
    
    
    
//Z-Axis: Altitude (Up/Down). If Z goes negative, that means it is underground.
//X-Axis: Horizontal plane (e.g., North/South).
//Y-Axis: Horizontal plane (e.g., East/West).    
    
    
    


equation




// Calculate relative velocity (Ground Speed - Wind Speed)
  Vair_x = Vx; //- windX;
  Vair_y = Vy; //- windY;
  Vair_z = Vz; //- windZ;
  
  // Magnitude of the relative velocity vector
  V_mag = sqrt(Vair_x^2 + Vair_y^2 + Vair_z^2 + 1e-6);

  der(x) = Vx;
  der(y) = Vy;
  der(z) = Vz;

  
  // X and Y use Area_side (Lateral drag)
  der(Vx) = (Trustx - (0.5 * K.rho * K.Cd * K.Area_side * V_mag * Vair_x)) / K.m;
  der(Vy) = (Trusty - (0.5 * K.rho * K.Cd * K.Area_side * V_mag * Vair_y)) / K.m;
  
  // Z uses Area_top (Vertical drag) and includes gravity [cite: 14, 19]
  der(Vz) = (Trustz - (K.m * K.g) - (0.5 * K.rho * K.Cd * K.Area_top * V_mag * Vair_z)) / K.m;
  
  




end UAV_HW;

