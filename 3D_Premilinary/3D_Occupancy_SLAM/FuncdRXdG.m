function dRXdG = FuncdRXdG(Gammai)

dRXdG = [0,0,0;
         0,-sin(Gammai),cos(Gammai);
         0,-cos(Gammai),-sin(Gammai)];