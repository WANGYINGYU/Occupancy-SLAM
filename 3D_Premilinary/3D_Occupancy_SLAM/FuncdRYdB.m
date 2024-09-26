function dRYdB = FuncdRYdB(Betai)

dRYdB = [-sin(Betai),0,-cos(Betai);
         0,0,0;
         cos(Betai),0,-sin(Betai)];