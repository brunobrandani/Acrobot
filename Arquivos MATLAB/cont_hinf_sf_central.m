function [norma, K, var] = cont_hinf_sf_central(A,Bu,Bw,Cz,Dzu,Dzw,gamma)

    n = size(A,1); % Numero de estados
    m = size(Bu,2); % Numero de entradas de controle
        
    setlmis([]);

    X = lmivar(1,[n 1]);
    L = lmivar(2,[m n]);
    
    lmiterm([-1 1 1 X],1,1);
    
    lmiterm([2 1 1 X],A,1,'s');
    lmiterm([2 1 1 L],Bu,1,'s');
    lmiterm([2 2 1 0],Bw');
    lmiterm([2 2 2 0],-gamma^2);
    lmiterm([2 3 1 X],Cz,1);
    lmiterm([2 3 1 L],Dzu,1);    
    lmiterm([2 3 2 0],Dzw);
    lmiterm([2 3 3 0],-1);
    
    lmisys = getlmis;
    options = [1e-7,2000,0,200,1];
    
    c = zeros(decnbr(lmisys),1);
    c( diag( decinfo(lmisys,X) ) ) = -1;

    [copt,xopt] = mincx(lmisys,c,options);
    
    if ~isempty(xopt)
        var.P = inv(dec2mat(lmisys,xopt,X));
        K = -dec2mat(lmisys,xopt,L) / dec2mat(lmisys,xopt,X);
        norma = copt;
    else
        var.P = [];
        K = NaN;
        norma = inf;
    end

end