function [E] = Error(F, P1, P2)
    N = size(P1, 1);
    E = zeros(N,1);
    
    % Constraint
    for i=1:N
        E(i,1) = P2(:,i)'*F*P1(:,i);
    end

    disp(E)
end