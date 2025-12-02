function F = EightPointsAlgorithmN (P1, P2)
    A = zeros(length(P1), 9);

    for i=1:length(P1)
        A(i, 1) = P2(1,i) * P1(1,i);
        A(i, 2) = P2(1,i) * P1(2,i);
        A(i, 3) = P2(1,i);
        A(i, 4) = P2(2,i) * P1(1,i);
        A(i, 5) = P2(2,i) * P1(2,i);
        A(i, 6) = P2(2,i);
        A(i, 7) = P1(1,i);
        A(i, 8) = P1(2,i);
        A(i, 9) = 1;
    end

    
    % step2 -Compute the Singular Value Decomposition of A
    [~, ~, V] = svd(A);
    % The fundamental matrix F is the last column of V reshaped into a 3x3 matrix
    F = reshape(V(:, end), [3, 3]);
    F = F';
    % Enforce the rank-2 constraint on the fundamental matrix F
    [U, D, V] = svd(F);
    D(3, 3) = 0; % Set the smallest singular value to zero
    F = U * D * V'; % Recompute F with the rank-2 constraint
    
end