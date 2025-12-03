function F = EightPointsAlgorithmN (P1, P2)

    [nP1, T1] = normalise2dpts(P1); % Normalize points from image 1
    [nP2, T2] = normalise2dpts(P2); % Normalize points from image 2

    % Compute the normalized fundamental matrix using the normalized points
    F_normalized = EightPointsAlgorithm(nP1, nP2);

    % Denormalize the Fundamental Matrix
    F = T2' * F_normalized * T1;
    
end