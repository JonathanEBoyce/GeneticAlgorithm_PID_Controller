function [c1, c2] = crossover(p1, p2, bounds)

    beta = 1.5;  % allow extrapolation
    alpha = (1 + 2*beta) * rand(1,3) - beta;  % 1x3 vector for per-gene blending

    % Vectorized extrapolation crossover
    c1 = p1 + alpha .* (p2 - p1);
    c2 = p2 - alpha .* (p2 - p1);

    % Clamp to bounds
    for i = 1:3
        c1(i) = max(bounds(i,1), min(bounds(i,2), c1(i)));
        c2(i) = max(bounds(i,1), min(bounds(i,2), c2(i)));
    end
end









% 
% function [c1, c2] = crossover(p1, p2)
%     beta = 1.5;  % allow extrapolation
%     alpha = (1 + 2*beta) * rand(1,3) - beta;  % 1x3 vector for per-gene blending
% 
%     % Vectorized extrapolation crossover
%     c1 = p1 + alpha .* (p2 - p1);
%     c2 = p2 - alpha .* (p2 - p1);
% end