function idx = roulette_wheel(probabilities)
    % Normalize the probabilities
    probabilities = probabilities(:)';  % Ensure it's a row vector
    total = sum(probabilities);

    % Handle edge case: zero or NaN total probability
    if total == 0 || any(isnan(probabilities))
        idx = randi(length(probabilities));  % Random fallback
        return;
    end

    probabilities = probabilities / total;  % Normalize

    cumulative = cumsum(probabilities);
    r = rand;

    idx = find(r <= cumulative, 1, 'first');

    % Fallback: in case cumulative doesn't reach 1 due to rounding
    if isempty(idx)
        [~, idx] = max(probabilities);
    end
end
