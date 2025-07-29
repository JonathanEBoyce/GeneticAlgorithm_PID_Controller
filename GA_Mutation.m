function mutated = GA_Mutation(individual, mutRate, bounds)
    
    sigma = (bounds(:,2) - bounds(:,1)).'; %max mutation size

    mutated = individual;  % Initialize output

    for gene = 1:3
        if rand < mutRate
            val = individual(gene) + sigma(gene) * randn;

            % Clip to bounds
            val = max(bounds(gene,1), min(bounds(gene,2), val));
            mutated(gene) = val;
        end
    end
end
