function milestones = sample_map(nb_samples, map, map_min, map_max, pos_start, pos_end)
    map_range = map_max - map_min;
    samples_x = int32(map_range(1) * rand(nb_samples, 1) + map_min(1));
    samples_y = int32(map_range(2) * rand(nb_samples, 1) + map_min(2));
    samples = [samples_x, samples_y];

    milestones = [
        pos_start(1:2);
        pos_end
    ];
    basecheckpoints = [ [0, 1];
                        [1, 1];
                        [1, 0];
                        [1, -1];
                        [0, -1];
                        [-1, -1];
                        [-1, 0];
                        [-1, 1]
                        ];
    checkradius = 10;
    checkpoint_vec = basecheckpoints;
    for tt = 2:checkradius
        checkpoint_vec = [checkpoint_vec; tt*basecheckpoints];
    end
    
    resample_locations = [];
    for i = 1:length(samples)
        sample = samples(i, :);   
        hit = 0;
        if map(sample(1), sample(2)) == 0
            for j = 1:length(checkpoint_vec)
                x_check =  sample(1) + checkpoint_vec(j, 1);
                y_check =  sample(2) + checkpoint_vec(j, 2);
                if (x_check > map_max(1) || y_check > map_max(2) || ...
                        x_check < 1 || y_check < 1) 
                    hit = 1;                    
                     continue;
                end
                if (map(x_check, y_check) == 1)
                    hit = 1;
                    resample_locations = [resample_locations; [sample(1), sample(2)]];
                    continue;
                end
            end
            if (hit ~= 1)
                milestones = [milestones; sample];
            end
        end
    end
    
    %do some resampling
    map_range = checkradius*3;
    resamples = [];
    for i = 1:length(resample_locations)
        sample = resample_locations(i, :);
        resamples_x = int32(map_range * rand(3, 1) - map_range/2) + sample(1);
        resamples_y = int32(map_range * rand(3, 1) - map_range/2) + sample(2);        
        resamples = [resamples; [resamples_x, resamples_y]];
    end
    
    for i = 1:length(resamples)
        sample = resamples(i, :);
        x_check =  sample(1);
        y_check =  sample(2);
        if (x_check > map_max(1) || y_check > map_max(2) || ...
                x_check < 1 || y_check < 1)
            continue;
        end
        hit = 0;
        if map(sample(1), sample(2)) == 0
            for j = 1:length(checkpoint_vec)
                x_check =  sample(1) + checkpoint_vec(j, 1);
                y_check =  sample(2) + checkpoint_vec(j, 2);
                if (x_check > map_max(1) || y_check > map_max(2) || ...
                        x_check < 1 || y_check < 1)
                    hit = 1;
                    continue;
                end
                if (map(x_check, y_check) == 1)
                    hit = 1;                   
                    continue;
                end
            end
            if (hit ~= 1)
                milestones = [milestones; sample];
            end
        end
    end
    
end