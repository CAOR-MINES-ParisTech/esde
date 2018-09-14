% UTIAS Multi-Robot Cooperative Localization and Mapping Dataset
% produced by Keith Leung (keith.leung@robotics.utias.utoronto.ca) 2009
% Matlab script animateMRCLAMdataSet.m
% Description: This scripts samples the dataset at fixed intervals 
% (default is 0.02s). Odometry data is interpolated using the recorded time. % Measurements are rounded to the nearest timestep. 
% Run this script after loadMRCLAMdataSet.m

% Option %
sample_time = 0.02;
% Option END %

min_time = Robot1_Groundtruth(1,1);
max_time = Robot1_Groundtruth(end,1);
for n=2:n_robots
   eval(['min_time = min(min_time, Robot' num2str(n) '_Groundtruth(1,1));']);
   eval(['max_time = max(max_time, Robot' num2str(n) '_Groundtruth(end,1));']);
end
for n=1:n_robots
    eval(['Robot' num2str(n) '_Groundtruth(:,1) = Robot' num2str(n) '_Groundtruth(:,1) - min_time;']);
    eval(['Robot' num2str(n) '_Measurement(:,1) = Robot' num2str(n) '_Measurement(:,1) - min_time;']);
    eval(['Robot' num2str(n) '_Odometry(:,1) = Robot' num2str(n) '_Odometry(:,1) - min_time;']);
end
max_time = max_time - min_time;
timesteps = floor(max_time/sample_time)+1;

disp(['time ' num2str(min_time) 'is the first timestep (t=0[s])']);
disp(['sampling time is ' num2str(sample_time) '[s] (' num2str(1/sample_time) '[Hz])']);
disp(['number of resulting timesteps is ' num2str(timesteps)]);

array_names = { 'Robot1_Groundtruth'; 'Robot1_Odometry'; 
                'Robot2_Groundtruth'; 'Robot2_Odometry'; 
                'Robot3_Groundtruth'; 'Robot3_Odometry'; 
                'Robot4_Groundtruth'; 'Robot4_Odometry'; 
                'Robot5_Groundtruth'; 'Robot5_Odometry'};
oldData = 0;
for name = 1:length(array_names)
    disp(['sampling ' array_names{name}])
    eval(['oldData =' array_names{name} ';'])

    k = 0;
    t = 0;
    i = 1;
    p = 0;

    [nr,nc] = size(oldData);
    newData = zeros(timesteps,nc);
    while(t <= max_time)
        newData(k+1,1) = t;     
        while(oldData(i,1) <= t)      
            if(i==nr)
                break;
            end
            i = i + 1;
        end
        if(i == 1 || i == nr)
            if(isempty(strfind(array_names{name},'Odo')))
                newData(k+1,2:end) = oldData(i,2:end);
            else
                newData(k+1,2:end) = 0;
            end
        else
            p = (t - oldData(i-1,1))/(oldData(i,1) - oldData(i-1,1));
            if(nc == 8) % i.e. ground truth data
                sc = 3;
                newData(k+1,2) = oldData(i,2); % keep id number 
            else
                sc = 2;
            end
            for c = sc:nc
                if(nc==8 && c>=6)
                    d = oldData(i,c) - oldData(i-1,c);
                    if d > pi
                        d = d - 2*pi;
                    elseif d < -pi
                        d = d + 2*pi;
                    end
                    newData(k+1,c) = p*d + oldData(i-1,c);
                else
                    newData(k+1,c) = p*(oldData(i,c) - oldData(i-1,c)) + oldData(i-1,c);
                end
            end
        end
        k = k + 1;
        t = t + sample_time;
    end

    eval([array_names{name} '= newData ;'])
end

array_names = { 'Robot1_Measurement'; 
                'Robot2_Measurement'; 
                'Robot3_Measurement';
                'Robot4_Measurement';
                'Robot5_Measurement'};
oldData = 0;
for name = 1:length(array_names)
    disp(['processing ' array_names{name}])
    eval(['oldData =' array_names{name} ';'])
    newData=oldData;
    for i = 1:length(oldData)
        newData(i,1) = floor(oldData(i,1)/sample_time + 0.5)*sample_time; 
    end
    eval([array_names{name} '= newData ;'])
end

clear min_time oldData newData nr nc n p sc t k c i d array_names name;

