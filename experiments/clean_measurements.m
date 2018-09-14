function [measurements] = clean_measurements(measurements, Barcodes)
% remove uncorrect measurement
for i = length(measurements):-1:1
    if sum(measurements(i, 2) == Barcodes(:,2)) == 0
        measurements(i, :) = [];
    else
        if measurements(i, 3) > 6
            measurements(i, :) = [];
        end
    end
end
end