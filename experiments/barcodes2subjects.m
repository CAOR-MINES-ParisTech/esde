function [subjects] = barcodes2subjects(barcodes, Barcodes)
subjects = zeros(length(barcodes), 1);
for i = 1:length(barcodes)
    subjects(i) = find(Barcodes(:,2) == barcodes(i));
end
end
