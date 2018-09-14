function [x_fit] = match_map(X_aux,Y_aux)
l_laps = size(X_aux,2);
shift_X = mean(X_aux,2);
X_centered = X_aux - repmat(shift_X,1,l_laps);

shift_Y = mean(Y_aux,2);
Y_centered = Y_aux - repmat(shift_Y,1,l_laps);

[U,D,V] = svd(X_centered*Y_centered');

x_fit = repmat(shift_Y,1,l_laps)+(V*U')*X_centered;
end