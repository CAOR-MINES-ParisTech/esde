function [xe,Pe,lm_seq] = update_rc(xe,Pe,lm_seq,z,R,v_m,omega_m)
lenz = length(find(z(3,:)>0));
J = [0 -1; 1 0];
nf = 0;
% % update
for i= 1:lenz
    % data association (based on landmark id)
    is_exist = ~(lm_seq - z(3,i));
    idx = find(is_exist);
    ii = 2*i+(-1:0);    
    % update: already in the state vecor
    if ~isempty(idx)
        nf = nf+1;
        jj = 2*nf+(-1:0);
        
        [zhat,Hii] = measurement_model_rc(xe,v_m,omega_m,idx);   %Hekf=Hii
        H(jj,:) = Hii;
        r(jj,1) = [ z(1,i)-zhat(1,1);  (z(2,i)-zhat(2,1)) ];
        Rf(jj,jj) = R(ii,ii);
        
    end
end

if nf~=0
    S = H*Pe*H'+ Rf;
    S = (S+S')*0.5;
    
    K = Pe*H'/S;

    innov = K*r;
    xe = xe + innov(1:end-3);
    omega_obs_filtered = omega_m + innov(end-2);
    u_obs_filtered = [v_m;0] + innov(end-1:end);
    Pe = (eye(length(Pe)) - K*H) * Pe *(eye(length(Pe)) - K*H)' + K*Rf*K';
    xe0 = xe;
    xe(3) = xe(3) - omega_obs_filtered;
    xe(1:2) = rot(omega_obs_filtered)' * (xe(1:2)-u_obs_filtered);
    NbFeatures = (length(xe)-3)/2;
    temp =  rot(omega_obs_filtered)' *(reshape(xe(4:end),2,NbFeatures) -  repmat(u_obs_filtered,1,NbFeatures));
    xe(4:end) = temp(:);

    A = eye(length(xe),length(xe));
    A(1:2,1:2) = rot(omega_obs_filtered)';
    AA = zeros(length(xe),3);
    for jj = 1:(length(xe)-3)/2
        A(3+2*jj-1:3+2*jj,3+2*jj-1:3+2*jj)= rot(omega_obs_filtered)';
        AA(3+2*jj-1:3+2*jj,:)= [-rot(omega_obs_filtered)'*J*(xe0(3+2*jj-1:3+2*jj)-u_obs_filtered),...
            -rot(omega_obs_filtered)'];
    end
    AA(3,end-2) = -1;
    AA(1:2,:) = [-rot(omega_obs_filtered)'*J*(xe0(1:2)-u_obs_filtered),-rot(omega_obs_filtered)'];
    A=[A,AA];
    Pe = A*Pe*A';
else
    omega_obs_filtered = omega_m;
    u_obs_filtered = [v_m;0];
    xe0 = xe;
    xe(3) = xe(3) - omega_obs_filtered;
    xe(1:2) = rot(omega_obs_filtered)' * (xe(1:2)-u_obs_filtered);
    NbFeatures = (length(xe)-3)/2;
    temp =  rot(omega_obs_filtered)' *(reshape(xe(4:end),2,NbFeatures) -  repmat(u_obs_filtered,1,NbFeatures));
    xe(4:end) = temp(:);
    A = eye(length(xe),length(xe));
    A(1:2,1:2) = rot(omega_obs_filtered)';
    AA = zeros(length(xe),3);
    for jj = 1:(length(xe)-3)/2
        A(3+2*jj-1:3+2*jj,3+2*jj-1:3+2*jj)= rot(omega_obs_filtered)';
        AA(3+2*jj-1:3+2*jj,:)= [-rot(omega_obs_filtered)'*J*(xe0(3+2*jj-1:3+2*jj)-u_obs_filtered),...
            -rot(omega_obs_filtered)'];
    end
    
    AA(3,end-2) = -1;
    AA(1:2,:) = [-rot(omega_obs_filtered)'*J*(xe0(1:2)-u_obs_filtered),-rot(omega_obs_filtered)'];
    A=[A,AA];
    Pe = A*Pe*A';
end

% % augment
for i= 1:lenz
    % data association (known)
    is_exist = ~(lm_seq - z(3,i));
    idx = find(is_exist);
    
    lenx= size(xe,1);
    ii = 2*i + (-1:0);
    
    % add the new landmark into the state vector
    if isempty(idx)
        lm_seq = [lm_seq; z(3,i)];
        % augment state
        x_L = z(1:2,i);
        xe = [xe; x_L];     
        % augment covariance
        rng= lenx+1:lenx+2;
        HL = rot(omega_obs_filtered)';
        P_LL = inv(HL)*R(ii,ii)*inv(HL)';
        Pe = blkdiag(Pe,P_LL);
        Pe = 0.5*(Pe+Pe');
    end
end
end
