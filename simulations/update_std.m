function [xe,Pe,lm_seq] = update_std(xe,Pe,lm_seq,z,R)
lenz = length(find(z(3,:)>0));
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
        [zhat,Hii] = measurement_model_std(xe,idx);   %Hekf=Hii
        H(jj,:) = Hii;
        r(jj,1) = [ z(1,i)-zhat(1,1);  (z(2,i)-zhat(2,1)) ];
        Rf(jj,jj) = R(ii,ii);       
    end
end

if nf~=0
    S = H*Pe*H'+ Rf;
    S = (S+S')*0.5;  
    K = Pe*H'/S;
    xe = xe + K*r;
    Pe = (eye(length(Pe)) - K*H) * Pe *(eye(length(Pe)) - K*H)' + K*Rf*K';
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
        k_xL = z(1:2,i);     
        C = rot(xe(3));
        x_L = xe(1:2,1) + C*k_xL;
        xe = [xe; x_L];
        
        % jacobians
        J = [0 -1; 1 0];        
        HR = - C'*[eye(2)  J*(x_L-xe(1:2,1))];
        HL = C';
        
        % augment covariance
        rng= lenx+1:lenx+2;
        Pe(rng,rng)= inv(HL)*HR*Pe(1:3,1:3)*HR'*inv(HL)' + inv(HL)*R(ii,ii)*inv(HL)'; % landmark cov
        Pe(rng,1:3)= -inv(HL)*HR*Pe(1:3,1:3); % landmark-robot xcorr
        Pe(1:3,rng)= Pe(rng,1:3)';
        if lenx>3
            rnm= 4:lenx;
            Pe(rng,rnm)= -inv(HL)*HR*Pe(1:3,rnm);
            Pe(rnm,rng)= Pe(rng,rnm)';
        end
        Pe = 0.5*(Pe+Pe');
    end
end

