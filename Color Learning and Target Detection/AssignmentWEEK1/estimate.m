%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%
load('Samples.mat', 'Samples');
mu = mean(Samples);
sig = zeros(3,3);

N = size(Samples, 1);
for i=1:N
    data = double(Samples(i,:));
    sig = sig + (data - mu)' * (data - mu) / N;
end

save('mu.mat', 'mu');
save('sig.mat', 'sig');