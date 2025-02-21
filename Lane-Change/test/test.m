clear
clc

trust_score_1 = zeros(1, 1 , 3);
trust_score_1(1, 1 , 3) = 0.7;
trust_score_1(1, 1 , 1) = 0.7;

trust_score_2 = zeros(1, 1 , 3);
trust_score_2(1, 1 , 1) = 0.5;

received_trusts = [trust_score_1; trust_score_2];

non_zero_trusts = received_trusts(received_trusts(:,:,1) ~= 0); % Filter out zeros

