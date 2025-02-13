% Distributed_Observer - Function to estimate the state of a vehicle in a distributed observer network.
%
% Syntax: x_hat_i_j = Distributed_Observer(i, j, A, B, x_bar_j, x_hat_i_j, u_j, W)

% Author: Quang Huy Nugyen
% Email: [huyq1471@gmail.com]
% Date: [14-1-2025]
% Version: 1.0

function x_hat_i_j = Distributed_Observer(i ,  j , A , B , x_bar_j , x_hat_i_j, u_j , W )
    % Inputs:
    %    i - Index of the vehicle to which this observer belongs.
    %    j - Index of the vehicle being estimated.
    %    A - State transition matrix.
    %    B - Control input matrix.
    %    x_bar_j - Local state of the vehicle being estimated.
    %    x_hat_i_j - Matrix containing the estimated states from all vehicles.
    %    u_j - Control input for the vehicle being estimated.
    %    W - Weight matrix for consensus calculation.
    %
    % Outputs:
    %    x_hat_i_j - Updated state estimate for vehicle j by observer in vehicle i.
    %
    % Example:
    %    x_hat_i_j = Distributed_Observer(1, 2, A, B, x_bar_j, x_hat_i_j, u_j, W)
    %
    %


    % estimate is the same . EX : vehicle 1 , give esimate vehicle 1
    %                               vehicle 1  , give esimate vehicle 2
    %                               vehicle 1  , give esimate vehicle 3
    %                               vehicle 1  , give esimate vehicle 4



    Sig = [0 ; 0 ;0]; % Initialize consensus term

    % Calculate the consensus term
    % Start from 2 , because the first element is the local state of the vehicle
    for l = 2:length(W)
        % l-1 because start from 2 , but we need to start from 1 for vehicle 
        Sig = W(2)*( x_hat_i_j(1:3 , l-1) - x_hat_i_j(1:3 , l-1) ) + Sig;
    end

    % Only if we try to estimated j = i , so we have real local state that was estimated by Local observer .
    % But if we try to estimate j != i , we have (fake) the local state of j , that by vehicle j send to i vehicle

    % Estimate the j vehicle , in i vehicle 
    % So the observer is implement in i vehicle
    w_i0 = W(1); % Weight of the local state
    
    %% Original paper (TRUE WORK)
    % x_hat_i_j = A*( x_hat_i_j(1:3 , j) + Sig + w_i0 * (x_bar_j - x_hat_i_j(1:3 , j)) ) + B*u_j ; 

    %% Test huy (no controller , and A not multiply with compenstate communication term)
    % Its still work , but the result is not good as the original paper

    % x_hat_i_j = A*( x_hat_i_j(1:3 , j)) + Sig + w_i0 * (x_bar_j - x_hat_i_j(1:3 , j))   ; 

    % x_hat_i_j = A*( x_hat_i_j(1:3 , j)) + Sig + w_i0 * (x_bar_j - x_hat_i_j(1:3 , j)) + B*u_j  ;

    %% Test huy (Dont have term Agent i Uses Local Measurements of Agent j)
    % Not work , because inital state the distributed observer is 0 
    % So its need some term to correct the initial state of the distributed observer 
    % And so on its can work , (Just assumtion , not sure) 
    % x_hat_i_j = A*( x_hat_i_j(1:3 , j)) + Sig  + B*u_j; 

    %% Test huy ( have term like classic observer)
    % when i = j , the observer is the same as the classic observer
    % ------ Its seem work !!!!!!
    if ( i == j )
        x_hat_i_j = A*( x_hat_i_j(1:3 , j) + Sig  + w_i0 * (x_bar_j - x_hat_i_j(1:3 , j)) ) + B*u_j; 
    else
        x_hat_i_j = A*( x_hat_i_j(1:3 , j) + Sig ) +  B*u_j  ;
    end
    
    
end