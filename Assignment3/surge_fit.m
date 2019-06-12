
tstart=0;           % Sim start time
tstop=10000;        % Sim stop time
tsamp=1;           % Sampling time for how often states are stored. (NOT ODE solver time step)

delta_0 = 1;
delta_lim = (85*2*pi)/60;

surge_param = [0 0 0];

for i = delta_0: .5: delta_lim;
    tau = i;
    sim MSFartoystyring2017b;
    K = u_out(tstop/tsamp);
    K_t = 0.632*K;
    [~,T_index] = min(abs(u_out-K_t));
    T = T_index*tsamp;
    d = - 1/K;
    m = -T*d;
    surge_param = [surge_param; [tau d m]];
end
%%
surge_param = surge_param(2:end,:)
%%
figure();
plot(surge_param(:,1), surge_param(:,2))
%ylim([0.006 0.007]);
grid on
%%
figure();
plot(surge_param(:,1), surge_param(:,3))
%ylim([0.006 0.007]);
grid on