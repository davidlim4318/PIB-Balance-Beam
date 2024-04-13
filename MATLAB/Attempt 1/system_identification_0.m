clear

figure(1)
clf
tiledlayout(1,3)

file_name = {"step_0+5_0-5_0_600ms.txt" "P0.08_D0.02.txt" "P0.10_D0.03.txt"};
n_exp = length(file_name);

input_offset = 0;
output_offset = [-33.2 -106 -111];
delay_samples = [11 11 11];

data = cell(1,n_exp);

for i = 1:n_exp
    
    % Read data
    M = readmatrix(file_name{i});
    % Extract relevant data
    time = M(:,1)/1000;
    input = M(:,3) - input_offset*ones(length(M),1);
    distance = M(:,2);
    
    % Plot data
    nexttile(i)
    hold on
    yyaxis left
    plot(time,input,'.--','MarkerSize',10)
    yyaxis right
    plot(time,distance,'.--','MarkerSize',10)
    yyaxis left
    ylabel("Angle (degrees)")
    yyaxis right
    ylabel("Distance (mm)")
    xlabel("Time (s)")
    grid on

    Ts = mean(diff(time));

    distance_no_delay = distance(delay_samples(i)+1:end) + output_offset(i)*ones(length(distance)-delay_samples(i),1);
    input_no_delay = input(1:end-delay_samples(i));

    % Encapsulate data for system identification, and save data in cell array
    data{i} = iddata(distance_no_delay, input_no_delay, Ts);
    data{i}.InputName = 'Angle';
    data{i}.OutputName = 'Distance';
    data{i}.TimeUnit = 'seconds';

end

%%
data_subset = data{1}(1:48);

% Plot data
figure(2)
clf
plot(data_subset)
set(findall(gcf,'Type','Line'),'LineStyle','none','Marker','.','MarkerSize',10)

%%
Gss = ssest(data_subset, 1:10);

%%
% Estimate transfer function model
sysTF = tfest(data_subset,6,0,'TimeUnit','seconds');

% Validate transfer function model against data
figure(3)
clf

for i = 1:n_exp
    subplot(1,n_exp,i);
    compare(data{i},sysTF)
end

set(findall(gcf,'Type','Line'),'LineWidth',1,'Marker','.','MarkerSize',10)
set(findall(gcf,'Type','Legend'),'Location','Best')

figure(4)
G = tf(sysTF.Numerator,sysTF.Denominator);

t1 = 0:0.001:3;
u1 = 5*heaviside(t1-0.625) - 5*heaviside(t1-1.125) - 5*heaviside(t1-1.625) + 5*heaviside(t1-2.125);
t2 = 0:0.001:10;
u2 = 7*sin(2*pi*2/7*t2);
t3 = t2;
u3 = 7*sin(2*pi*3/7*t2);

u = {u1 u2 u3};
t = {t1 t2 t3};

for i = 1:n_exp
    subplot(1,n_exp,i);
    lsim(sysTF,u{i},t{i})
end

set(findall(gcf,'Type','Line'),'LineWidth',2)

%%
save('system_identification_1')
