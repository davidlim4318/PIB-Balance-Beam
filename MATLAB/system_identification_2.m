clear

file_name = {"exp1.txt" "exp2.txt"};
n_exp = length(file_name);

delay_samples = [10 10];

data = cell(1,n_exp);

figure(1)
clf
tiledlayout(1,n_exp)

for i = 1:n_exp
    
    % Read data
    M = readmatrix(file_name{i},'TrimNonNumeric',true);
    M(:,1) = [];
    % Extract relevant data
    time = M(:,1)/1000;
    Ts = mean(diff(time));
    input = M(:,3);
    distance = M(:,2);
    speed = diff(distance)/Ts;
    speed_smooth = diff(smoothdata(distance,'lowess',30)/Ts);
    
    % Plot data
    nexttile(i)
    hold on
    yyaxis left
    plot(time,input,'.--','MarkerSize',10)
    yyaxis right
    plot(time,distance,'m.--','MarkerSize',10)
    plot(time(1:end-1),speed,'.--','MarkerSize',10)
    plot(time(1:end-1),speed_smooth,'-','LineWidth',2)
    yyaxis left
    ylabel("Angle (degrees)")
    yyaxis right
    ylabel("Distance (mm)")
    xlabel("Time (s)")
    grid on

    speed_no_delay = speed(delay_samples(i)+1:end);
    input_no_delay = input(1:end-delay_samples(i));

    % Encapsulate data for system identification, and save data in cell array
    data{i} = iddata(speed_no_delay, input_no_delay(1:end-1), Ts);
    data{i}.InputName = 'Angle';
    data{i}.OutputName = 'Speed';
    data{i}.TimeUnit = 'seconds';

end

%%
data_subset = data{2};

% Plot data
figure(2)
clf
plot(data_subset)
set(findall(gcf,'Type','Line'),'LineStyle','none','Marker','.','MarkerSize',10)

%%
Gss = ssest(data_subset, 1:10);

%%
% Estimate transfer function model
sysTF = tfest(data_subset,3,0,'TimeUnit','seconds');

% Validate transfer function model against data
figure(3)
clf

for i = 1:n_exp
    subplot(1,n_exp,i);
    compare(data{i},sysTF)
end

set(findall(gcf,'Type','Line'),'LineWidth',1,'Marker','.','MarkerSize',10)
set(findall(gcf,'Type','Legend'),'Location','Best')

%% 
%{
% Simulate transfer function model
t1 = 0:0.001:3;
u1 = 5*heaviside(t1-0.625) - 5*heaviside(t1-1.225) - 5*heaviside(t1-1.825) + 5*heaviside(t1-2.425);
t2 = 0:0.001:10;
u2 = 7*sin(2*pi*2/7*t2);
t3 = t2;
u3 = 7*sin(2*pi*3/7*t2);

u = {u1 u2 u3};
t = {t1 t2 t3};

figure(4)

for i = 1:n_exp
    subplot(1,n_exp,i);
    lsim(sysTF,u{i},t{i})
end

set(findall(gcf,'Type','Line'),'LineWidth',2)

%%
% Validate transfer function for distance
G = tf(sysTF.Numerator,[sysTF.Denominator 0]);

figure(5)

for i = 1:n_exp
    subplot(1,n_exp,i);
    lsim(G,u{i},t{i})
end

set(findall(gcf,'Type','Line'),'LineWidth',2)

%%
% Compare transfer function response with distance data

G = tf(sysTF.Numerator,[sysTF.Denominator 0]);

M = readmatrix(file_name{i},'TrimNonNumeric',true);
M(:,1) = [];

time = M(:,1)/1000;
Ts = mean(diff(time));
input = M(:,3);
distance = M(:,2);

distance_offset = distance - mean(distance(1:30))*ones(length(distance),1);
time_offset = time - time(1)*ones(length(time),1);

time_delay = .15;

T = 0:0.001:(time(end)-time(1));
U = 5*heaviside(T-0.575) - 5*heaviside(T-1.175) - 5*heaviside(T-1.775) + 5*heaviside(T-2.377);
Y = lsim(G,U,T);

figure(6)
clf
hold on
yyaxis left
plot(time_offset,input,'.','MarkerSize',10)
plot(T,U,'--','LineWidth',1)
yyaxis right
plot(time_offset - time_delay*ones(length(time),1),distance_offset,'.','MarkerSize',10)
plot(T,Y,'--','LineWidth',1)
yyaxis left
ylabel("Angle (degrees)")
yyaxis right
ylabel("Distance (mm)")
xlabel("Time (s)")
grid on
%}
%%
save('system_identification_2')
