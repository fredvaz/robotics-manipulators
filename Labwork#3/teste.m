t = 0 ;
x = 0 ;
startSpot = 0;
interv = 1000 ; % considering 1000 samples
step = 0.1 ; % lowering step has a number of cycles and then acquire more data
while ( t <interv )
    b = sin(t)+5;
    x = [ x, b ];
    plot(x) ;
      if ((t/step)-500 < 0)
          startSpot = 0;
      else
          startSpot = (t/step)-500;
      end
      axis([ startSpot, (t/step+50), 0 , 10 ]);
      grid
      t = t + step;
      drawnow;
      pause(0.01)
end
  
%%
x = 1:0.01:25;
y = sin(x);
n = numel(x);
figure
xlim([0 25])
ylim([-1.1 1.1])
hold on
for i = 1:n
    plot(x(1:i),y(1:i))
    pause(0.05)
end