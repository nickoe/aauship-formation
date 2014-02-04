h1 = figure(1); clf;

Xpos = 1:101;
Ypos = 1:101;
heading=sin(Xpos/30);
N = length(Xpos);
Nskip = 20;


% set(gca,'nextplot','replacechildren');
% set(gcf,'Renderer','zbuffer');


mode = 'plot';
% mode = 'anim';
if strcmp(mode,'anim')
    Nskip = 1; % Disable skips in animation
end
    hold on
    plot(Xpos,Ypos,'-')
    hold off
% Main loop
for i = 1:Nskip:length(Xpos)

    ship(Xpos(i),Ypos(i),heading(i),'r');
    ship(Xpos(i)+30,Ypos(i),heading(i),'y');

end
