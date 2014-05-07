function [ output_args ] = annotatefill( start, stop, description, min, max )
%ANNOTATEFILL Fills the areas
%   Detailed explanation goes here
    for ii = 1:length(start)
        fill([start(ii),...
              start(ii),...
              stop(ii),...
              stop(ii)],...
             [max min min max],'y','FaceAlpha', 0.5)
        text(start(ii),max,description(ii),'rotation',90);
    end

end

