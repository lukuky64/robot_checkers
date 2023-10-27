%%
object = Blackout();

%%
while true
    status = object.activated();
    display(status);
    pause(0.5);
end

function updatePosition(self, newPosition)
    % Get the handle to the surface plot
    h = findobj('Type', 'surface');
    
    % Update the XData, YData, and ZData properties
    set(h, 'XData', new_x, 'YData', new_y, 'ZData', new_z);
    
    % Redraw the plot
    drawnow;
end