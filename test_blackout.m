%%
object = Blackout();

%%
while true
    status = object.activated();
    display(status);
    pause(0.5);
end
