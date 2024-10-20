clear a 

a = arduino();

buttonPressed = false;

while true
    
    state = readDigitalPin(a, 'D2');
    
    if state == 1 && buttonPressed == false
        % ESTOP activated
        % Code to stop robot
        buttonPressed = true;
    end
    
end