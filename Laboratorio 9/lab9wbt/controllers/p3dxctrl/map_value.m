function mappedValue = map_value(value, fromLow, fromHigh, toLow, toHigh)
    % Adjust the value to the current range
    value = max(min(value, fromHigh), fromLow);
    
    % Calculate the new value in the target range
    mappedValue = (value - fromLow) .* (toHigh - toLow) ./ (fromHigh - fromLow) + toLow;
end