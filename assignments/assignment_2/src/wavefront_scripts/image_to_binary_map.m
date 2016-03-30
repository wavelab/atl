function [imout] = image_to_binary_map(I)
imout = im2bw(I, 0.7); % Convert to 0-1 image
imout = 1-flipud(imout)'; % Convert to 0 free, 1 occupied and flip.
end

