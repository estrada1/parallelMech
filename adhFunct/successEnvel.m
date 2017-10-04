function success = successEnvel(FBD,adhLimit)
    succ1 = FBD(:,:,1) < adhLimit; 
    succ2 = FBD(:,:,2) < adhLimit;
    success = succ1 & succ2;  
end