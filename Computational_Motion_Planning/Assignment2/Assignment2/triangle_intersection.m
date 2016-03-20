function flag = triangle_intersection(P1, P2)
% P1, P2: are 3 by 2 arrays (each), describing the vertices of a triangle, 
% the first column corresponds to the x coordinates while the second column corresponds to the y coordinates
% triangle_test : returns true if the triangles overlap and false otherwise
% *******************************************************************
   
    % Check if the first point of the second triangle P2 are inside the
    % first triangle --pads each points with a 0 beacuse cross product
    % function expects a 3D vector
    P2_11 = same_side(P2(1,1:2), P1(1,1:2), P1(2,1:2), P1(3,1:2));
    P2_12 = same_side(P2(1,1:2), P1(2,1:2), P1(1,1:2), P1(3,1:2));
    P2_13 = same_side(P2(1,1:2), P1(3,1:2), P1(1,1:2), P1(2,1:2));
    
    if (P2_11 && P2_12 && P2_13)
        flag = true;
        return
    end
    
    P2_21 = same_side(P2(2,1:2), P1(1,1:2), P1(2,1:2), P1(3,1:2));
    P2_22 = same_side(P2(2,1:2), P1(2,1:2), P1(1,1:2), P1(3,1:2));
    P2_23 = same_side(P2(2,1:2), P1(3,1:2), P1(1,1:2), P1(2,1:2));
    
    if (P2_21 && P2_22 && P2_23)
        flag = true;
        return
    end
%     
    P2_31 = same_side(P2(3,1:2), P1(1,1:2), P1(2,1:2), P1(3,1:2));
    P2_32 = same_side(P2(3,1:2), P1(2,1:2), P1(1,1:2), P1(3,1:2));
    P2_33 = same_side(P2(3,1:2), P1(3,1:2), P1(1,1:2), P1(2,1:2)); 
    
    if (P2_31 && P2_32 && P2_33)
        flag = true;
        return
    end
    
    %------------------------------------%
    
    P1_11 = same_side(P1(1,1:2), P2(1,1:2), P2(2,1:2), P2(3,1:2));
    P1_12 = same_side(P1(1,1:2), P2(2,1:2), P2(1,1:2), P2(3,1:2));
    P1_13 = same_side(P1(1,1:2), P2(3,1:2), P2(1,1:2), P2(2,1:2));
    
    if (P1_11 && P1_12 && P1_13)
        flag = true;
        return
    end
    
    P1_21 = same_side(P1(2,1:2), P2(1,1:2), P2(2,1:2), P2(3,1:2));
    P1_22 = same_side(P1(2,1:2), P2(2,1:2), P2(1,1:2), P2(3,1:2));
    P1_23 = same_side(P1(2,1:2), P2(3,1:2), P2(1,1:2), P2(2,1:2));
    
    if (P1_21 && P1_22 && P1_23)
        flag = true;
        return
    end
%     
    P1_31 = same_side(P1(3,1:2), P2(1,1:2), P2(2,1:2), P2(3,1:2));
    P1_32 = same_side(P1(3,1:2), P2(2,1:2), P2(1,1:2), P2(3,1:2));
    P1_33 = same_side(P1(3,1:2), P2(3,1:2), P2(1,1:2), P2(2,1:2)); 
    
    if (P1_31 && P1_32 && P1_33)
        flag = true;
        return
    end
%     if (P2_11 && P2_12 && P2_13)
%         flag = true;
%         return
%     elseif (P2_21 && P2_22 && P2_23)
%         flag = true;
%         return
%     elseif (P2_31 && P2_32 && P2_33)
%         flag = true;
%         return
%     else
%         flag = false;
%     end

flag = false;
end


%Implements tehnique to find on which side of a vector between two points
%the given point lies. 

 %A B C forms a triangle and all the points inside. 
 %Lines AB, BC, and CA each split space in half and one of those halves is entirely outside the triangle. 
 %For a point to be inside the traingle A B C it must be below AB and left of BC and right of AC. 
 %If any one of these tests fails we can return early.
 
 %If you take the cross product of [B-A] and [p-A], you'll get a vector pointing out of the screen. 
 %On the other hand, if you take the cross product of [B-A] and [p'-A] you'll get a vector pointing into the screen. 
 %Ah ha! In fact if you cross [B-A] with the vector from A to any point above the line AB, the resulting vector points out-
 %-of the screen while using any point below AB yields a vector pointing into the screen. 
 %So all we need to do to distinguish which side of a line a point lies on is take a cross product.

%Because the triangle can be oriented in any way in 3d-space, there isn't some set value we can compare with. 
%Instead what we need is a reference point - a point that we know is on a certain side of the line. 
%For our triangle, this is just the third point C.

%So, any point p where [B-A] cross [p-A] does not point in the same direction as [B-A] cross [C-A] isn't inside the triangle. 
%If the cross products do point in the same direction, then we need to test p with the other lines as well. 
%If the point was on the same side of AB as C and is also on the same side of BC as A and on the same side of CA as B, then it is in the triangle.
function side = same_side(p, c, a, b) 
    % cross product of a 2d vecotr is a scalar determinant below
    %^ x × ^ y = x 1 y 2 ? x 2 y 1 
    % cross product of a 2d vecotr is a
    %scalar determinant
    x = b - a;
    y = p - a;
    w = c - a;
    cp1 = x(1)*y(2) - x(2)*y(1); %cross(b-a, p-a);
    cp2 = x(1)*w(2) - x(2)*w(1); %cross(b-a, c-a);
    
    if (dot(cp1, cp2) >= 0)
        side = true;    
    else
        side = false;
    end
end
