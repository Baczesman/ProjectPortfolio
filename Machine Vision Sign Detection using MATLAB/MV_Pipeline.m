%version
%MATLAB R2022b

%load datasets
load('Dataset_R');
load('Dataset_B');
load('Dataset_W');


%% loading the image/video
%video has a end at the end tht needs to be commented

%making a txt;
fid = fopen( 'results.txt','wt');
fprintf( fid, "dashcam_clip_01.mp4");
fprintf( fid, "\nframe, sign, x, y, width, height");

%set variables
frame = 0;
step = 1;
delta = 0.1;   %max acceptable error
min_shape_size =  (size(source, 1)+size(source,2))/10;
Signs = struct('Shape',"",'BoundingBox',[0,0,0,0], 'Error', []);
mask_multiplier = (size(source,1)+size(source,2))/250;



         video = VideoReader("dashcam_clip_01.mp4");
             while hasFrame(video)
        source = readFrame(video);
        %source = imread("image");
        source = imgaussfilt(source,1);     %blur

frame  = frame+1;

%% creating r/g/b masks (for rgb values possible for signs)


img_hsv = rgb2hsv(source);

%changing S and V to be dynamic (depending on image) (normalizing s and v)

img_hsv(:,:,2) = histeq(img_hsv(:,:,2));
img_hsv(:,:,3) = histeq(img_hsv(:,:,3));

%splitting layers r g b
img_HR = (~imbinarize(img_hsv(:,:,1),25/360) | imbinarize(img_hsv(:,:,1),320/360)) & imbinarize(img_hsv(:,:,2),50/100) & imbinarize(img_hsv(:,:,3),40/100);
img_HB = (imbinarize(img_hsv(:,:,1),180/360) & ~imbinarize(img_hsv(:,:,1),270/360)) & imbinarize(img_hsv(:,:,2),50/100) & imbinarize(img_hsv(:,:,3),40/100);
img_HW = (~imbinarize(img_hsv(:,:,2),70/100) & imbinarize(img_hsv(:,:,3),30/100));

%(eroding decreased accuracy)
%eroding masks a bit
% se = strel('sphere',1);
% img_HR = imerode(img_HR,se);
% img_HB = imerode(img_HB,se);
% se = strel('sphere',1);
% img_HW = imdilate(img_HW,se);



%% cleaning up filters (removing small objects < x pixels)

cc = bwconncomp(img_HR,4);
for i = 1:cc.NumObjects
    if cellfun(@(x)numel(x),cc.PixelIdxList(i)) <= min_shape_size
        img_HR(cc.PixelIdxList{i}) = 0;
    end
end

cc = bwconncomp(img_HB,4);
for i = 1:cc.NumObjects
    if cellfun(@(x)numel(x),cc.PixelIdxList(i)) <= min_shape_size
        img_HB(cc.PixelIdxList{i}) = 0;
    end
end

%% getting bounding boxes for each detected object, red and blue
cc = bwconncomp(img_HR,4);           %using this we can specify connectivity for regionprops
Red_regions = regionprops(cc,'BoundingBox');
cc = bwconncomp(img_HB,4);
Blue_regions = regionprops(cc,'BoundingBox');


%% expanding bounding box by a few pixels
for i = 1:size(Red_regions,1)
    Red_regions(i).BoundingBox(1) = Red_regions(i).BoundingBox(1) - mask_multiplier;
    Red_regions(i).BoundingBox(2) = Red_regions(i).BoundingBox(2) - mask_multiplier;
    Red_regions(i).BoundingBox(3) = Red_regions(i).BoundingBox(3) + 2*mask_multiplier;
    Red_regions(i).BoundingBox(4) = Red_regions(i).BoundingBox(4) + 2*mask_multiplier;
end

for i = 1:size(Blue_regions,1)
    Blue_regions(i).BoundingBox(1) = Blue_regions(i).BoundingBox(1) - mask_multiplier;
    Blue_regions(i).BoundingBox(2) = Blue_regions(i).BoundingBox(2) - mask_multiplier;
    Blue_regions(i).BoundingBox(3) = Blue_regions(i).BoundingBox(3) + 2*mask_multiplier;
    Blue_regions(i).BoundingBox(4) = Blue_regions(i).BoundingBox(4) + 2*mask_multiplier;
end



%%  splitting each red and blue object and checking and comparing its signature
%This function takes every region with a red object and checks its distance
%signature, if it matches one from the database it stores it along with
%some details, to then decide what sign it is. This is done for blue and
%red objects

%% red

for i = 1:size(Red_regions,1)
    maskRed = imcrop(source, (Red_regions(i).BoundingBox));

    %getting mask of region
    mask_HR = imcrop(img_HR, (Red_regions(i).BoundingBox));
    mask_HW = imcrop(img_HW, (Red_regions(i).BoundingBox));
    mask_WH = mask_HW & imfill(mask_HR, 'holes');

    %% removing small objects

    cc = bwconncomp(mask_HR,4);
    for j = 1:cc.NumObjects
        if cellfun(@(x)numel(x),cc.PixelIdxList(j)) <= min_shape_size
            mask_HR(cc.PixelIdxList{j}) = 0;
        end
    end

    cc = bwconncomp(mask_HW,4);
    for j = 1:cc.NumObjects
        if cellfun(@(x)numel(x),cc.PixelIdxList(j)) <= min_shape_size/4
            mask_HW(cc.PixelIdxList{j}) = 0;
        end
    end

    %%  getting red regions
    cc = bwconncomp(edge(mask_HR,'canny'),8);
    regions = regionprops(cc,'PixelList', 'Centroid');


    %% clear small edges from regions
    for j = (cc.NumObjects):-1:1
        if size(regions(j).PixelList,1) <= (size(mask_HR,1)+size(mask_HR,2))/4
            regions(j) = [];
        end
    end
    if isempty(regions)
        continue
    end

    %%  storing the distance angle signateure of each red object
    for j = 1:size(regions,1)
        for k = 1:size(regions(j).PixelList)
            Signature_R(j).Coords(k,1) = atan2( (regions(j).PixelList(k,1) - regions(j).Centroid(1)) , (regions(j).PixelList(k,2)- regions(j).Centroid(2) ) );
            Signature_R(j).Coords(k,2) = ( sqrt((regions(j).PixelList(k,1) - regions(j).Centroid(1))^2 + (regions(j).PixelList(k,2)- regions(j).Centroid(2))^2));
        end
    end

    %%  NORMALIZE   RESCALING values Y to be between 0 and 1
    for j = 1:size(regions,1)
        Signature_R(j).Coords(:,2) = rescale(Signature_R(j).Coords(:,2), 0, 1);
    end

    %%  Sorting data to be  Y from -pi to pi
    for j = 1:size(regions,1)
        Signature_R(j).Coords = sortrows(Signature_R(j).Coords,1);
    end

    %%  comparing values in region X = -pi, pi      Comparing signatuers with maybe a dataset?
    %calculating FOR each point in object in question, the distance between
    %point and a point within a x range (pi/100)? in the dataset and taking shortest distance,
    %this gives us how close the graph is to the dataset????  (/datapoints)

    distance = 12;      %the distance, set to 12 to be larger than possible max distance
    Distance_sum = 0;

    %a structure containing the BB and shapes (later added)
    %This loop compares the aquired angle/distance signature
    %to a pregenerated set to return the shapes of this
    %colour found in the image
    Signs(end+1).BoundingBox = Red_regions(i).BoundingBox;

    for j = 1:size(Signature_R,2)
        for l = 1:size(Dataset_R,2)
            for k = 1:step:size(Signature_R(j).Coords,1)
                temp_m = find(Dataset_R(l).Coords(:,1) > Signature_R(j).Coords(k)-0.1, 1);
                temp_m2 = find(Dataset_R(l).Coords(:,1) > Signature_R(j).Coords(k)+0.1, 1);
                if isempty(temp_m); temp_m = 1; end
                if isempty(temp_m2); temp_m2 = size(Dataset_R(l).Coords,1); end
                for m = temp_m:step:temp_m2     %this compares the y values for a similar x in the database
                    x = sqrt((Signature_R(j).Coords(k,1) - Dataset_R(l).Coords(m,1))^2 + (Signature_R(j).Coords(k,2) - Dataset_R(l).Coords(m,2))^2);
                    if (x < distance)
                        distance = x;       %smallest distance from point in test image to point in dataset image
                    end
                end
                Distance_sum = Distance_sum + distance;
                distance = 12;

            end
            error = Distance_sum/size(Signature_R(j).Coords,1);
            Distance_sum = 0;
            if error < delta    %if error is smaller than the aceptable threshold, save image data
                Signs(end).Shape(end+1) = Dataset_R(l).Shape;
                Signs(end).Error(end+1) = error;
            end

        end
    end
    clear Signature_R;  %next signature will not be same size, need to reset it

    %%  getting white regions
    cc = bwconncomp(edge(mask_HW,'canny'),8);
    regions = regionprops(cc,'PixelList', 'Centroid');

    %% clear small edges from regions
    for j = (cc.NumObjects):-1:1
        if size(regions(j).PixelList,1) <= (size(mask_HW,1)+size(mask_HW,2))/4
            regions(j) = [];
        end
    end
    if isempty(regions)
        continue
    end
    
    %%  storing the distance angle signateure of each red object

    for j = 1:size(regions,1)
        for k = 1:size(regions(j).PixelList)
            Signature_W(j).Coords(k,1) = atan2( (regions(j).PixelList(k,1) - regions(j).Centroid(1)) , (regions(j).PixelList(k,2)- regions(j).Centroid(2) ) );
            Signature_W(j).Coords(k,2) = ( sqrt((regions(j).PixelList(k,1) - regions(j).Centroid(1))^2 + (regions(j).PixelList(k,2)- regions(j).Centroid(2))^2));
        end
    end

    %%  NORMALIZE   RESCALING values Y to be between 0 and 1
    for j = 1:size(regions,1)
        Signature_W(j).Coords(:,2) = rescale(Signature_W(j).Coords(:,2), 0, 1);
    end

    %%  Sorting data to be  Y from -pi to pi
    for j = 1:size(regions,1)
        Signature_W(j).Coords = sortrows(Signature_W(j).Coords,1);
    end

    if exist("Dataset_W", 'var')      %test if dataset is loaded, else do not compare
    else
        "no dataset to compare values,";
        break;
    end

    distance = 12;      %the distance, set to 12 to be larger than possible max distance
    Distance_sum = 0;

    %a structure containing the BB and shapes
    %This loop compares the aquired angle/distance signature
    %to a pregenerated set to return the shapes of this
    %colour found in the image
    %Signs(end+1).BoundingBox = Red_regions(i).BoundingBox;
    if exist('Signature_W', 'var')
        for j = 1:size(Signature_W,2)
            for l = 1:size(Dataset_W,2)
                for k = 1:step:size(Signature_W(j).Coords,1)
                    temp_m = find(Dataset_W(l).Coords(:,1) > Signature_W(j).Coords(k)-0.1, 1);
                    temp_m2 = find(Dataset_W(l).Coords(:,1) > Signature_W(j).Coords(k)+0.1, 1);
                    if isempty(temp_m); temp_m = 1; end
                    if isempty(temp_m2); temp_m2 = size(Dataset_W(l).Coords,1); end
                    for m = temp_m:step:temp_m2     %this compares the y values for a similar x in the database
                        x = sqrt((Signature_W(j).Coords(k,1) - Dataset_W(l).Coords(m,1))^2 + (Signature_W(j).Coords(k,2) - Dataset_W(l).Coords(m,2))^2);
                        if (x < distance)
                            distance = x;       %smallest distance from point in test image to point in dataset image
                        end
                    end
                    Distance_sum = Distance_sum + distance;
                    distance = 12;

                end
                error = Distance_sum/size(Signature_W(j).Coords,1);
                Distance_sum = 0;
                if error < delta    %if error is smaller than the aceptable threshold, save image data
                    Signs(end).Shape(end+1) = Dataset_W(l).Shape;
                    Signs(end).Error(end+1) = error;
                end

            end
        end
        clear Signature_W;  %next signature will not be same size, need to reset it
    end
end
%% blue

for i = 1:size(Blue_regions,1)
    maskBlue = imcrop(source, (Blue_regions(i).BoundingBox));

    %getting mask of region
    mask_HB = imcrop(img_HB, (Blue_regions(i).BoundingBox));
    mask_HW = imcrop(img_HW, (Blue_regions(i).BoundingBox));
    mask_WH = mask_HW & imfill(mask_HB, 'holes');


    %% removing small objects

    cc = bwconncomp(mask_HB,4);
    for j = 1:cc.NumObjects
        if cellfun(@(x)numel(x),cc.PixelIdxList(j)) <= min_shape_size
            mask_HB(cc.PixelIdxList{j}) = 0;
        end
    end

    cc = bwconncomp(mask_HW,4);
    for j = 1:cc.NumObjects
        if cellfun(@(x)numel(x),cc.PixelIdxList(j)) <= min_shape_size/4
            mask_HW(cc.PixelIdxList{j}) = 0;
        end
    end

    %%  getting blue regions
    cc = bwconncomp(edge(mask_HB,'canny'),8);
    regions = regionprops(cc,'PixelList', 'Centroid');


    %% clear small edges from regions
    for j = (cc.NumObjects):-1:1
        if size(regions(j).PixelList,1) <= (size(mask_HB,1)+size(mask_HB,2))/4
            regions(j) = [];
        end
    end
    if isempty(regions)
        continue
    end
    
    %%  storing the distance angle signateure of each red object
    for j = 1:size(regions,1)
        for k = 1:size(regions(j).PixelList)
            Signature_B(j).Coords(k,1) = atan2( (regions(j).PixelList(k,1) - regions(j).Centroid(1)) , (regions(j).PixelList(k,2)- regions(j).Centroid(2) ) );
            Signature_B(j).Coords(k,2) = ( sqrt((regions(j).PixelList(k,1) - regions(j).Centroid(1))^2 + (regions(j).PixelList(k,2)- regions(j).Centroid(2))^2));
        end
    end

    %%  NORMALIZE   RESCALING values Y to be between 0 and 1
    for j = 1:size(regions,1)
        Signature_B(j).Coords(:,2) = rescale(Signature_B(j).Coords(:,2), 0, 1);
    end

    %%  Sorting data to be  Y from -pi to pi
    for j = 1:size(regions,1)
        Signature_B(j).Coords = sortrows(Signature_B(j).Coords,1);
    end

    %%  comparing values in region X = -pi, pi      Comparing signatuers with maybe a dataset?
    %calculating FOR each point in object in question, the distance between
    %point and a point within a x range (pi/100)? in the dataset and taking shortest distance,
    %this gives us how close the graph is to the dataset????  (/datapoints)

    distance = 12;      %the distance, set to 12 to be larger than possible max distance
    Distance_sum = 0;

    %a structure containing the BB and shapes
    %This loop compares the aquired angle/distance signature
    %to a pregenerated set to return the shapes of this
    %colour found in the image
    Signs(end+1).BoundingBox = Blue_regions(i).BoundingBox;

    for j = 1:size(Signature_B,2)
        for l = 1:size(Dataset_B,2)
            for k = 1:step:size(Signature_B(j).Coords,1)
                temp_m = find(Dataset_B(l).Coords(:,1) > Signature_B(j).Coords(k)-0.1, 1);
                temp_m2 = find(Dataset_B(l).Coords(:,1) > Signature_B(j).Coords(k)+0.1, 1);
                if isempty(temp_m); temp_m = 1; end
                if isempty(temp_m2); temp_m2 = size(Dataset_B(l).Coords,1); end
                for m = temp_m:step:temp_m2     %this compares the y values for a similar x in the database
                    x = sqrt((Signature_B(j).Coords(k,1) - Dataset_B(l).Coords(m,1))^2 + (Signature_B(j).Coords(k,2) - Dataset_B(l).Coords(m,2))^2);
                    if (x < distance)
                        distance = x;       %smallest distance from point in test image to point in dataset image
                    end
                end
                Distance_sum = Distance_sum + distance;
                distance = 12;

            end
            error = Distance_sum/size(Signature_B(j).Coords,1);
            Distance_sum = 0;
            if error < delta    %if error is smaller than the aceptable threshold, save image data
                Signs(end).Shape(end+1) = Dataset_B(l).Shape;
                Signs(end).Error(end+1) = error;

            end

        end
    end
    clear Signature_B;  %next signature will not be same size, need to reset it

    %%  getting white regions
    cc = bwconncomp(edge(mask_HW,'canny'),8);
    regions = regionprops(cc,'PixelList', 'Centroid');

    %% clear small edges from regions
    for j = (cc.NumObjects):-1:1
        if size(regions(j).PixelList,1) <= (size(mask_HW,1)+size(mask_HW,2))/4
            regions(j) = [];
        end
    end
    if isempty(regions)
        continue
    end

    %%  storing the distance angle signateure of each red object

    for j = 1:size(regions,1)
        for k = 1:size(regions(j).PixelList)
            Signature_W(j).Coords(k,1) = atan2( (regions(j).PixelList(k,1) - regions(j).Centroid(1)) , (regions(j).PixelList(k,2)- regions(j).Centroid(2) ) );
            Signature_W(j).Coords(k,2) = ( sqrt((regions(j).PixelList(k,1) - regions(j).Centroid(1))^2 + (regions(j).PixelList(k,2)- regions(j).Centroid(2))^2));
        end
    end

    %%  NORMALIZE   RESCALING values Y to be between 0 and 1
    for j = 1:size(regions,1)
        Signature_W(j).Coords(:,2) = rescale(Signature_W(j).Coords(:,2), 0, 1);
    end

    %%  Sorting data to be  Y from -pi to pi
    for j = 1:size(regions,1)
        Signature_W(j).Coords = sortrows(Signature_W(j).Coords,1);
    end

    if exist("Dataset_W", 'var')      %test if dataset is loaded, else do not compare
    else
        "no dataset to compare values,";
        break;
    end

    distance = 12;      %the distance, set to 12 to be larger than possible max distance
    Distance_sum = 0;

    %a structure containing the BB and shapes (later added)
    %This loop compares the aquired angle/distance signature
    %to a pregenerated set to return the shapes of this
    %colour found in the image
    %Signs(end+1).BoundingBox = Blue_regions(i).BoundingBox;
    if exist('Signature_W', 'var')
        for j = 1:size(Signature_W,2)
            for l = 1:size(Dataset_W,2)
                for k = 1:step:size(Signature_W(j).Coords,1)
                    temp_m = find(Dataset_W(l).Coords(:,1) > Signature_W(j).Coords(k)-0.1, 1);
                    temp_m2 = find(Dataset_W(l).Coords(:,1) > Signature_W(j).Coords(k)+0.1, 1);
                    if isempty(temp_m); temp_m = 1; end
                    if isempty(temp_m2); temp_m2 = size(Dataset_W(l).Coords,1); end
                    for m = temp_m:step:temp_m2     %this compares the y values for a similar x in the database
                        x = sqrt((Signature_W(j).Coords(k,1) - Dataset_W(l).Coords(m,1))^2 + (Signature_W(j).Coords(k,2) - Dataset_W(l).Coords(m,2))^2);
                        if (x < distance)
                            distance = x;       %smallest distance from point in test image to point in dataset image
                        end
                    end
                    Distance_sum = Distance_sum + distance;
                    distance = 12;

                end
                error = Distance_sum/size(Signature_W(j).Coords,1);
                Distance_sum = 0;
                if error < delta    %if error is smaller than the aceptable threshold, save image data
                    Signs(end).Shape(end+1) = Dataset_W(l).Shape;
                    Signs(end).Error(end+1) = error;
                end

            end
        end
        clear Signature_W;  %next signature will not be same size, need to reset it
    end
end


%at this point we know the centre, bounding box etc. of each shape containing a
%potential sign, now we use this to determin the sign

%% finding signs based on the detected shapes
Signs(1) = [];  %removing 1st element (empty, created when variable was created)

for i = size(Signs,2):-1:1
    if ~isstring(Signs(i).Shape)        %delete non-signs from structure
        Signs(i) = [];
    end
end
EndSigns = struct('Name',"",'BoundingBox',[0,0,0,0]);

%this function puts the most likely signs into EndSigns, which gets
%displayed
for i = 1:size(Signs,2) %checking each shape and chosing correct sign based on foun geometries
    sorted = sort(Signs(i).Error);
    for j = 1:size(Signs(i).Shape,2)
        smallest = find(Signs(i).Error == (sorted(j)));
        if(Signs(i).Shape(smallest)) == "TriangleUp"         %red triangle up /\
            for k = 1:size(Signs(i).Shape,2)
                smallest = find(Signs(i).Error == (sorted(k)));
                if(Signs(i).Shape(smallest)) == "DoubleBend"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Double Bend";
                    break
                end
                if(Signs(i).Shape(smallest)) == "DualCarriagewayEnd"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Dual Carriageway End";
                    break
                end
                if(Signs(i).Shape(smallest)) == "Roadworks"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Roadworks";
                    break
                end
                if(Signs(i).Shape(smallest)) == "TrafficLights"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Traffic Lights";
                    break
                end
                if(Signs(i).Shape(smallest)) == "Roundabout"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Roundabout";
                    break
                end
                if(Signs(i).Shape(smallest)) == "Ducks"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Ducks";
                    break
                end
            end
        end
        if(Signs(i).Shape(smallest)) == "TriangleDown"         %red triangle down \/
            EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
            EndSigns(end).Name = "Warning/Give way";
            break

        end
        if(Signs(i).Shape(smallest)) == "Stop"                %red stop
            for k = 1:size(Signs(i).Shape,2)
                smallest = find(Signs(i).Error == (sorted(k)));
                if(Signs(i).Shape(smallest)) == "S"              %red stop
                    for l = 1:size(Signs(i).Shape,2)
                        if(Signs(i).Shape(l)) == "T"              %if we find s and t it might be enough
                            EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                            EndSigns(end).Name = "Stop";
                            break
                        end
                    end
                end
            end
        end
        if(Signs(i).Shape(smallest)) == "CircleR"             %red circle
            for k = 1:size(Signs(i).Shape,2)
                smallest = find(Signs(i).Error == (sorted(k)));
                if(Signs(i).Shape(smallest)) == "NoEntry"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "No entry";
                    break
                end
                if(Signs(i).Shape(smallest)) == "Two"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "20MPH";
                    break
                end
                if(Signs(i).Shape(smallest)) == "Three"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "30MPH";
                    break
                end
                if(Signs(i).Shape(smallest)) == "Four"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "40MPH";
                    break
                end
                if(Signs(i).Shape(smallest)) == "Five"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "50MPH";
                    break
                end

            end
        end
        if(Signs(i).Shape(smallest)) == "OneWay"          %blue one way
            for k = 1:size(Signs(i).Shape,2)
                smallest = find(Signs(i).Error == (sorted(k)));
                if(Signs(i).Shape) == "OneWay"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "One way";
                    break
                end
            end
        end
        if(Signs(i).Shape(j)) == "CircleB"         %blue circle
            for k = 1:size(Signs(i).Shape,2)
                smallest = find(Signs(i).Error == (sorted(k)));
                if(Signs(i).Shape(smallest)) == "TurnLeft"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Turn Left";
                    break
                end
                if(Signs(i).Shape(smallest)) == "TurnRight"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Turn Right";
                    break
                end
                if(Signs(i).Shape(smallest)) == "KeepLeft"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Keep Left";
                    break
                end
                if(Signs(i).Shape(smallest)) == "KeepRight"       %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Keep Right";
                    break
                end
                if(Signs(i).Shape(smallest)) == "MiniRoundabout1"     %
                    EndSigns(end+1).BoundingBox = Signs(i).BoundingBox;
                    EndSigns(end).Name = "Mini Roundabout";
                    break
                end
            end
        end
    end
end






    clf;
    imshow (source);
    hold on;


%this removes duplicates, adapted from Jonathan Sullivan, https://uk.mathworks.com/matlabcentral/answers/76058-how-do-i-remove-duplicates-from-a-structure-array

isUnique = true(size(EndSigns));
for i = 1:length(EndSigns)-1
    for j = i+1:length(EndSigns)
        if isequal(EndSigns(i),EndSigns(j))
            isUnique(i) = false;
            break;
        end
    end
end
EndSigns(~isUnique) = [];

%plotting signs and titling it the name
 for i = 1:size(EndSigns,2)
%     rectangle('Position', [EndSigns(i).BoundingBox(1),EndSigns(i).BoundingBox(2),EndSigns(i).BoundingBox(3),EndSigns(i).BoundingBox(4)], 'EdgeColor',[1.0,0.0,0.0],'LineWidth',2)
%     text(EndSigns(i).BoundingBox(1) + EndSigns(i).BoundingBox(3)/2,EndSigns(i).BoundingBox(2)-20,EndSigns(i).Name,'HorizontalAlignment','center', Color='r', FontSize=12)
  fprintf( fid, '\n%f,%f,%f,%f,%f,%f', frame, EndSigns(i).Name, EndSigns(i).BoundingBox(1) +EndSigns(i).BoundingBox(3)/2, EndSigns(i).BoundingBox(2) +EndSigns(i).BoundingBox(4)/2,EndSigns(i).BoundingBox(3), EndSigns(i).BoundingBox(4) );
 end


%videoreader end
     end
fclose(fid);
